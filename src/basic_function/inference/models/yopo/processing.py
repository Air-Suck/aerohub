"""Shared YOPO CPU preprocessing and candidate decoding; no engine dependencies."""
import math
import cv2
import numpy as np

PREPROCESSING_VERSION = "ros-reference-u8-inpaint-v1"


def preprocess_depth(depth, width, height):
    """Match the ROS reference: resize, cap, quantize and inpaint invalid depth."""
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2 or not depth.size:
        raise ValueError("depth must be a nonempty matrix in meters")
    if depth.shape != (height, width):
        depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
    depth = np.minimum(depth, 20.0) / 20.0
    mask = ~np.isfinite(depth) | (depth < 0.04 / 20.0)
    with np.errstate(invalid="ignore"):
        quantized = np.uint8(depth * 255)
    repaired = cv2.inpaint(quantized, np.uint8(mask), 1, cv2.INPAINT_NS)
    return (repaired.astype(np.float32) / 255.0).reshape(1, 1, height, width)


class YopoProcessing:
    def __init__(self, data, velocity):
        base_vel_max = float(data["vel_max"])
        base_acc_max = float(data["acc_max"])
        base_segment_time = float(data["segment_time"])
        self.goal_length = float(data["goal_length"])
        self.yaw_diff = float(data["yaw_diff"])
        self.pitch_diff = float(data["pitch_diff"])
        self.radio_range = float(data["radio_range"])
        self.vertical_num = int(data["vertical_num"])
        self.horizon_num = int(data["horizon_num"])
        self.image_height = int(data["image_height"])
        self.image_width = int(data["image_width"])
        self.lattice_yaw = np.array(data["lattice_yaw"], dtype=np.float32)
        self.lattice_pitch = np.array(data["lattice_pitch"], dtype=np.float32)
        self.lattice_Rbp = np.array(data["lattice_Rbp"], dtype=np.float32)
        self.lattice_Rbp_flip = self.lattice_Rbp[::-1].copy()
        if not math.isfinite(velocity) or velocity <= 0 or velocity > base_vel_max + 1e-6:
            raise ValueError("YOPO_VELOCITY must be positive and <= trained {}".format(base_vel_max))
        ratio = velocity / base_vel_max
        self.vel_max = velocity
        self.acc_max = base_acc_max * ratio * ratio
        self.segment_time = base_segment_time / ratio

    @classmethod
    def from_file(cls, path, velocity):
        with np.load(path, allow_pickle=False) as data:
            return cls(data, velocity)

    @property
    def inputs(self):
        return {
            "depth": {"shape": [1, 1, self.image_height, self.image_width], "dtype": "float32"},
            "obs": {"shape": [1, 9, self.vertical_num, self.horizon_num], "dtype": "float32"},
        }

    @property
    def outputs(self):
        return {
            "predictions": {"shape": [1, 9, self.vertical_num, self.horizon_num], "dtype": "float32"},
            "scores": {"shape": [1, self.vertical_num, self.horizon_num], "dtype": "float32"},
        }

    def prepare_observation(self, state):
        normalized = np.asarray(state, dtype=np.float32).reshape(1, 9).copy()
        if not np.isfinite(normalized).all():
            raise ValueError("state must contain 9 finite values")
        normalized[:, 0:3] /= self.vel_max
        normalized[:, 3:6] /= self.acc_max
        goal_norm = np.linalg.norm(normalized[:, 6:9], axis=1, keepdims=True)
        normalized[:, 6:9] /= np.maximum(goal_norm, self.goal_length)
        transformed = np.matmul(normalized.reshape(1, 1, 3, 3), self.lattice_Rbp_flip)
        return np.ascontiguousarray(transformed.reshape(1, -1, 9).transpose(0, 2, 1)).reshape(
            1, 9, self.vertical_num, self.horizon_num
        )

    def decode(self, predictions, scores):
        predictions = np.asarray(predictions, dtype=np.float32).reshape(9, -1).T
        scores = np.asarray(scores, dtype=np.float32).reshape(-1)
        if len(predictions) != len(scores) or not len(scores):
            raise RuntimeError("model returned inconsistent candidate dimensions")
        lattice_id = np.arange(len(scores) - 1, -1, -1)
        delta_yaw = predictions[:, 0] * self.yaw_diff
        delta_pitch = predictions[:, 1] * self.pitch_diff
        radio = (predictions[:, 2] + 1.0) * self.radio_range
        yaw = self.lattice_yaw[lattice_id]
        pitch = self.lattice_pitch[lattice_id]
        positions = np.stack(
            (
                np.cos(pitch + delta_pitch) * np.cos(yaw + delta_yaw) * radio,
                np.cos(pitch + delta_pitch) * np.sin(yaw + delta_yaw) * radio,
                np.sin(pitch + delta_pitch) * radio,
            ),
            axis=1,
        )
        velocities = np.matmul(
            self.lattice_Rbp[lattice_id],
            (predictions[:, 3:6] * self.vel_max)[:, :, None],
        ).squeeze(-1)
        accelerations = np.matmul(
            self.lattice_Rbp[lattice_id],
            (predictions[:, 6:9] * self.acc_max)[:, :, None],
        ).squeeze(-1)
        endstates = np.concatenate((positions, velocities, accelerations), axis=1)
        if not np.isfinite(endstates).all() or not np.isfinite(scores).all():
            raise RuntimeError("model produced nonfinite predictions")
        return endstates, scores, int(scores.argmin())
