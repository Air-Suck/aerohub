import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BodyTwist:
    linear: np.ndarray
    angular: np.ndarray


def _rotation_log(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    vee = np.array(
        [rotation[2, 1] - rotation[1, 2],
         rotation[0, 2] - rotation[2, 0],
         rotation[1, 0] - rotation[0, 1]],
        dtype=np.float64,
    )
    if angle < 1.0e-8:
        return 0.5 * vee
    if math.pi - angle < 1.0e-5:
        values, vectors = np.linalg.eigh(rotation)
        axis = vectors[:, int(np.argmin(np.abs(values - 1.0)))]
        axis /= np.linalg.norm(axis)
        return axis * angle
    return vee * (angle / (2.0 * math.sin(angle)))


def body_twist(previous_pose: np.ndarray, current_pose: np.ndarray, dt: float) -> BodyTwist:
    """Finite-difference sensor-origin twist expressed in current sensor axes."""
    previous_pose = np.asarray(previous_pose, dtype=np.float64)
    current_pose = np.asarray(current_pose, dtype=np.float64)
    if previous_pose.shape != (4, 4) or current_pose.shape != (4, 4):
        raise ValueError("poses must be 4x4 matrices")
    if not np.isfinite(previous_pose).all() or not np.isfinite(current_pose).all():
        raise ValueError("poses must be finite")
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be positive and finite")

    current_rotation = current_pose[:3, :3]
    world_linear = (current_pose[:3, 3] - previous_pose[:3, 3]) / dt
    linear = current_rotation.T @ world_linear

    relative_rotation = previous_pose[:3, :3].T @ current_rotation
    angular_previous = _rotation_log(relative_rotation) / dt
    angular = current_rotation.T @ previous_pose[:3, :3] @ angular_previous
    return BodyTwist(linear=linear, angular=angular)


def quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized xyzw quaternion."""
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        j = (index + 1) % 3
        k = (index + 2) % 3
        scale = math.sqrt(max(0.0, 1.0 + rotation[index, index] - rotation[j, j] - rotation[k, k])) * 2.0
        quat = np.zeros(4)
        quat[index] = 0.25 * scale
        quat[j] = (rotation[j, index] + rotation[index, j]) / scale
        quat[k] = (rotation[k, index] + rotation[index, k]) / scale
        quat[3] = (rotation[k, j] - rotation[j, k]) / scale
    norm = np.linalg.norm(quat)
    if norm <= 0.0:
        raise ValueError("rotation produced a zero quaternion")
    return quat / norm
