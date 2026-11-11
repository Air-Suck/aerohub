"""YOPO inference only. Algorithm ownership remains src/planner/yopo/upstream."""

import hashlib
import os
import math
import time
import sys
from pathlib import Path

import cv2
import numpy as np
import torch


PREPROCESSING_VERSION = "ros-reference-u8-inpaint-v1"


def preprocess_depth(depth, width, height):
    """Match test_yopo_ros.callback_depth: resize, cap, quantize, inpaint."""
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


class Predictor:
    def __init__(self):
        repo = Path(__file__).resolve().parents[4]
        source = repo / "src/planner/yopo/upstream/YOPO"
        if not source.is_dir():
            raise RuntimeError("Initialize src/planner/yopo/upstream submodule first")
        sys.path.insert(0, str(source))
        from config.config import cfg
        from policy.yopo_network import YopoNetwork

        device = os.environ.get("YOPO_DEVICE", "cuda")
        if device not in ("cuda", "cpu"):
            raise ValueError("YOPO_DEVICE must be cuda or cpu")
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA is required; CPU smoke tests must set YOPO_DEVICE=cpu")
        if device == "cpu" and torch.cuda.is_available():
            raise RuntimeError("For CPU mode set CUDA_VISIBLE_DEVICES='' before startup")
        torch.set_num_threads(int(os.environ.get("YOPO_THREADS", "4")))
        self.device = torch.device(device)
        self.cfg = cfg
        cfg["train"] = False
        # Upstream LatticeParam explicitly scales velocity, acceleration and time by this ratio.
        velocity = float(os.environ.get("YOPO_VELOCITY", str(cfg["velocity"])))
        if not math.isfinite(velocity) or not 0 < velocity <= cfg["vel_max_train"]:
            raise ValueError("YOPO_VELOCITY must be positive and <= trained 6 m/s")
        cfg["velocity"] = velocity
        weights = Path(os.environ.get("YOPO_WEIGHTS", str(Path(__file__).resolve().parent / "artifacts/epoch50.pth")))
        if not weights.is_file():
            raise RuntimeError("Weights are missing; run src/inference/models/yopo/setup.sh or set YOPO_WEIGHTS")
        digest = hashlib.sha256(weights.read_bytes()).hexdigest()
        self.metadata = {"model_id": "yopo", "model_version": "sha256:" + digest + ";" + PREPROCESSING_VERSION + ";velocity=" + str(velocity), "device": device, "preprocessing_version": PREPROCESSING_VERSION, "velocity_m_s": velocity, "source_revision": "bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132"}
        self.model = YopoNetwork().to(self.device).eval()
        self.model.load_state_dict(torch.load(weights, map_location=self.device, weights_only=True))
        self.infer({"request_id": "warmup", "depth": [[20.0]], "state": [0, 0, 0, 0, 0, 0, 10, 0, 0]})

    @torch.inference_mode()
    def infer(self, request):
        started = time.perf_counter()
        depth = np.asarray(request["depth"], dtype=np.float32)
        state = np.asarray(request["state"], dtype=np.float32)
        if state.shape != (9,) or not np.isfinite(state).all():
            raise ValueError("state must contain 9 finite values")
        depth = preprocess_depth(depth, self.cfg["image_width"], self.cfg["image_height"])
        transform = self.model.state_transform
        normalized = transform.normalize_obs_cpu(state[None])
        obs = transform.prepare_input_cpu(normalized)
        prepared = time.perf_counter()
        predictions, scores = self.model(torch.from_numpy(depth).to(self.device),
                                         torch.from_numpy(obs).to(self.device))
        predictions = predictions.cpu().numpy().reshape(9, -1).T
        scores = scores.cpu().numpy().reshape(-1)
        forwarded = time.perf_counter()
        count = len(predictions)
        endstates = transform.pred_to_endstate_cpu(predictions, np.arange(count - 1, -1, -1))
        if not np.isfinite(endstates).all() or not np.isfinite(scores).all():
            raise RuntimeError("model produced nonfinite predictions")
        best = int(scores.argmin())
        return dict(
            request_id=request.get("request_id", ""),
            model_version=self.metadata["model_version"], device=str(self.device),
            best_index=best, best_endstate=endstates[best].tolist(),
            endstates=endstates.tolist(), scores=scores.tolist(),
            duration_s=self.model.state_transform.lattice_primitive.segment_time,
            timings_ms=dict(preprocess=(prepared-started)*1000,
                            forward_and_transfer=(forwarded-prepared)*1000,
                            decode=(time.perf_counter()-forwarded)*1000),
        )
