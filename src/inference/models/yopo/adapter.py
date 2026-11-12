"""Construct YOPO's Torch backend; CPU model semantics live in the shared pipeline."""
import math
import os
import sys
from pathlib import Path

import torch

from aerohub_inference.backends.torch import TorchBackend
from predictor import YopoPredictor
from processing import YopoProcessing, preprocess_depth


class Predictor(YopoPredictor):
    def __init__(self):
        repo = Path(__file__).resolve().parents[4]
        source = repo / "src/planner/yopo/upstream/YOPO"
        if not (source / "policy/yopo_network.py").is_file():
            raise RuntimeError("Initialize src/planner/yopo/upstream submodule first")
        sys.path.insert(0, str(source))
        from config.config import cfg
        from policy.yopo_network import YopoNetwork

        device = os.environ.get("YOPO_DEVICE", "cuda")
        if device not in ("cuda", "cpu"):
            raise ValueError("YOPO_DEVICE must be cuda or cpu")
        if device == "cpu" and torch.cuda.is_available():
            raise RuntimeError("For CPU mode set CUDA_VISIBLE_DEVICES='' before startup")
        torch.set_num_threads(int(os.environ.get("YOPO_THREADS", "4")))
        cfg["train"] = False
        velocity = float(os.environ.get("YOPO_VELOCITY", str(cfg["velocity"])))
        if not math.isfinite(velocity) or not 0 < velocity <= cfg["vel_max_train"]:
            raise ValueError("YOPO_VELOCITY must be positive and <= trained 6 m/s")
        cfg["velocity"] = velocity
        weights = Path(os.environ.get("YOPO_WEIGHTS", str(Path(__file__).resolve().parent / "artifacts/epoch50.pth")))
        if not weights.is_file():
            raise RuntimeError("Weights are missing; run setup.sh or set YOPO_WEIGHTS")
        # Extract constants from the locked upstream model, without copying its network.
        model = YopoNetwork()
        transform = model.state_transform
        lattice = transform.lattice_primitive
        data = dict(lattice_yaw=transform.lattice_yaw_np,
                    lattice_pitch=transform.lattice_pitch_np,
                    lattice_Rbp=transform.lattice_Rbp_np,
                    goal_length=transform.goal_length, vel_max=lattice.vel_max,
                    acc_max=lattice.acc_max, segment_time=lattice.segment_time,
                    yaw_diff=lattice.yaw_diff, pitch_diff=lattice.pitch_diff,
                    radio_range=lattice.radio_range, vertical_num=lattice.vertical_num,
                    horizon_num=lattice.horizon_num, image_height=cfg["image_height"],
                    image_width=cfg["image_width"])
        processing = YopoProcessing(data, velocity)
        backend = TorchBackend(lambda: model, weights, "yopo", processing.inputs,
                               processing.outputs, device=device)
        super().__init__(backend, processing)
