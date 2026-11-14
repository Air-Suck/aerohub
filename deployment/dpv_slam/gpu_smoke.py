#!/usr/bin/env python3
"""Exercise the locked DPV-SLAM model and all native CUDA extensions."""

import argparse
import json

import numpy as np
import torch

import cuda_ba
import cuda_corr
import lietorch_backends
from uav_dpv_slam_ros2.engine import DpvSlamEngine


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--network", required=True)
    args = parser.parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available")
    print(json.dumps({
        "torch": torch.__version__,
        "cuda": torch.version.cuda,
        "gpu": torch.cuda.get_device_name(0),
        "extensions": [cuda_corr.__name__, cuda_ba.__name__, lietorch_backends.__name__],
    }))
    engine = DpvSlamEngine(args.config, args.network)
    generator = np.random.default_rng(7)
    base = generator.integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
    results = []
    for index in range(12):
        image = np.roll(base, index * 7, axis=1)
        result = engine.process(
            index / 15.0,
            image,
            np.array([160.0, 160.0, 159.5, 119.5], dtype=np.float32),
        )
        results.append({
            "frame": index,
            "initialized": result.initialized,
            "frame_count": result.frame_count,
            "pose": result.pose is not None,
            "points": 0 if result.points is None else int(len(result.points)),
        })
    if results[-1]["frame_count"] != len(results):
        raise RuntimeError("DPV-SLAM did not process every GPU smoke frame")
    allocated = torch.cuda.memory_allocated()
    if allocated <= 0:
        raise RuntimeError("DPV-SLAM did not allocate CUDA memory")
    print(json.dumps(results))
    print(json.dumps({
        "allocated_bytes": allocated,
        "initialized": any(item["initialized"] for item in results),
        "scope": "synthetic GPU execution only; initialization requires dataset or platform evidence",
    }))


if __name__ == "__main__":
    main()
