#!/usr/bin/env python3
"""Run one official TUM RGB-D sequence through DPV-SLAM on a real GPU."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import torch

from dpvo.config import cfg
from dpvo.dpvo import DPVO


def read_rows(path: Path, value_count: int):
    rows = []
    with path.open(encoding="ascii") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < value_count + 1:
                continue
            rows.append((float(fields[0]), fields[1 : value_count + 1]))
    return rows


def associate(reference, estimate_times, max_delta):
    reference_times = np.asarray([row[0] for row in reference])
    reference_xyz = np.asarray([[float(value) for value in row[1][:3]] for row in reference])
    pairs = []
    for estimate_index, stamp in enumerate(estimate_times):
        insertion = int(np.searchsorted(reference_times, stamp))
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(reference_times)]
        reference_index = min(candidates, key=lambda index: abs(reference_times[index] - stamp))
        delta = abs(reference_times[reference_index] - stamp)
        if delta <= max_delta:
            pairs.append((estimate_index, reference_index, delta))
    return pairs, reference_xyz


def align_sim3(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = target_centered.T @ source_centered / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    sign = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sign[-1] = -1
    rotation = u @ np.diag(sign) @ vt
    variance = np.sum(source_centered * source_centered) / len(source)
    scale = float(np.sum(singular_values * sign) / variance)
    translation = target_mean - scale * (rotation @ source_mean)
    return scale, rotation, translation


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@torch.no_grad()
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", type=Path, required=True)
    parser.add_argument("--network", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-association-delta", type=float, default=0.02)
    parser.add_argument("--max-rmse", type=float, default=0.05)
    args = parser.parse_args()

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for official DPV-SLAM validation")

    cfg.merge_from_file(str(args.config))
    cfg.LOOP_CLOSURE = True
    cfg.CLASSIC_LOOP_CLOSURE = False
    torch.manual_seed(1234)

    rgb_rows = read_rows(args.sequence / "rgb.txt", 1)[:: args.stride]
    groundtruth = read_rows(args.sequence / "groundtruth.txt", 7)
    if not rgb_rows or not groundtruth:
        raise RuntimeError("TUM rgb.txt or groundtruth.txt is empty")

    fx, fy, cx, cy = 517.3, 516.5, 318.6, 255.3
    camera_matrix = np.array([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]).reshape(3, 3)
    distortion = np.array([0.2624, -0.9531, -0.0054, 0.0026, 1.1633])
    intrinsics_values = np.array([fx, fy, cx - 16, cy - 8], dtype=np.float32)

    slam = None
    processed = 0
    for stamp, values in rgb_rows:
        image = cv2.imread(str(args.sequence / values[0]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read {values[0]}")
        image = cv2.undistort(image, camera_matrix, distortion)
        image = image[8:-8, 16:-16].transpose(2, 0, 1)
        image_tensor = torch.as_tensor(image, device="cuda")
        intrinsics = torch.as_tensor(intrinsics_values, device="cuda")
        if slam is None:
            slam = DPVO(cfg, str(args.network), ht=image.shape[-2], wd=image.shape[-1], viz=False)
        slam(stamp, image_tensor, intrinsics)
        processed += 1

    poses, estimate_times = slam.terminate()
    pairs, reference_xyz = associate(groundtruth, estimate_times, args.max_association_delta)
    if len(pairs) < 20:
        raise RuntimeError(f"only {len(pairs)} timestamp associations")

    estimate_indices = np.asarray([pair[0] for pair in pairs])
    reference_indices = np.asarray([pair[1] for pair in pairs])
    estimate_xyz = np.asarray(poses)[estimate_indices, :3]
    target_xyz = reference_xyz[reference_indices]
    scale, rotation, translation = align_sim3(estimate_xyz, target_xyz)
    aligned_xyz = (scale * (rotation @ estimate_xyz.T)).T + translation
    errors = np.linalg.norm(aligned_xyz - target_xyz, axis=1)

    args.output.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output / "estimated-aligned.txt", np.column_stack((np.asarray(estimate_times)[estimate_indices], aligned_xyz)), fmt="%.9f")
    np.savetxt(args.output / "groundtruth-associated.txt", np.column_stack((np.asarray(estimate_times)[estimate_indices], target_xyz)), fmt="%.9f")
    rmse = float(np.sqrt(np.mean(errors**2)))
    result = {
        "passed": bool(np.isfinite(errors).all() and rmse <= args.max_rmse),
        "sequence": args.sequence.name,
        "input_frames": len(rgb_rows),
        "processed_frames": processed,
        "estimated_poses": len(estimate_times),
        "associated_poses": len(pairs),
        "max_association_delta_s": args.max_association_delta,
        "observed_association_delta_max_s": max(pair[2] for pair in pairs),
        "loop_closure": bool(cfg.LOOP_CLOSURE),
        "classic_loop_closure": bool(cfg.CLASSIC_LOOP_CLOSURE),
        "acceptance_rmse_max_m": args.max_rmse,
        "network_sha256": sha256(args.network),
        "torch_version": torch.__version__,
        "cuda_version": torch.version.cuda,
        "gpu": torch.cuda.get_device_name(0),
        "sim3_scale": scale,
        "translation_ate_rmse_m": rmse,
        "translation_error_median_m": float(np.median(errors)),
        "translation_error_p95_m": float(np.percentile(errors, 95)),
        "translation_error_max_m": float(np.max(errors)),
    }
    (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
