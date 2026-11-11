"""Compare raw FP16 RoboPi outputs with the exported reference sample."""

import argparse
import json
from pathlib import Path

import numpy as np


def flatten(path, shape):
    return np.fromfile(path, dtype=np.float16).reshape(shape).astype(np.float32).reshape(-1)


def metrics(expected, actual):
    return {
        "max_abs": float(np.max(np.abs(expected - actual))),
        "mean_abs": float(np.mean(np.abs(expected - actual))),
        "cosine": float(
            np.dot(expected, actual)
            / (np.linalg.norm(expected) * np.linalg.norm(actual))
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=Path("artifacts/references/reference-000.npz"))
    parser.add_argument("--predictions", type=Path, default=Path("artifacts/npu-predictions.f16"))
    parser.add_argument("--scores", type=Path, default=Path("artifacts/npu-scores.f16"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/npu-compare.json"))
    args = parser.parse_args()

    reference = np.load(args.reference)
    expected = {
        "predictions": reference["predictions"].astype(np.float32).reshape(-1),
        "scores": reference["scores"].astype(np.float32).reshape(-1),
    }
    actual = {
        "predictions": flatten(args.predictions, reference["predictions"].shape),
        "scores": flatten(args.scores, reference["scores"].shape),
    }
    report = {
        "reference": str(args.reference),
        "predictions": str(args.predictions),
        "scores": str(args.scores),
        "metrics": {name: metrics(expected[name], actual[name]) for name in expected},
        "best_index_reference": int(expected["scores"].argmin()),
        "best_index_npu": int(actual["scores"].argmin()),
        "best_index_match": bool(
            expected["scores"].argmin() == actual["scores"].argmin()
        ),
        "scope": "one synthetic sample; not representative validation or flight acceptance",
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
