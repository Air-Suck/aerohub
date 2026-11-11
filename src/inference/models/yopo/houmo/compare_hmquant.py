"""Compare the quantized HMONNX on CPU with the exported PyTorch references."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from xhquant.api import HMONNXGoldenInference


def flatten(value):
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float32).reshape(-1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--hmonnx",
        type=Path,
        default=Path("artifacts/smoke-output/xh2/hmquant/hmquant_yopo_with_act.onnx"),
    )
    parser.add_argument("--data-dir", type=Path, default=Path("artifacts/synthetic-inputs"))
    parser.add_argument("--reference-dir", type=Path, default=Path("artifacts/references"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/quant-compare.json"))
    args = parser.parse_args()

    session = HMONNXGoldenInference(str(args.hmonnx))
    session.to("cpu")
    results = []
    for data_path in sorted(args.data_dir.glob("*.npz")):
        reference_path = args.reference_dir / data_path.name.replace("synthetic-", "reference-")
        data = np.load(data_path)
        reference = np.load(reference_path)
        outputs = session(
            torch.from_numpy(data["depth"]).half(),
            torch.from_numpy(data["obs"]).half(),
        )
        if not isinstance(outputs, (tuple, list)):
            outputs = (outputs,)
        names = ("predictions", "scores")
        metrics = {}
        for name, output in zip(names, outputs):
            expected = flatten(reference[name])
            actual = flatten(output)
            metrics[name] = {
                "max_abs": float(np.max(np.abs(expected - actual))),
                "mean_abs": float(np.mean(np.abs(expected - actual))),
                "cosine": float(
                    np.dot(expected, actual)
                    / (np.linalg.norm(expected) * np.linalg.norm(actual))
                ),
            }
        expected_scores = reference["scores"].reshape(-1)
        actual_scores = flatten(outputs[1])
        results.append(
            {
                "sample": data_path.name,
                "metrics": metrics,
                "best_index_reference": int(expected_scores.argmin()),
                "best_index_quantized": int(actual_scores.argmin()),
                "best_index_match": bool(expected_scores.argmin() == actual_scores.argmin()),
            }
        )

    report = {
        "hmonnx": str(args.hmonnx),
        "sample_count": len(results),
        "best_index_match_count": sum(item["best_index_match"] for item in results),
        "samples": results,
        "scope": "synthetic export-only inputs; not representative quantization or flight acceptance",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
