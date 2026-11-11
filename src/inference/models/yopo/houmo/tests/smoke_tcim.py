"""Load an HMM through the board tcim_lite binding and save the raw outputs."""

import argparse
import json
import time
from pathlib import Path

import numpy as np

import tcim_lite as tcim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("hmm", type=Path)
    parser.add_argument("depth", type=Path)
    parser.add_argument("obs", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    started = time.perf_counter()
    module = tcim.runtime.Module.load(str(args.hmm))
    inputs = {
        "depth": np.load(args.depth),
        "obs": np.load(args.obs),
    }
    for name, value in inputs.items():
        module.set_input(name, value)
    prepared = time.perf_counter()
    module.run()
    module.sync()
    finished = time.perf_counter()

    outputs = {}
    for name in ("predictions", "scores"):
        outputs[name] = module.get_output(name).astype(np.float32).numpy()
    np.savez(args.output, **outputs)
    print(
        json.dumps(
            {
                "module_version": module.get_model_version(),
                "inputs": {name: list(value.shape) for name, value in inputs.items()},
                "outputs": {name: list(value.shape) for name, value in outputs.items()},
                "setup_ms": (prepared - started) * 1000,
                "run_ms": (finished - prepared) * 1000,
            }
        )
    )


if __name__ == "__main__":
    main()
