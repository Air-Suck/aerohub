"""Assemble the YOPO Houmo Python service for upload to RoboPi."""

import argparse
import shutil
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[6]
HERE = Path(__file__).resolve().parent
MODEL = HERE.parent


def copy_tree(source, target):
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=HERE / "artifacts/board-runtime")
    parser.add_argument("--hmm", type=Path, default=HERE / "artifacts/smoke-output/xh2/yopo_xh2_b1_1core_O2.hmm")
    parser.add_argument("--metadata", type=Path, default=HERE / "artifacts/yopo-runtime.npz")
    parser.add_argument("--tcim-lite", type=Path, default=HERE / "artifacts/board-tcim/tcim_lite")
    parser.add_argument("--tcim-lib", type=Path, required=True)
    parser.add_argument("--hal-lib", type=Path, required=True)
    args = parser.parse_args()

    required = [args.hmm, args.metadata, args.tcim_lib, args.hal_lib]
    required.extend([args.tcim_lite / "pytcim.cpython-312-aarch64-linux-ohos.so"])
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("\n".join(missing))

    output = args.output.resolve()
    # Fail closed on an existing destination; never recursively delete user paths.
    if output.exists():
        raise FileExistsError("Choose a new --output directory: {}".format(output))
    output.mkdir(parents=True)
    shutil.copy2(HERE / "adapter.py", output / "adapter.py")
    shutil.copy2(HERE / "serve.py", output / "serve.py")
    shutil.copy2(HERE / "start_board.sh", output / "start_board.sh")
    for name in ("schemas.py", "processing.py", "predictor.py"):
        shutil.copy2(MODEL / name, output / name)
    shutil.copy2(args.hmm, output / "model.hmm")
    shutil.copy2(args.metadata, output / "yopo-runtime.npz")
    copy_tree(ROOT / "src/basic_function/inference/runtime/aerohub_inference", output / "aerohub_inference")
    copy_tree(ROOT / "src/basic_function/inference/runtime_houmo/aerohub_houmo", output / "aerohub_houmo")
    copy_tree(args.tcim_lite, output / "tcim_lite")
    libraries = output / "lib"
    libraries.mkdir()
    shutil.copy2(args.tcim_lib, libraries / args.tcim_lib.name)
    shutil.copy2(args.hal_lib, libraries / args.hal_lib.name)

    wheelhouse = HERE / "artifacts/wheels/board"
    wheels = sorted(wheelhouse.glob("*.whl"))
    expected = {"fastapi", "starlette", "uvicorn"}
    if {path.name.split("-", 1)[0] for path in wheels} != expected:
        raise RuntimeError("board wheelhouse must contain FastAPI, Starlette and Uvicorn")
    for wheel in wheels:
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(output)

    print(output)


if __name__ == "__main__":
    main()
