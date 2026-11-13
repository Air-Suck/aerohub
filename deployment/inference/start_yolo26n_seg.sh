#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
model_dir="$repo_dir/src/inference/models/yolo26n_seg"
if [[ ! -x "$model_dir/.venv/bin/python" ]]; then
  echo "Run bash $model_dir/setup.sh --check-cuda first" >&2
  exit 1
fi
cd "$model_dir"
export PYTHONDONTWRITEBYTECODE=1
export YOLO_CONFIG_DIR="$model_dir/artifacts/ultralytics"
exec .venv/bin/python -m uvicorn serve:app --host "${INFERENCE_HOST:-127.0.0.1}" --port "${INFERENCE_PORT:-8002}" --workers 1 --limit-concurrency 16
