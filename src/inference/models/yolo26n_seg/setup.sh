#!/usr/bin/env bash
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
check_cuda=false
case "${1:-}" in
  '') ;;
  --check-cuda) check_cuda=true ;;
  -h|--help) echo "Usage: bash $0 [--check-cuda]"; exit 0 ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo 'Unexpected extra arguments' >&2; exit 2; fi
command -v uv >/dev/null || { echo 'Install uv first' >&2; exit 1; }
python3 "$model_dir/prepare_artifacts.py"
UV_PROJECT_ENVIRONMENT="$model_dir/.venv" uv sync --locked --project "$model_dir"
export PYTHONDONTWRITEBYTECODE=1
export YOLO_CONFIG_DIR="$model_dir/artifacts/ultralytics"
cd "$model_dir"
.venv/bin/python prepare.py
YOLO_DEVICE=cpu .venv/bin/python -c 'from adapter import Predictor; p=Predictor(); print(p.metadata); p.close()'
if "$check_cuda"; then
  YOLO_DEVICE=0 .venv/bin/python -c 'from adapter import Predictor; p=Predictor(); print(p.metadata); p.close()'
fi
