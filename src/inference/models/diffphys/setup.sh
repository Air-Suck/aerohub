#!/usr/bin/env bash
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$model_dir/../../../.." && pwd)"
archive="${DIFFPHYS_ARCHIVE:-$repo_dir/log/diffphys/assets/validation_code.zip}"
if [[ ! -f "$archive" && -f "$archive.part" ]]; then archive="$archive.part"; fi
check_cuda=false
while (( $# )); do
  case "$1" in
    --archive) archive="${2:?--archive requires a path}"; shift 2 ;;
    --check-cuda) check_cuda=true; shift ;;
    -h|--help) echo "Usage: $0 [--archive official-validation_code.zip] [--check-cuda]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
command -v uv >/dev/null || { echo 'Install uv first' >&2; exit 1; }
python3 "$model_dir/prepare_artifacts.py" --archive "$archive"
UV_PROJECT_ENVIRONMENT="$model_dir/.venv" uv sync --locked --project "$model_dir"
cd "$model_dir"
export PYTHONDONTWRITEBYTECODE=1
CUDA_VISIBLE_DEVICES='' DIFFPHYS_DEVICE=cpu .venv/bin/python -c 'from adapter import Predictor; print(Predictor().metadata)'
if "$check_cuda"; then
  DIFFPHYS_DEVICE=cuda .venv/bin/python -c 'from adapter import Predictor; print(Predictor().metadata)'
fi
