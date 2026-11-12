#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
model_dir="$repo_dir/src/inference/models/yopo"
if [[ ! -x "$model_dir/.venv/bin/python" ]]; then
  echo "Run bash $model_dir/setup.sh first" >&2
  exit 1
fi
cd "$model_dir"
export PYTHONDONTWRITEBYTECODE=1
exec .venv/bin/python -m uvicorn serve:app --host "${INFERENCE_HOST:-127.0.0.1}" --port "${INFERENCE_PORT:-8000}" --workers 1 --limit-concurrency 16 --h11-max-incomplete-event-size 16777216
