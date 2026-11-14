#!/usr/bin/env bash
# Prepare only this model's environment and weights; never start ROS or a server.
set -euo pipefail
model_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$model_dir/../../../../.." && pwd)"
check_cuda=false
case "${1:-}" in
  "") ;;
  --check-cuda) check_cuda=true ;;
  -h|--help)
    echo "Usage: bash $0 [--check-cuda]"
    echo "Prepare locked dependencies and verified weights; validate CPU load/warmup."
    echo "--check-cuda additionally requires successful CUDA load/warmup."
    exit 0 ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo "Too many arguments" >&2; exit 2; fi
command -v uv >/dev/null || { echo "Install uv before preparing YOPO" >&2; exit 1; }
upstream="$repo_dir/src/tool/navigation/planner/yopo/upstream"
expected_revision=bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132
if [[ ! -f "$upstream/YOPO/policy/yopo_network.py" ]]; then
  echo "Initialize algorithm source first: git submodule update --init src/tool/navigation/planner/yopo/upstream" >&2
  exit 1
fi
if [[ "$(git -C "$upstream" rev-parse HEAD)" != "$expected_revision" ]]; then
  echo "YOPO source differs from the supported revision $expected_revision; preserving checkout" >&2
  exit 1
fi
source_weights="$upstream/YOPO/saved/YOPO_1/epoch50.pth"
read -r expected_hash weight_name < "$model_dir/weights.sha256"
printf '%s  %s\n' "$expected_hash" "$source_weights" | sha256sum --check -
mkdir -p "$model_dir/artifacts"
target="$model_dir/artifacts/$weight_name"
if [[ ! -e "$target" ]]; then
  temporary_weights="$(mktemp "$model_dir/artifacts/.weights.XXXXXX")"
  trap 'rm -f -- "$temporary_weights"' EXIT
  cp "$source_weights" "$temporary_weights"
  printf '%s  %s\n' "$expected_hash" "$temporary_weights" | sha256sum --check -
  mv "$temporary_weights" "$target"
  trap - EXIT
fi
# Refuse an unexpected existing artifact rather than silently replacing it.
printf '%s  %s\n' "$expected_hash" "$target" | sha256sum --check -
UV_PROJECT_ENVIRONMENT="$model_dir/.venv" uv sync --locked --project "$model_dir"
cd "$model_dir"
export PYTHONDONTWRITEBYTECODE=1
echo "Validating prepared weights and CPU load/warmup (not GPU acceptance)"
CUDA_VISIBLE_DEVICES='' YOPO_DEVICE=cpu YOPO_WEIGHTS="$target" .venv/bin/python - <<'PY'
import sys
import torch
from adapter import Predictor
predictor = Predictor()
print('Python:', sys.version.split()[0], 'PyTorch:', torch.__version__)
print('CPU preparation passed:', predictor.metadata)
PY
if "$check_cuda"; then
  echo "Validating CUDA load/warmup"
  YOPO_DEVICE=cuda YOPO_WEIGHTS="$target" .venv/bin/python - <<'PY'
from adapter import Predictor
print('CUDA load/warmup passed:', Predictor().metadata)
PY
fi
echo "YOPO prepared. Start with: bash $repo_dir/deployment/inference/start_yopo.sh"
