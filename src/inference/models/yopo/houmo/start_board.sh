#!/system/bin/sh
set -eu
runtime_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
export PYTHONPATH="$runtime_dir${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$runtime_dir/lib:/sys_prod/robot/out/lib:/system/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TCIM_RUNTIME_PATH="$runtime_dir"
export YOPO_HMM_PATH="$runtime_dir/model.hmm"
export YOPO_RUNTIME_METADATA="$runtime_dir/yopo-runtime.npz"
cd "$runtime_dir"
exec /sys_prod/robot/out/bin/python3 -m uvicorn serve:app \
  --host "${INFERENCE_HOST:-127.0.0.1}" \
  --port "${INFERENCE_PORT:-8000}" \
  --workers 1 --limit-concurrency 16 --h11-max-incomplete-event-size 16777216
