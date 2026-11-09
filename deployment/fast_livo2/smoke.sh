#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root_dir"
mkdir -p log/fast_livo2
export FAST_LIVO_CONFIG=/tests/synthetic.yaml
export FAST_LIVO_CAMERA=/tests/synthetic_camera.yaml
export FAST_LIVO_SIM_TIME=false
export FAST_LIVO_DOMAIN=182
export FAST_LIVO_NAMESPACE=drone0
unset FAST_LIVO_ESTIMATOR_CONFIG
compose=(docker compose -p aerohub-fast-livo2-smoke -f deployment/fast_livo2/compose.yaml)
if [[ -n "$("${compose[@]}" ps -q)" ]]; then
  echo 'Smoke project is already running; stop that test before starting another.' >&2
  exit 1
fi
# Host-network services must not collide with an existing user deployment.
python3 - <<'PY'
import socket
for port in (11331,18761,18762,18763):
    with socket.socket() as s:
        s.bind(('127.0.0.1',port))
PY
cleanup() {
  "${compose[@]}" logs --no-color > log/fast_livo2/pipeline.log 2>&1 || true
  "${compose[@]}" down --timeout 5
}
trap cleanup EXIT
"${compose[@]}" up -d --no-build
"${compose[@]}" exec -T estimator bash -lc \
  'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && source /opt/fast_livo2_as2_ws/install/setup.bash && python3 /tests/observe_pipeline.py' \
  > log/fast_livo2/pipeline-observer.log 2>&1 &
observer_pid=$!
"${compose[@]}" exec -T algorithm bash -lc \
  'source /opt/fast_livo_ws/devel/setup.bash && python3 /tests/feed_synthetic.py' \
  > log/fast_livo2/synthetic-input.log 2>&1
wait "$observer_pid"
cat log/fast_livo2/pipeline-observer.log
