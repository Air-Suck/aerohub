#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${FAST_LIVO2_WORKSPACE:-$root/log/workspaces/fast_livo2}
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}
domain=${FAST_LIVO2_TEST_DOMAIN:-219}
run_id=${FAST_LIVO2_TEST_RUN_ID:-adapter-$(date +%Y%m%d-%H%M%S)}
evidence=$root/log/fast_livo2/$run_id

test -f "$workspace/install/setup.bash" || {
  printf 'Prepared workspace is missing: %s\n' "$workspace" >&2
  exit 2
}
mkdir -p "$evidence"
docker image inspect "$image" > "$evidence/shared-image.json"
sha256sum "$root/deployment/fast_livo2/tests/adapter_integration.py" \
  > "$evidence/test.sha256"

container=aerohub-fast-livo2-adapter-test-$$
cleanup_container() {
  docker rm --force "$container" >/dev/null 2>&1 || true
}
trap cleanup_container EXIT INT TERM

docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=fast-livo2-adapter-test \
  --env "ROS_DOMAIN_ID=$domain" \
  --volume "$workspace:/workspace:ro" \
  --volume "$root/deployment/fast_livo2/tests:/tests:ro" \
  --volume "$evidence:/evidence" \
  --entrypoint bash "$image" -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /workspace/install/setup.bash
pids=()
cleanup_nodes() {
  if ((${#pids[@]} == 0)); then return; fi
  kill -INT "${pids[@]}" 2>/dev/null || true
  for _ in $(seq 1 50); do
    live=0
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && live=1; done
    ((live == 0)) && break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      printf "cleanup escalation: SIGTERM pid=%s\n" "$pid" | tee -a /evidence/cleanup.log
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup_nodes EXIT INT TERM
/workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/pointcloud_to_livox \
  >/evidence/pointcloud_to_livox.log 2>&1 &
pids+=("$!")
/workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/sensor_qos_adapter \
  >/evidence/sensor_qos_adapter.log 2>&1 &
pids+=("$!")
/workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/output_adapter \
  >/evidence/output_adapter.log 2>&1 &
pids+=("$!")
python3 /tests/adapter_integration.py | tee /evidence/result.log
'
trap - EXIT INT TERM

if docker ps -a --filter "name=^/$container$" --format '{{.ID}}' | grep -q .; then
  printf 'Owned test container remains after completion: %s\n' "$container" >&2
  exit 1
fi
printf 'FAST-LIVO2 adapter integration evidence: %s\n' "$evidence"
