#!/usr/bin/env bash
set -eo pipefail

workspace=${FAST_LIVO2_WORKSPACE_CONTAINER:-/workspace}
config=${FAST_LIVO2_CONFIG:-/config/airsim.yaml}
camera=${FAST_LIVO2_CAMERA_CONFIG:-/config/camera.yaml}

source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source "$workspace/install/setup.bash"
set -u

pids=()
stopping=0
shutdown() {
  ((stopping == 0)) || return
  stopping=1
  if ((${#pids[@]} == 0)); then return; fi
  kill -INT "${pids[@]}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    live=0
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && live=1; done
    ((live == 0)) && break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do
    kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap shutdown EXIT INT TERM

"$workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/pointcloud_to_livox" &
pids+=("$!")
"$workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/sensor_qos_adapter" &
pids+=("$!")
"$workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/output_adapter" &
pids+=("$!")
"$workspace/install/fast_livo/lib/fast_livo/fastlivo_mapping" \
  --ros-args --params-file "$config" --params-file "$camera" &
pids+=("$!")

set +e
wait -n "${pids[@]}"
status=$?
set -e
if ((stopping == 0)); then
  printf 'FAST-LIVO2 owned process exited unexpectedly (status=%s)\n' "$status" >&2
  exit 1
fi
exit "$status"
