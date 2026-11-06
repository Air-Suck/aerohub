#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/workspace}
install_root=${2:-/tmp/uav_planner_install}
resolved_dir=${3:-/tmp/uav_planner_resolved}

source /opt/ros/humble/setup.bash
source "$install_root/setup.bash"
set -u
rm -rf "$resolved_dir"

ros2 run uav_planner_framework planner_framework_node \
  --ros-args \
  -p registry_root:="$workspace/src/planner_framework/registry" \
  -p resolved_dir:="$resolved_dir" >/tmp/uav_planner_node.log 2>&1 &
node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true' EXIT

for _ in {1..20}; do
  if ros2 service list | grep -qx /planner/control; then
    break
  fi
  sleep 0.25
done

response=$(ros2 service call \
  /planner/control \
  uav_planner_interfaces/srv/ControlPlanner \
  "{command: 0, task_config_path: '$workspace/src/planner_framework/examples/cat_ora_single.yaml', deployment_config_path: '$workspace/src/planner_framework/examples/deployment_single.yaml'}")

grep -q 'accepted=True' <<<"$response"
test -n "$(find "$resolved_dir" -name '*.yaml' -print -quit)"
task_id=$(grep -oE "task_id='[^']+'" <<<"$response" | head -n1 | cut -d"'" -f2)
test -n "$task_id"

timeout 8 ros2 topic echo /planner/setpoints >/tmp/uav_planner_setpoints.log &
echo_pid=$!
now=$(date +%s)
valid_from=$((now + 1))
ros2 topic pub --once \
  /planner/trajectory \
  uav_planner_interfaces/msg/TimedTrajectorySegment \
  "{interface_version: 1, task_id: '$task_id', generated_at: {sec: $now}, valid_from: {sec: $valid_from}, is_final: true, vehicle_trajectories: [{vehicle_id: drone0, valid_fields: 0, points: [{time_from_start: {sec: 0}, position: {x: 0.0, y: 0.0, z: -1.0}}, {time_from_start: {sec: 2}, position: {x: 2.0, y: 0.0, z: -1.0}}]}]}" \
  >/tmp/uav_planner_publish.log
sleep 2
kill "$echo_pid" 2>/dev/null || true
grep -q "task_id: $task_id" /tmp/uav_planner_setpoints.log
grep -q 'vehicle_id: drone0' /tmp/uav_planner_setpoints.log
printf '%s\n' "$response"
sed -n '1,20p' /tmp/uav_planner_setpoints.log
