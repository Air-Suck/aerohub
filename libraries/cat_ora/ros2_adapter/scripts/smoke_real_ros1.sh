#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/workspace}
install_root=${2:-/tmp/uav_planner_install}
resolved_dir=${3:-/tmp/uav_cat_ora_real_resolved}
domain_id=${4:-78}

source /opt/ros/humble/setup.bash
source "$install_root/setup.bash"
export ROS_DOMAIN_ID="$domain_id"
set -u

ros2 run uav_planner_framework planner_framework_node \
  --ros-args \
  -p registry_root:="$workspace/planner_framework/registry" \
  -p resolved_dir:="$resolved_dir" >/tmp/uav_cat_real_framework.log 2>&1 &
framework_pid=$!
trap 'kill "$framework_pid" "${adapter_pid:-}" "${trajectory_echo_pid:-}" "${setpoint_echo_pid:-}" 2>/dev/null || true' EXIT

for _ in {1..20}; do
  ros2 service list | grep -qx /planner/control && break
  sleep 0.25
done
ros2 service list | grep -qx /planner/control

response=$(timeout 10 ros2 service call \
  /planner/control \
  uav_planner_interfaces/srv/ControlPlanner \
  "{command: 0, task_config_path: '$workspace/planner_framework/examples/cat_ora_four.yaml', deployment_config_path: '$workspace/planner_framework/examples/deployment_four.yaml'}")
grep -q 'accepted=True' <<<"$response"
task_id=$(grep -oE "task_id='[^']+'" <<<"$response" | head -n1 | cut -d"'" -f2)
resolved_config="$resolved_dir/$task_id.yaml"
test -f "$resolved_config"

ros2 run uav_cat_ora_adapter cat_ora_adapter_node \
  --ros-args \
  -p resolved_config:="$resolved_config" \
  -p valid_from_lead:=0.5 >/tmp/uav_cat_real_adapter.log 2>&1 &
adapter_pid=$!

for _ in {1..20}; do
  ros2 node list | grep -qx /cat_ora_adapter && break
  sleep 0.25
done
ros2 node list | grep -qx /cat_ora_adapter

timeout 15 ros2 topic echo /planner/trajectory >/tmp/uav_cat_real_trajectory.log &
trajectory_echo_pid=$!
timeout 15 ros2 topic echo /planner/setpoints >/tmp/uav_cat_real_setpoints.log &
setpoint_echo_pid=$!

pose_pids=()
for vehicle in drone0 drone1 drone2 drone3; do
  timeout 10 ros2 topic pub --once \
    "/$vehicle/self_localization/pose" \
    geometry_msgs/msg/PoseStamped \
    '{pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}' \
    >/dev/null &
  pose_pids+=("$!")
done
for pose_pid in "${pose_pids[@]}"; do
  wait "$pose_pid"
done

for _ in {1..40}; do
  grep -q "task_id: $task_id" /tmp/uav_cat_real_setpoints.log 2>/dev/null && break
  sleep 0.25
done
kill "$trajectory_echo_pid" "$setpoint_echo_pid" 2>/dev/null || true

grep -q "task_id: $task_id" /tmp/uav_cat_real_trajectory.log
grep -q "task_id: $task_id" /tmp/uav_cat_real_setpoints.log
for vehicle in drone0 drone1 drone2 drone3; do
  grep -q "vehicle_id: $vehicle" /tmp/uav_cat_real_trajectory.log
  grep -q "vehicle_id: $vehicle" /tmp/uav_cat_real_setpoints.log
done
printf '%s\n' "$response"
grep -E 'task_id:|vehicle_id:' /tmp/uav_cat_real_trajectory.log
