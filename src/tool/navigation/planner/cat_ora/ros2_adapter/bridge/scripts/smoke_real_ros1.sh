#!/usr/bin/env bash
# Historical filename; the JSON endpoint may be backed by ROS 1 or ROS 2.
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
  -p registry_root:="$workspace/src/tool/navigation/framework/registry" \
  -p resolved_dir:="$resolved_dir" >/tmp/uav_cat_real_framework.log 2>&1 &
framework_pid=$!
trap 'kill "$framework_pid" "${trajectory_echo_pid:-}" "${setpoint_echo_pid:-}" "${pose_pid:-}" 2>/dev/null || true' EXIT

for _ in {1..20}; do
  ros2 service list | grep -qx /planner/control && break
  sleep 0.25
done
ros2 service list | grep -qx /planner/control

response=$(timeout 10 ros2 service call \
  /planner/control \
  uav_planner_interfaces/srv/ControlPlanner \
  "{command: 0, task_config_path: '$workspace/src/tool/navigation/framework/examples/cat_ora_four.yaml', deployment_config_path: '$workspace/src/tool/navigation/framework/examples/deployment_four.yaml'}")
grep -q 'accepted=True' <<<"$response"
task_id=$(grep -oE "task_id='[^']+'" <<<"$response" | head -n1 | cut -d"'" -f2)
resolved_config="$resolved_dir/$task_id.yaml"
test -f "$resolved_config"

# START already launches the registered adapter through PlannerFramework.
for _ in {1..20}; do
  ros2 node list | grep -qx /cat_ora_adapter && break
  sleep 0.25
done
ros2 node list | grep -qx /cat_ora_adapter

timeout 15 ros2 topic echo /planner/trajectory >/tmp/uav_cat_real_trajectory.log &
trajectory_echo_pid=$!
timeout 15 ros2 topic echo /planner/setpoints >/tmp/uav_cat_real_setpoints.log &
setpoint_echo_pid=$!

python3 "$workspace/src/tool/navigation/planner/cat_ora/ros2_adapter/bridge/scripts/publish_test_poses.py" &
pose_pid=$!

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
