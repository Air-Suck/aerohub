#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/workspace}
install_root=${2:-/tmp/uav_planner_install}
resolved_dir=${3:-/tmp/uav_as2_adapter_resolved}
domain_id=${4:-79}

source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source "$install_root/setup.bash"
export ROS_DOMAIN_ID="$domain_id"
set -u

ros2 run uav_planner_framework planner_framework_node \
  --ros-args \
  -p registry_root:="$workspace/planner_framework/registry" \
  -p resolved_dir:="$resolved_dir" >/tmp/uav_as2_adapter_framework.log 2>&1 &
framework_pid=$!
python3 "$workspace/planner_framework/bridges/as2_setpoint/scripts/mock_as2.py" \
  drone0 drone1 drone2 drone3 >/tmp/uav_as2_adapter_mock.log 2>&1 &
mock_pid=$!
echo_pids=()
trap 'kill "$framework_pid" "$mock_pid" "${adapter_pid:-}" "${echo_pids[@]}" 2>/dev/null || true' EXIT

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

ros2 run uav_as2_setpoint_adapter as2_setpoint_adapter_node \
  --ros-args -p resolved_config:="$resolved_config" \
  >/tmp/uav_as2_adapter.log 2>&1 &
adapter_pid=$!

for _ in {1..20}; do
  ros2 node list | grep -qx /as2_setpoint_adapter && break
  sleep 0.25
done
ros2 node list | grep -qx /as2_setpoint_adapter

for vehicle in drone0 drone1 drone2 drone3; do
  timeout 10 ros2 service call "/$vehicle/controller/set_control_mode" as2_msgs/srv/SetControlMode "{control_mode: {control_mode: 7, yaw_mode: 1}}" >/dev/null
  timeout 10 ros2 topic echo --once "/$vehicle/motion_reference/trajectory" \
    >"/tmp/uav_as2_adapter_${vehicle}.log" &
  echo_pids+=("$!")
done

sleep 1
now=$(date +%s)
timeout 10 ros2 topic pub --once \
  /planner/setpoints \
  uav_planner_interfaces/msg/FleetSetpoint \
  "{task_id: '$task_id', sampled_at: {sec: $now}, setpoints: [
    {vehicle_id: drone0, position: {x: 1.0, y: 2.0, z: -3.0}, yaw: .nan},
    {vehicle_id: drone1, position: {x: 1.0, y: 2.0, z: -3.0}, yaw: .nan},
    {vehicle_id: drone2, position: {x: 1.0, y: 2.0, z: -3.0}, yaw: .nan},
    {vehicle_id: drone3, position: {x: 1.0, y: 2.0, z: -3.0}, yaw: .nan}]}" \
  >/dev/null

for echo_pid in "${echo_pids[@]}"; do
  wait "$echo_pid"
done
for vehicle in drone0 drone1 drone2 drone3; do
  output="/tmp/uav_as2_adapter_${vehicle}.log"
  grep -q "frame_id: $vehicle/odom" "$output"
  grep -q 'x: 2.0' "$output"
  grep -q 'y: 1.0' "$output"
  grep -q 'z: 3.0' "$output"
done
printf '%s\n' "$response"
sed -n '1,16p' /tmp/uav_as2_adapter_drone0.log
