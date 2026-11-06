#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/workspace}
install_root=${2:-/tmp/uav_planner_install}
resolved_dir=${3:-/tmp/uav_cat_ora_resolved}
proxy_port=${4:-28501}
domain_id=${5:-77}

source /opt/ros/humble/setup.bash
source "$install_root/setup.bash"
export ROS_DOMAIN_ID="$domain_id"
set -u

python3 "$workspace/src/cat_ora_adapter/scripts/fake_proxy.py" "$proxy_port" &
proxy_pid=$!
ros2 run uav_planner_framework planner_framework_node \
  --ros-args \
  -p registry_root:="$workspace/src/planner_framework/registry" \
  -p resolved_dir:="$resolved_dir" >/tmp/uav_cat_ora_framework.log 2>&1 &
framework_pid=$!
trap 'kill "$proxy_pid" "$framework_pid" "${adapter_pid:-}" "${echo_pid:-}" 2>/dev/null || true' EXIT

for _ in {1..20}; do
  ros2 service list | grep -qx /planner/control && break
  sleep 0.25
done
ros2 service list | grep -qx /planner/control

response=$(timeout 10 ros2 service call \
  /planner/control \
  uav_planner_interfaces/srv/ControlPlanner \
  "{command: 0, task_config_path: '$workspace/src/planner_framework/examples/cat_ora_single.yaml', deployment_config_path: '$workspace/src/planner_framework/examples/deployment_single.yaml'}")
grep -q 'accepted=True' <<<"$response"
task_id=$(grep -oE "task_id='[^']+'" <<<"$response" | head -n1 | cut -d"'" -f2)
resolved_config="$resolved_dir/$task_id.yaml"
test -f "$resolved_config"

ros2 run uav_cat_ora_adapter cat_ora_adapter_node \
  --ros-args \
  -p resolved_config:="$resolved_config" \
  -p proxy_port:="$proxy_port" \
  -p valid_from_lead:=0.5 >/tmp/uav_cat_ora_adapter.log 2>&1 &
adapter_pid=$!

for _ in {1..20}; do
  ros2 node list | grep -qx /cat_ora_adapter && break
  sleep 0.25
done
ros2 node list | grep -qx /cat_ora_adapter

timeout 8 ros2 topic echo /planner/setpoints >/tmp/uav_cat_ora_setpoints.log &
echo_pid=$!
timeout 10 ros2 topic pub --once \
  /drone0/self_localization/pose \
  geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}' \
  >/tmp/uav_cat_ora_pose.log
sleep 2

kill "$echo_pid" 2>/dev/null || true
grep -q "task_id: $task_id" /tmp/uav_cat_ora_setpoints.log
grep -q 'vehicle_id: drone0' /tmp/uav_cat_ora_setpoints.log
grep -q 'x: 2.0' /tmp/uav_cat_ora_setpoints.log
grep -q 'z: -1.0' /tmp/uav_cat_ora_setpoints.log
printf '%s\n' "$response"
sed -n '1,20p' /tmp/uav_cat_ora_setpoints.log
