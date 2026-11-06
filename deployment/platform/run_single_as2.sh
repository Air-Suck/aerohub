#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
motion_package=as2_behaviors_motion
motion_launch=motion_behaviors_launch.py
if [[ -f /opt/uav_planner_ws/install/setup.bash ]]; then
  source /opt/uav_planner_ws/install/setup.bash
  motion_package=uav_follow_path_execution
  motion_launch=motion_behaviors.launch.py
fi
set -u

export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}

namespace="${AEROSTACK2_SIMULATION_DRONE_ID:-drone0}"
config="${AS2_CONFIG_FILE:-/opt/aerostack2-integration/config.yaml}"
controller_config="${AS2_CONTROLLER_CONFIG_FILE:-/opt/aerostack2-integration/pid_speed_controller.yaml}"
pids=()

cleanup() {
  trap - EXIT INT TERM
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

start() {
  "$@" &
  pids+=("$!")
}

start MicroXRCEAgent udp4 -p 8888
sleep 1
start ros2 launch as2_platform_pixhawk pixhawk_launch.py \
  namespace:="$namespace" platform_config_file:="$config"
start ros2 launch as2_state_estimator state_estimator_launch.py \
  namespace:="$namespace" config_file:="$config" plugin_name:=raw_odometry
start ros2 launch as2_motion_controller controller_launch.py \
  namespace:="$namespace" config_file:="$config" plugin_name:=pid_speed_controller \
  plugin_config_file:="$controller_config"
start ros2 launch "$motion_package" "$motion_launch" \
  namespace:="$namespace" config_file:="$config"

wait -n "${pids[@]}"
echo "Aerostack2 component exited unexpectedly" >&2
exit 1
