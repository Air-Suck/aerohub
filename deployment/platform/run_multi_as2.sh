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

# ROS 2 nodes run in separate Docker IPC namespaces. Fast DDS can discover
# peers through host networking and still select an unusable shared-memory
# path, so keep cross-container traffic on UDP.
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}

vehicle_count=${VEHICLE_COUNT:-4}
xrce_agent_port=${XRCE_AGENT_PORT:-8889}
config=${AS2_CONFIG_FILE:-/opt/aerostack2-integration/config.yaml}
default_platform_config_dir=/opt/aerostack2-integration/platform-multi
if [[ -d /workspace/deployment/platform/multi ]]; then
  default_platform_config_dir=/workspace/deployment/platform/multi
fi
platform_config_dir=${AS2_PLATFORM_CONFIG_DIR:-$default_platform_config_dir}
controller_config=${AS2_CONTROLLER_CONFIG_FILE:-/opt/aerostack2-integration/pid_speed_controller.yaml}
pids=()

if [[ ! -d "$platform_config_dir" ]]; then
  echo "AS2 platform config directory not found: $platform_config_dir" >&2
  exit 2
fi

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

start MicroXRCEAgent udp4 -p "$xrce_agent_port"
sleep 1

for ((i = 0; i < vehicle_count; ++i)); do
  namespace="drone$i"
  platform_config="$platform_config_dir/drone$i.yaml"
  if [[ ! -f "$platform_config" ]]; then
    echo "AS2 platform config not found: $platform_config" >&2
    exit 2
  fi
  platform_args=(
    namespace:="$namespace"
    platform_config_file:="$platform_config"
  )
  start ros2 launch as2_platform_pixhawk pixhawk_launch.py "${platform_args[@]}"
  start ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:="$namespace" config_file:="$config" plugin_name:=raw_odometry
  start ros2 launch as2_motion_controller controller_launch.py \
    namespace:="$namespace" config_file:="$config" plugin_name:=pid_speed_controller \
    plugin_config_file:="$controller_config"
  start ros2 launch "$motion_package" "$motion_launch" \
    namespace:="$namespace" config_file:="$config"
done

wait -n "${pids[@]}"
echo "Aerostack2 component exited unexpectedly" >&2
exit 1
