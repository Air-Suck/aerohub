#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
work="${YOPO_ROS_WORKSPACE:-$repo/log/yopo-single/ros_ws}"
[[ -n "${ROS_DISTRO:-}" ]] || { echo 'Source ROS Humble and matching AS2 overlay first' >&2; exit 1; }
mkdir -p "$work"
cd "$work"
exec colcon build --base-paths "$repo/src/planner/interfaces" "$repo/src/inference/interfaces" "$repo/src/inference/ros2_adapter" "$repo/src/planner/yopo/ros2_adapter" --packages-select uav_planner_interfaces uav_inference_interfaces uav_inference uav_yopo_planner --event-handlers console_direct+
