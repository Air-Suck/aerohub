#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
work_dir="${INFERENCE_ROS_WORKSPACE:-$repo_dir/log/inference/yolo26n_seg_ros_ws}"
if ! command -v colcon >/dev/null || [[ "${ROS_DISTRO:-}" != humble ]]; then
  echo 'Load ROS 2 Humble and its build tools first' >&2
  exit 1
fi
mkdir -p "$work_dir"
cd "$work_dir"
exec colcon build --base-paths "$repo_dir/src/inference/interfaces" "$repo_dir/src/inference/ros2_adapter" "$repo_dir/src/perception/object_detection" --packages-select uav_inference_interfaces uav_inference uav_object_detection_interfaces uav_object_detection --event-handlers console_direct+
