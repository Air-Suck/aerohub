#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
work_dir="${INFERENCE_ROS_WORKSPACE:-$repo_dir/log/inference/ros_ws}"
if ! command -v colcon >/dev/null || [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Load the ROS Humble environment before running this script" >&2
  exit 1
fi
mkdir -p "$work_dir"
cd "$work_dir"
exec colcon build --base-paths "$repo_dir/src/basic_function/inference/interfaces" "$repo_dir/src/basic_function/inference/ros2_adapter" --packages-select uav_inference_interfaces uav_inference --event-handlers console_direct+
