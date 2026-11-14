#!/usr/bin/env bash
set -euo pipefail
GCOPTER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
python3 "$GCOPTER_ROOT/deployment/gcopter/check_super_alignment.py"
docker run --rm --entrypoint bash -e ROS_DOMAIN_ID="${GCOPTER_TEST_DOMAIN:-186}" \
  -e GCOPTER_TEST_OUTPUT="/workspace/log/gcopter/${GCOPTER_TEST_NAME:-local-ros-check}" \
  -v "$GCOPTER_ROOT:/workspace" 3e5d82c5dce5 -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/log/gcopter/install/setup.bash
for package in /workspace/log/gcopter/deps/*.deb; do dpkg-deb -x "$package" /; done
python3 /workspace/src/tool/navigation/planner/gcopter/ros2_adapter/native/tests/local_ros_check.py
'
