#!/usr/bin/env bash
set -euo pipefail
GCOPTER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
docker run --rm --entrypoint bash -v "$GCOPTER_ROOT:/workspace" 3e5d82c5dce5 -lc '
set -euo pipefail
cd /workspace
for package in log/gcopter/deps/*.deb; do dpkg-deb -x "$package" /; done
set +u
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/super_app_ws/install/setup.bash
source log/gcopter/install/setup.bash
set -u
export ROS_DOMAIN_ID=187
mkdir -p log/gcopter/ros-check
ros2 run uav_gcopter_core gcopter_function --ros-args -r __ns:=/drone0 > log/gcopter/ros-check/node.log 2>&1 &
GCOPTER_NODE_PID=$!
ros2 run uav_gcopter_adapter gcopter_adapter --ros-args -r __ns:=/drone0 > log/gcopter/ros-check/adapter.log 2>&1 &
GCOPTER_ADAPTER_PID=$!
trap '\''kill "$GCOPTER_NODE_PID" "$GCOPTER_ADAPTER_PID" 2>/dev/null || true; wait "$GCOPTER_NODE_PID" "$GCOPTER_ADAPTER_PID" 2>/dev/null || true'\'' EXIT
python3 src/planner/gcopter/ros2_adapter/native/tests/ros_check.py | tee log/gcopter/ros-check/result.json
'
