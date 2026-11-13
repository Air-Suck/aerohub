#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /work/ground_overlay/install/local_setup.bash
exec ros2 launch /input/deployment/airsim/demos/ego_planner_v2/pc/ground.launch.py
