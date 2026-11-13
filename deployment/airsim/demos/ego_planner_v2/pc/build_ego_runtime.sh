#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /work/install/humble/local_setup.bash
source /root/aerostack2_ws/install/setup.bash
source /work/ground_overlay/install/local_setup.bash
mkdir -p /work/ego_runtime
cd /work/ego_runtime
colcon --log-base log build --base-paths \
  /input/src/planner/framework \
  /input/src/planner/ego_planner_v2/ros2_adapter/bridge \
  --build-base build --install-base install --executor sequential
