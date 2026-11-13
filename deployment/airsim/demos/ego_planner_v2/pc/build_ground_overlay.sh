#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
mkdir -p /work/ground_overlay
cd /work/ground_overlay
colcon --log-base log build --base-paths \
  /input/src/planner/interfaces \
  /input/src/behavior/follow_path \
  --build-base build --install-base install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
