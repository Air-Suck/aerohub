#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /work/install/humble/local_setup.bash
source /root/aerostack2_ws/install/setup.bash
source /work/ground_overlay/install/local_setup.bash
source /work/ego_runtime/install/local_setup.bash
exec ros2 launch uav_ego_planner_adapter ego_as2.launch.py \
  resolved_config:="${EGO_CONFIG_FILE:-/input/deployment/airsim/demos/ego_planner_v2/pc/ego_single.yaml}" use_sim_time:=true
