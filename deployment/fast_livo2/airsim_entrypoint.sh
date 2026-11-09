#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/fast_livo2_as2_ws/install/setup.bash
set -u

children=()
stop() {
  local pid
  for pid in "${children[@]}"; do kill -TERM "-$pid" 2>/dev/null || true; done
  for pid in "${children[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap stop EXIT INT TERM

setsid ros2 run uav_fast_livo2 bridge.py --ros-args -r __ns:=/drone0 \
  -p use_sim_time:=true -p forward_clock:=false -p cloud_frame:=drone0/livo_world \
  -p port:=18761 >"$FAST_LIVO_RESULTS/ros2-output-bridge.log" 2>&1 &
children+=("$!")
setsid python3 /opt/onboard_computer/run.py >"$FAST_LIVO_RESULTS/as2.log" 2>&1 &
children+=("$!")

python3 /opt/fast_livo2_as2_ws/airsim_mission.py
