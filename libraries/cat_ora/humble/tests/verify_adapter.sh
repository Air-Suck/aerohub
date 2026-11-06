#!/usr/bin/env bash
# Exercise the real native planner through the existing four-vehicle adapter.
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../../.." && pwd)
result_dir=${1:-/tmp/aerohub-catora-adapter-validation}
mkdir -p "$result_dir"
result_dir=$(cd "$result_dir" && pwd)
docker run --rm --network none --entrypoint bash \
  -v "$repo:/work:ro" -v "$result_dir:/results" \
  onboard_computer:airsim-px4 -lc '
    set -e
    source /root/aerostack2_ws/install/setup.bash
    export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
    cd /results
    colcon build --base-paths \
      /work/libraries/cat_ora/humble /work/planner_interfaces \
      /work/planner_framework /work/planner_framework/bridges/as2_setpoint \
      /work/libraries/cat_ora/ros2_adapter > build-output.log 2>&1
    source install/setup.bash
    ros2 run mrs_formation_reshaping cat_ora_node > native-node.log 2>&1 &
    node_pid=$!
    ros2 run mrs_formation_reshaping ros2_proxy.py > native-proxy.log 2>&1 &
    proxy_pid=$!
    trap "kill $node_pid $proxy_pid 2>/dev/null || true" EXIT
    bash /work/libraries/cat_ora/ros2_adapter/scripts/smoke_real_ros1.sh \
      /work /results/install /results/resolved 179
  '
