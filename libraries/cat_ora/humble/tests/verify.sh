#!/usr/bin/env bash
# Reproduce the migration comparison without installing ROS on the host.
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../../.." && pwd)
result_dir=${1:-/tmp/aerohub-catora-validation}
mkdir -p "$result_dir"
result_dir=$(cd "$result_dir" && pwd)

docker run --rm --entrypoint bash \
  -v "$repo:/work:ro" -v "$result_dir:/results" \
  uav/catora:noetic-upstream -lc '
    source /opt/catora_ws/devel/setup.bash
    roslaunch mrs_formation_reshaping catora.launch rviz:=false > /results/ros1.log 2>&1 &
    planner=$!
    trap "kill $planner 2>/dev/null || true" EXIT
    python3 /work/libraries/cat_ora/humble/tests/capture_ros1.py /results/baseline.json
  '

docker run --rm --entrypoint bash \
  -v "$repo:/work:ro" -v "$result_dir:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    set -e
    source /opt/ros/humble/setup.bash
    cd /results
    colcon build --base-paths /work/libraries/cat_ora/humble --cmake-args -DCMAKE_BUILD_TYPE=Release > build-output.log 2>&1
    source install/setup.bash
    export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
    ros2 run mrs_formation_reshaping cat_ora_node > node.log 2>&1 &
    node_pid=$!
    ros2 run mrs_formation_reshaping ros2_proxy.py > proxy.log 2>&1 &
    proxy_pid=$!
    trap "kill $node_pid $proxy_pid 2>/dev/null || true" EXIT
    python3 /work/libraries/cat_ora/humble/tests/verify_service.py /results/baseline.json
  '
