#!/usr/bin/env bash
# Rebuild both versions, compare identical inputs, then exercise native rolling planning.
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../.." && pwd)
results=${1:-/tmp/aerohub-swarm-validation}
mkdir -p "$results"
results=$(cd "$results" && pwd)

docker run --rm --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  uav/catora:noetic-upstream -lc '
    set -e
    bash /work/libraries/swarm_formation/tests/build_baseline.sh /work /results/noetic
  '
docker run --rm --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    set -e
    source /opt/ros/humble/setup.bash
    mkdir -p /results/humble
    cd /results/humble
    MAKEFLAGS=-j2 colcon build --base-paths /work/libraries/swarm_formation/humble \
      /work/planner_interfaces /work/libraries/swarm_formation/ros2_adapter \
      --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DSWARM_BUILD_VALIDATION=ON \
      > build-output.log 2>&1
  '
# Run measurements sequentially, after both builds finish.
docker run --rm --network none --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  uav/catora:noetic-upstream -lc '
    set -e
    source /results/noetic/devel/setup.bash
    export ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311
    roscore > /results/noetic/roscore.log 2>&1 & master=$!
    trap "kill $master 2>/dev/null || true" EXIT
    python3 /work/libraries/swarm_formation/tests/run_benchmark.py ros1 /results/noetic /results/noetic/benchmark.json
  '
docker run --rm --network none --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    set -e
    source /results/humble/install/setup.bash
    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=183
    python3 /work/libraries/swarm_formation/tests/run_benchmark.py ros2 /results/humble /results/humble/benchmark.json
    python3 /work/libraries/swarm_formation/tests/compare_benchmark.py \
      /results/noetic/benchmark.json /results/humble/benchmark.json /results/comparison.json
  '
docker run --rm --network none --memory=7g --memory-swap=8g --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    set -e
    source /results/humble/install/setup.bash
    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=183
    ros2 launch ego_planner seven_planners.launch.py input_mode:=legacy > /results/humble/runtime.log 2>&1 & launch_pid=$!
    trap "kill -TERM $launch_pid 2>/dev/null || true; wait $launch_pid || true" EXIT
    python3 /work/libraries/swarm_formation/tests/verify_runtime.py /results/humble/runtime-summary.json
    python3 /work/libraries/swarm_formation/tests/verify_adapter.py
  '
docker run --rm --network none --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    set -e
    source /results/humble/install/setup.bash
    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184
    python3 /work/libraries/swarm_formation/tests/verify_navigation_bridge.py
    export ROS_DOMAIN_ID=185
    python3 /work/libraries/swarm_formation/tests/verify_proxy_health.py
  '
