#!/usr/bin/env bash
# Read-only acceptance against an existing real AirSim -> navigation input chain.
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../.." && pwd)
results=${1:-$repo/artifacts/swarm-real-cloud}
mkdir -p "$results"
results=$(cd "$results" && pwd)
dds=${SWARM_DDS_PROFILE:-$repo/config/dds/fastdds_udp.xml}
dds=$(realpath "$dds")
domain=${ROS_DOMAIN_ID:-81}

docker run --rm --entrypoint bash \
  -v "$repo:/work:ro" -v "$results:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    set -e
    source /opt/ros/humble/setup.bash
    mkdir -p /results/workspace
    cd /results/workspace
    MAKEFLAGS=-j2 colcon build --base-paths /work/libraries/swarm_formation/humble \
      --packages-up-to plan_env --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DSWARM_BUILD_VALIDATION=ON > build.log 2>&1
  '
docker run --rm --network host --entrypoint bash \
  -e "ROS_DOMAIN_ID=$domain" -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/dds.xml \
  -v "$dds:/config/dds.xml:ro" -v "$repo:/work:ro" -v "$results:/results" \
  osrf/ros:humble-desktop-full-jammy -lc '
    source /results/workspace/install/setup.bash
    python3 /work/libraries/swarm_formation/tests/verify_real_cloud.py --output /results
  '
