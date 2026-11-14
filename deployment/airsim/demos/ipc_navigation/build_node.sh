#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/ipc/native-deps"}
control=${2:-"$repo/log/ipc/control-overlay"}
image=${IPC_CONTROL_IMAGE:-uav/apps-super-forest:repo-native-20261109}
test -f "$workspace/install/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake"
docker run --rm --network none --entrypoint bash \
 -v "$repo:/repo:ro" -v "$workspace:/deps" -v "$control:/control:ro" "$image" -lc '
 set -e
 source /root/aerostack2_ws/install/setup.bash
 source /control/install/setup.bash
 cd /deps
 CMAKE_PREFIX_PATH=/deps/install:$CMAKE_PREFIX_PATH MAKEFLAGS=-j2 colcon build \
 --base-paths /repo/src/tool/navigation/planner/ipc/ros2_adapter/native --merge-install \
 --install-base ros-install --build-base ros-build --parallel-workers 1 \
 --cmake-args -DCMAKE_BUILD_TYPE=Release
 cd ros-build/ipc_native_core
 ctest --output-on-failure
 ' > "$workspace/node-build.log" 2>&1
