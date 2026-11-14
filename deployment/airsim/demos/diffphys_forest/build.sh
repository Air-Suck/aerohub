#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace="$repo/log/diffphys/attitude-overlay"
bash "$repo/deployment/airsim/demos/attitude_control/build_overlay.sh" "$workspace"
docker run --rm --network none --entrypoint /bin/bash -v "$repo:/repo:ro" -v "$workspace:/work" "${IPC_CONTROL_IMAGE:-uav/apps-super-forest:repo-native-20261109}" -lc '
set -eo pipefail
source /root/aerostack2_ws/install/setup.bash
source /work/install/setup.bash
cd /work
MAKEFLAGS=-j2 colcon build --base-paths /repo/src/basic_function/inference/interfaces /repo/src/basic_function/inference/ros2_adapter /repo/src/tool/navigation/planner/diffphys/ros2_adapter --merge-install --parallel-workers 1 --packages-select uav_inference_interfaces uav_inference uav_diffphys_planner --event-handlers console_direct+ > diffphys-build.log 2>&1
'
