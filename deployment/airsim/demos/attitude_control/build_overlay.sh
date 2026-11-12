#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/diffphys/attitude-overlay"}
if [[ ! -f "$workspace/as2-attitude-pairing-provenance.json" ]]; then
  bash "$repo/deployment/airsim/demos/ipc_control/build_overlay.sh" "$workspace"
fi
docker run --rm --network none --entrypoint /bin/bash -v "$repo:/repo:ro" -v "$workspace:/work" "${IPC_CONTROL_IMAGE:-uav/apps-super-forest:repo-native-20261109}" -lc '
set -eo pipefail
source /root/aerostack2_ws/install/setup.bash
for entry in as2_motion_controller:as2-attitude-pairing as2_platform_pixhawk:pixhawk-attitude-pairing; do
 package=${entry%%:*}; patch=${entry#*:}
 cd /work/src/$package
 python3 /repo/deployment/airsim/demos/ipc_control/check_build_input.py src/$(test "$package" = as2_motion_controller && echo controller_handler || echo pixhawk_platform).cpp /repo/deployment/platform/patches/$patch.json /repo/deployment/platform/patches/$patch.patch > /work/$patch-provenance.json
 if git apply --check /repo/deployment/platform/patches/$patch.patch 2>/dev/null; then git apply /repo/deployment/platform/patches/$patch.patch; else git apply --reverse --check /repo/deployment/platform/patches/$patch.patch; fi
done
cd /work
MAKEFLAGS=-j2 colcon build --base-paths src/as2_motion_controller src/as2_platform_pixhawk /repo/src/behavior/follow_reference --merge-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF > attitude-build.log 2>&1
source install/local_setup.bash
ROS_DOMAIN_ID=185 build/as2_platform_pixhawk/ipc_pairing_test --attitude > attitude-pairing-test.log 2>&1
'
