#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/ipc/control-overlay"}
image=${IPC_CONTROL_IMAGE:-uav/apps-super-forest:repo-native-20261109}
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
docker image inspect "$image" --format '{{.Id}}' > "$workspace/image-id.txt"
docker run --rm --network none --entrypoint /bin/bash \
  -v "$repo:/repo:ro" -v "$workspace:/work" "$image" -lc '
    set -eo pipefail
    source /root/aerostack2_ws/install/setup.bash
    cd /work
    mkdir -p src
    if test ! -d src/as2_motion_controller; then
      cp -a /root/aerostack2_ws/src/aerostack2/as2_motion_controller src/
    fi
    python3 /repo/deployment/airsim/demos/ipc_control/check_build_input.py \
      src/as2_motion_controller/src/controller_handler.cpp \
      /repo/deployment/platform/patches/as2-body-rates-bypass.json \
      /repo/deployment/platform/patches/as2-body-rates-bypass.patch > source-provenance.json
    cd src/as2_motion_controller
    if git apply --check /repo/deployment/platform/patches/as2-body-rates-bypass.patch 2>/dev/null; then
      git apply /repo/deployment/platform/patches/as2-body-rates-bypass.patch
    else
      git apply --reverse --check /repo/deployment/platform/patches/as2-body-rates-bypass.patch
    fi
    cd /work/src/as2_motion_controller
    python3 /repo/deployment/airsim/demos/ipc_control/check_build_input.py \
      src/controller_handler.cpp /repo/deployment/platform/patches/as2-body-rates-source-stamp.json \
      /repo/deployment/platform/patches/as2-body-rates-source-stamp.patch > /work/stamp-provenance.json
    if git apply --check /repo/deployment/platform/patches/as2-body-rates-source-stamp.patch 2>/dev/null; then
      git apply /repo/deployment/platform/patches/as2-body-rates-source-stamp.patch
    else
      git apply --reverse --check /repo/deployment/platform/patches/as2-body-rates-source-stamp.patch
    fi
    cd /work
    if test ! -d src/as2_platform_pixhawk; then
      cp -a /root/aerostack2_ws/src/as2_platform_pixhawk src/
    fi
    cd src/as2_platform_pixhawk
    python3 /repo/deployment/airsim/demos/ipc_control/check_build_input.py \
      src/pixhawk_platform.cpp /repo/deployment/platform/patches/pixhawk-body-rates-pairing.json \
      /repo/deployment/platform/patches/pixhawk-body-rates-pairing.patch > /work/platform-provenance.json
    if git apply --check /repo/deployment/platform/patches/pixhawk-body-rates-pairing.patch 2>/dev/null; then
      git apply /repo/deployment/platform/patches/pixhawk-body-rates-pairing.patch
    else
      git apply --reverse --check /repo/deployment/platform/patches/pixhawk-body-rates-pairing.patch
    fi
    python3 - <<"PY_BUILD"
from pathlib import Path
p=Path("CMakeLists.txt");text=p.read_text();line="include(/repo/deployment/airsim/demos/ipc_control/tests/pairing.cmake)"
if line not in text:p.write_text(text+"\n"+line+"\n")
PY_BUILD
    cd /work
    MAKEFLAGS=-j2 colcon build --base-paths src/as2_motion_controller src/as2_platform_pixhawk \
      /repo/src/behavior/follow_reference --merge-install --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF \
      > build.log 2>&1
    source install/local_setup.bash
    ROS_DOMAIN_ID=185 build/as2_platform_pixhawk/ipc_pairing_test > pairing-test.log 2>&1
  '
