#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/ipc/plugin-build"}
image=${IPC_PLUGIN_BUILD_IMAGE:-uav/apps-super-forest:repo-native-20261109}
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
run="$(date -u +%Y%m%dT%H%M%SZ)-$$"
docker image inspect "$image" --format '{{.Id}}' > "$workspace/image-$run.txt"
docker run --rm --network none --entrypoint /bin/bash \
  -e ROS_DOMAIN_ID=184 -e VERIFY_RUN="$run" \
  -v "$repo:/repo:ro" -v "$workspace:/work" "$image" -lc '
    set -euo pipefail
    set +u
    source /root/aerostack2_ws/install/setup.bash
    set -u
    cd /work
    mkdir -p "runs/$VERIFY_RUN"
    MAKEFLAGS=-j2 colcon build --base-paths /repo/src/behavior/follow_reference \
      --merge-install --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
      > "runs/$VERIFY_RUN/build.log" 2>&1
    set +u
    source install/setup.bash
    set -u
    ctest --test-dir build/uav_follow_reference_acro --output-on-failure \
      > "runs/$VERIFY_RUN/ctest.log" 2>&1
    ros2 run uav_follow_reference_acro plugin_load_test \
      > "runs/$VERIFY_RUN/load.log" 2>&1
    python3 /repo/src/behavior/follow_reference/tests/interface_smoke.py \
      > "runs/$VERIFY_RUN/interfaces.log" 2>&1
    python3 /repo/src/behavior/follow_reference/tests/interface_smoke.py --disabled \
      > "runs/$VERIFY_RUN/disabled.log" 2>&1
    echo "PASS: runs/$VERIFY_RUN"
  '
