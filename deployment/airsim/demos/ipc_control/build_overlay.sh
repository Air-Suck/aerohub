#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/ipc/control-overlay"}
image=${IPC_CONTROL_IMAGE:-uav/apps-super-forest:repo-native-20261109}
expected_as2=ca38fa8e537f9bfbe7451e733f8b3f0dda034f88
expected_pixhawk=4e5616408e0078abf38387d5b8c681f1387cfed6
actual_as2=$(git -C "$repo/as2/aerostack2" rev-parse HEAD)
actual_pixhawk=$(git -C "$repo/as2/as2_platform_pixhawk" rev-parse HEAD)
[[ $actual_as2 == "$expected_as2" ]] || {
  echo "Expected AS2 fork $expected_as2, found $actual_as2" >&2
  exit 1
}
[[ $actual_pixhawk == "$expected_pixhawk" ]] || {
  echo "Expected Pixhawk fork $expected_pixhawk, found $actual_pixhawk" >&2
  exit 1
}
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
docker image inspect "$image" --format '{{.Id}}' > "$workspace/image-id.txt"
python3 - "$workspace/source-provenance.json" "$actual_as2" "$actual_pixhawk" <<'PY_PROVENANCE'
import json
from pathlib import Path
import sys

Path(sys.argv[1]).write_text(json.dumps({
    'as2_fork': {
        'repository': 'https://github.com/Popc0rn7/aerostack2-aerohub.git',
        'commit': sys.argv[2],
    },
    'pixhawk_fork': {
        'repository': 'https://github.com/Popc0rn7/as2_platform-aerohub.git',
        'commit': sys.argv[3],
    },
}, indent=2) + '\n')
PY_PROVENANCE
docker run --rm --network none --entrypoint /bin/bash \
  -v "$repo:/repo:ro" -v "$workspace:/work" "$image" -lc '
    set -eo pipefail
    source /root/aerostack2_ws/install/setup.bash
    cd /work
    rm -rf build install log
    mkdir -p src
    rm -rf src/as2_motion_controller src/as2_platform_pixhawk
    cp -a /repo/as2/aerostack2/as2_motion_controller src/
    cp -a /repo/as2/as2_platform_pixhawk src/
    rm -f src/as2_motion_controller/.git src/as2_platform_pixhawk/.git
    if test ! -f src/as2_motion_controller/thirdparties/pid_controller/CMakeLists.txt; then
      mkdir -p src/as2_motion_controller/thirdparties
      cp -a /root/aerostack2_ws/src/aerostack2/as2_motion_controller/thirdparties/pid_controller \
        src/as2_motion_controller/thirdparties/pid_controller
    fi
    find src/as2_motion_controller/thirdparties/pid_controller -type f -print0 \
      | sort -z | xargs -0 sha256sum > pid-controller-files.sha256
    cd src/as2_platform_pixhawk
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
