#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
workspace=${1:-"$repo/log/ipc/native-deps"}
image=${IPC_CONTROL_IMAGE:-uav/apps-super-forest:repo-native-20261109}
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
# Dependency acquisition is explicit; no floating branches or image mutation.
for entry in 'osqp f9fc23d3436e4b17dd2cb95f70cfa1f37d122c24' 'osqp-eigen 0f3f8c7271931333b70f77a4db7039d764834ffe'; do
  read -r name revision <<< "$entry"
  test "$(git -C "$workspace/$name" rev-parse HEAD)" = "$revision"
  test -z "$(git -C "$workspace/$name" status --porcelain)"
done
test "$(git -C "$workspace/osqp/lin_sys/direct/qdldl/qdldl_sources" rev-parse HEAD)" = 7d16b70a10a152682204d745d814b6eb63dc5cd2
docker image inspect "$image" --format '{{.Id}}' > "$workspace/image-id.txt"
docker run --rm --network none --entrypoint bash \
  -v "$repo:/repo:ro" -v "$workspace:/deps" "$image" -lc '
    set -e
    cmake -S /deps/osqp -B /deps/build-osqp -DCMAKE_INSTALL_PREFIX=/deps/install -DUNITTESTS=OFF -DPRINTING=OFF
    cmake --build /deps/build-osqp -j2
    cmake --install /deps/build-osqp
    cmake -S /deps/osqp-eigen -B /deps/build-eigen -DCMAKE_PREFIX_PATH=/deps/install -DCMAKE_INSTALL_PREFIX=/deps/install -DBUILD_TESTING=OFF
    cmake --build /deps/build-eigen -j2
    cmake --install /deps/build-eigen
    cmake -S /repo/src/planner/ipc/ros2_adapter/native -B /deps/build-core -DCMAKE_PREFIX_PATH=/deps/install
    cmake --build /deps/build-core -j2
    cd /deps/build-core
    ctest --output-on-failure
  ' > "$workspace/build-core.log" 2>&1
