#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${ROG_MAP_WORKSPACE:-$root/log/workspaces/rog_map}
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}
clean=${ROG_MAP_CLEAN_BUILD:-0}

docker image inspect "$image" >/dev/null
if [[ $workspace == / || -z $workspace ]]; then
  printf 'Refusing unsafe ROG_MAP_WORKSPACE: %s\n' "$workspace" >&2
  exit 2
fi
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
if [[ $clean == 1 ]]; then
  rm -rf "$workspace/build" "$workspace/install" "$workspace/log" "$workspace/test-log"
fi
docker image inspect "$image" > "$workspace/shared-image.json"

container=aerohub-rog-map-prepare-$$
cleanup() {
  docker rm --force "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=rog-map-workspace-prepare \
  --label "aerohub.workspace=$workspace" \
  --entrypoint bash \
  --env HOST_UID="$(id -u)" \
  --env HOST_GID="$(id -g)" \
  --volume "$root:/repo:ro" \
  --volume "$workspace:/workspace" \
  "$image" -lc '
set -eo pipefail
restore_ownership() {
  chown -R "$HOST_UID:$HOST_GID" /workspace 2>/dev/null || true
}
trap restore_ownership EXIT

mkdir -p /workspace/src
rm -rf /workspace/src/rog_map /workspace/src/uav_rog_map \
  /workspace/src/uav_rog_map_interfaces /workspace/src/as2_navigation_inputs
cp -a /repo/src/planner/super/ros2_adapter/native/rog_map /workspace/src/rog_map
cp -a /repo/src/perception/rog_map/interfaces /workspace/src/uav_rog_map_interfaces
cp -a /repo/src/perception/rog_map/ros2_adapter /workspace/src/uav_rog_map
cp -a /repo/src/perception/navigation_inputs /workspace/src/as2_navigation_inputs

source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
colcon --log-base /workspace/log build \
  --base-paths /workspace/src \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --merge-install \
  --executor sequential \
  --packages-up-to uav_rog_map as2_navigation_inputs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /workspace/install/setup.bash
colcon --log-base /workspace/test-log test \
  --base-paths /workspace/src \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --merge-install \
  --packages-select rog_map uav_rog_map_interfaces uav_rog_map as2_navigation_inputs \
  --ctest-args -E "^(copyright|cppcheck|cpplint|flake8|lint_cmake|pep257|uncrustify|xmllint)$" \
  --event-handlers console_direct+
colcon test-result --test-result-base /workspace/build --verbose
'
trap - EXIT INT TERM

for artifact in \
  "$workspace/install/setup.bash" \
  "$workspace/install/lib/librog_map.a" \
  "$workspace/install/lib/uav_rog_map/rog_map_node" \
  "$workspace/install/lib/as2_navigation_inputs/navigation_inputs_node"; do
  if [[ ! -e $artifact ]]; then
    printf 'Required ROG-Map workspace artifact is missing: %s\n' "$artifact" >&2
    exit 1
  fi
done

{
  printf 'repository=%s\n' "$(git -C "$root" rev-parse HEAD)"
  printf 'repository_status_begin\n'
  git -C "$root" status --short
  printf 'repository_status_end\n'
  printf 'shared_image=%s\n' "$(docker image inspect "$image" --format '{{.Id}}')"
  printf 'active_rog_map=src/planner/super/ros2_adapter/native/rog_map\n'
  printf 'reference_super=3bfa4033361bbdc7cd814c94aff826e6e1679018\n'
  printf 'imported_source=148116eee43112b2bee242b2480d147a2c1f4e3d\n'
  printf 'official_ros1_head=df59c21304579a13fb3875100f8ce9523ba379a0\n'
  printf 'workspace=%s\n' "$workspace"
} > "$workspace/source-state.txt"

printf 'ROG_MAP_WORKSPACE=%q\n' "$workspace"
