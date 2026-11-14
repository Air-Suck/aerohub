#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${FAST_LIVO2_WORKSPACE:-$root/log/workspaces/fast_livo2}
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}

declare -A locked_sources=(
  [src/state_estimator/fast_livo2/ros2_upstream]=a713004f0ba0624c8fb80d85c7047fe62523c6fb
  [src/state_estimator/fast_livo2/dependencies/livox_ros_driver2]=b6ff7d1c8b210a96b74e919823771e1e32249758
  [src/state_estimator/fast_livo2/dependencies/rpg_vikit]=4afd27502eeb204dff9963fbbeb16ac92a369b95
  [src/state_estimator/fast_livo2/dependencies/sophus]=de0f8d3d92bf776271e16de56d1803940ebccab9
)

docker image inspect "$image" >/dev/null
for source_rel in "${!locked_sources[@]}"; do
  source_dir=$root/$source_rel
  expected=${locked_sources[$source_rel]}
  if [[ ! -e $source_dir/.git ]]; then
    printf 'FAST-LIVO2 submodule is not initialized: %s\n' "$source_rel" >&2
    printf 'Run: git submodule update --init -- %s\n' "$source_rel" >&2
    exit 2
  fi
  if [[ $(git -C "$source_dir" rev-parse --show-toplevel) != "$source_dir" ]]; then
    printf 'Submodule path is not an independent Git worktree: %s\n' "$source_rel" >&2
    exit 2
  fi
  gitlink=$(git -C "$root" ls-files -s "$source_rel" | awk '$1 == "160000" {print $2}')
  checkout=$(git -C "$source_dir" rev-parse HEAD)
  if [[ $gitlink != "$expected" || $checkout != "$expected" ]]; then
    printf 'Source mismatch: path=%s expected=%s gitlink=%s checkout=%s\n' \
      "$source_rel" "$expected" "${gitlink:-missing}" "$checkout" >&2
    exit 2
  fi
  if [[ -n $(git -C "$source_dir" status --short --untracked-files=no) ]]; then
    printf 'Submodule has tracked modifications: %s\n' "$source_rel" >&2
    git -C "$source_dir" status --short --untracked-files=no >&2
    exit 2
  fi
done

# The lightweight interface package must remain wire-compatible with the
# locked Livox messages while avoiding the hardware SDK in AirSim deployments.
normalize_msg() {
  sed -E 's/#.*$//; /^[[:space:]]*$/d; s/[[:space:]]+/ /g; s/^ //; s/ $//' "$1"
}
for message in CustomMsg.msg CustomPoint.msg; do
  if ! diff -u \
    <(normalize_msg "$root/src/state_estimator/fast_livo2/dependencies/livox_ros_driver2/msg/$message") \
    <(normalize_msg "$root/src/state_estimator/fast_livo2/ros2_adapter/livox_ros_driver2/msg/$message"); then
    printf 'Lightweight Livox interface differs from locked dependency: %s\n' "$message" >&2
    exit 2
  fi
done

mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
docker image inspect "$image" > "$workspace/shared-image.json"

container=aerohub-fast-livo2-prepare-$$
cleanup() {
  docker rm --force "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=fast-livo2-workspace-prepare \
  --label "aerohub.workspace=$workspace" \
  --entrypoint bash \
  --env HOST_UID="$(id -u)" \
  --env HOST_GID="$(id -g)" \
  --volume "$root:/repo:ro" \
  --volume "$workspace:/workspace" \
  "$image" -lc '
set -eo pipefail
restore_ownership() {
  chown -R "$HOST_UID:$HOST_GID" \
    /workspace/src /workspace/build /workspace/install /workspace/log \
    /workspace/prefix /workspace/sophus-build 2>/dev/null || true
}
trap restore_ownership EXIT

mkdir -p /workspace/src
for package in fast_livo vikit_common vikit_ros livox_ros_driver2 \
               uav_fast_livo2_ros2 uav_fast_livo2; do
  rm -rf "/workspace/src/$package"
done
# Copy rather than symlink fast_livo: its upstream CMake resolves vikit from
# ../../install relative to the source tree.
cp -a /repo/src/state_estimator/fast_livo2/ros2_upstream /workspace/src/fast_livo
cp -a /repo/src/state_estimator/fast_livo2/dependencies/rpg_vikit/vikit_common /workspace/src/vikit_common
cp -a /repo/src/state_estimator/fast_livo2/dependencies/rpg_vikit/vikit_ros /workspace/src/vikit_ros
cp -a /repo/src/state_estimator/fast_livo2/ros2_adapter/livox_ros_driver2 /workspace/src/livox_ros_driver2
cp -a /repo/src/state_estimator/fast_livo2/ros2_adapter/uav_fast_livo2_ros2 /workspace/src/uav_fast_livo2_ros2
cp -a /repo/src/state_estimator/fast_livo2/plugin /workspace/src/uav_fast_livo2
mkdir -p /workspace/src/fast_livo/Log

cmake -S /repo/src/state_estimator/fast_livo2/dependencies/sophus \
  -B /workspace/sophus-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/workspace/prefix \
  -DBUILD_SOPHUS_TESTS=OFF \
  -DBUILD_SOPHUS_EXAMPLES=OFF \
  -DSOPHUS_INSTALL=ON
cmake --build /workspace/sophus-build --parallel 2
cmake --install /workspace/sophus-build

source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
colcon --log-base /workspace/log build \
  --base-paths /workspace/src \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --executor sequential \
  --packages-select livox_ros_driver2 vikit_common vikit_ros fast_livo \
                    uav_fast_livo2_ros2 uav_fast_livo2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/workspace/prefix
'
trap - EXIT INT TERM

for required in \
  "$workspace/install/setup.bash" \
  "$workspace/install/fast_livo/lib/fast_livo/fastlivo_mapping" \
  "$workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/pointcloud_to_livox" \
  "$workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/sensor_qos_adapter" \
  "$workspace/install/uav_fast_livo2_ros2/lib/uav_fast_livo2_ros2/output_adapter" \
  "$workspace/install/uav_fast_livo2/lib/libfast_livo2_plugin.so"; do
  if [[ ! -e $required ]]; then
    printf 'Required workspace artifact is missing: %s\n' "$required" >&2
    exit 1
  fi
done

{
  printf 'repository=%s\n' "$(git -C "$root" rev-parse HEAD)"
  printf 'shared_image=%s\n' "$(docker image inspect "$image" --format '{{.Id}}')"
  printf 'workspace=%s\n' "$workspace"
  for source_rel in "${!locked_sources[@]}"; do
    printf 'source=%s %s %s\n' "$source_rel" "${locked_sources[$source_rel]}" \
      "$(git -C "$source_rel" remote get-url origin)"
  done
  git -C "$root" status --short
} > "$workspace/source-state.txt"

printf 'FAST_LIVO2_WORKSPACE=%q\n' "$workspace"
