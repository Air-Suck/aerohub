#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${AS2_EXECUTION_WORKSPACE:-$root/log/workspaces/as2_execution}
px4_workspace=${PX4_MSGS_OVERLAY_WORKSPACE:-$root/log/workspaces/px4_msgs-61907962c9220437ec01a98fa26bfbf5cbb254b5}
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}
as2_rel=as2/aerostack2
pixhawk_rel=as2/as2_platform_pixhawk
as2_commit=ca38fa8e537f9bfbe7451e733f8b3f0dda034f88
pixhawk_commit=4e5616408e0078abf38387d5b8c681f1387cfed6

docker image inspect "$image" >/dev/null
test -f "$px4_workspace/install/setup.bash" || {
  printf 'PX4 message overlay is missing: %s\n' "$px4_workspace" >&2
  exit 2
}
for specification in "$as2_rel:$as2_commit" "$pixhawk_rel:$pixhawk_commit"; do
  source_rel=${specification%%:*}
  expected=${specification#*:}
  gitlink=$(git -C "$root" ls-files -s "$source_rel" | awk '$1 == "160000" {print $2}')
  checkout=$(git -C "$root/$source_rel" rev-parse HEAD)
  if [[ $gitlink != "$expected" || $checkout != "$expected" ]]; then
    printf 'Source mismatch: path=%s expected=%s gitlink=%s checkout=%s\n' \
      "$source_rel" "$expected" "${gitlink:-missing}" "$checkout" >&2
    exit 2
  fi
done

if [[ -e $workspace ]] && [[ -n $(find "$workspace" -mindepth 1 -maxdepth 1 -print -quit) ]]; then
  printf 'Execution overlay workspace is not empty: %s\n' "$workspace" >&2
  exit 2
fi
mkdir -p "$workspace/src"
workspace=$(cd "$workspace" && pwd)

# Export committed trees only. Local formatting or diagnostic edits inside a
# submodule must not become an implicit build input.
git -C "$root/$as2_rel" archive "$as2_commit" as2_motion_controller \
  | tar -x -C "$workspace/src"
mkdir -p "$workspace/src/as2_platform_pixhawk"
git -C "$root/$pixhawk_rel" archive "$pixhawk_commit" \
  | tar -x -C "$workspace/src/as2_platform_pixhawk"
docker image inspect "$image" > "$workspace/shared-image.json"

container=aerohub-as2-execution-prepare-$$
cleanup() {
  docker rm --force "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=as2-execution-workspace-prepare \
  --entrypoint bash \
  --env HOST_UID="$(id -u)" \
  --env HOST_GID="$(id -g)" \
  --volume "$workspace:/workspace" \
  --volume "$px4_workspace:/px4-overlay:ro" \
  "$image" -lc '
set -eo pipefail
restore_ownership() {
  chown -R "$HOST_UID:$HOST_GID" /workspace 2>/dev/null || true
}
trap restore_ownership EXIT
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /px4-overlay/install/setup.bash
colcon --log-base /workspace/log build \
  --base-paths /workspace/src \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --merge-install \
  --executor sequential \
  --packages-select as2_motion_controller as2_platform_pixhawk \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /workspace/install/setup.bash
colcon --log-base /workspace/test-log test \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --merge-install \
  --packages-select as2_motion_controller as2_platform_pixhawk \
  --ctest-args -E "^(copyright|cppcheck|cpplint|flake8|lint_cmake|pep257|uncrustify|xmllint)$" \
  --event-handlers console_direct+
colcon test-result --test-result-base /workspace/build --verbose
'
trap - EXIT INT TERM

test -f "$workspace/install/setup.bash"
test -x "$workspace/install/lib/as2_platform_pixhawk/as2_platform_pixhawk_node"
{
  printf 'repository=%s\n' "$(git -C "$root" rev-parse HEAD)"
  printf 'shared_image=%s\n' "$(docker image inspect "$image" --format '{{.Id}}')"
  printf 'px4_overlay=%s\n' "$px4_workspace"
  printf 'as2=%s %s\n' "$as2_commit" "$(git -C "$root/$as2_rel" remote get-url origin)"
  printf 'pixhawk=%s %s\n' "$pixhawk_commit" \
    "$(git config -f "$root/.gitmodules" --get submodule.pixhawk.url)"
  printf '%s\n' \
    'tests=runtime_and_interface' \
    'excluded_lint=copyright,cppcheck,cpplint,flake8,lint_cmake,pep257,uncrustify,xmllint'
} > "$workspace/source-state.txt"
printf 'AS2_EXECUTION_WORKSPACE=%q\n' "$workspace"
