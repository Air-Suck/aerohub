#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${AIRSIM_SENSOR_WORKSPACE:-$root/log/workspaces/airsim_sensors}
image=${FLIGHT_BOARD_IMAGE:-flight_board:v0.1.0}
source_dir=$root/simulator/airsim/sensors

docker image inspect "$image" >/dev/null
test -f "$source_dir/package.xml" || {
  printf 'AirSim sensor package is missing: %s\n' "$source_dir" >&2
  exit 2
}
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
docker image inspect "$image" > "$workspace/shared-image.json"

container=aerohub-airsim-sensor-prepare-$$
cleanup() {
  docker rm --force "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=airsim-sensor-workspace-prepare \
  --label "aerohub.workspace=$workspace" \
  --entrypoint bash \
  --env HOST_UID="$(id -u)" \
  --env HOST_GID="$(id -g)" \
  --volume "$root:/repo:ro" \
  --volume "$workspace:/workspace" \
  "$image" -lc '
set -eo pipefail
restore_ownership() {
  chown -R "$HOST_UID:$HOST_GID" /workspace/build /workspace/install /workspace/log 2>/dev/null || true
}
trap restore_ownership EXIT
source /opt/ros/humble/setup.bash
source /opt/platform_sensors/install/setup.bash
colcon --log-base /workspace/log build \
  --base-paths /repo/simulator/airsim/sensors \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --merge-install \
  --packages-select uav_as2_sensor_adapter
source /workspace/install/setup.bash
PYTHONPATH=/repo/simulator/airsim/sensors:${PYTHONPATH:-} \
  python3 -m unittest discover -s /repo/simulator/airsim/sensors/tests -v
'
trap - EXIT INT TERM

test -f "$workspace/install/setup.bash"
test -x "$workspace/install/lib/uav_as2_sensor_adapter/airsim_sensor_node"
{
  printf 'repository=%s\n' "$(git -C "$root" rev-parse HEAD)"
  printf 'shared_image=%s\n' "$(docker image inspect "$image" --format '{{.Id}}')"
  printf 'workspace=%s\n' "$workspace"
  find "$source_dir" -type f -not -path '*/__pycache__/*' -print0 \
    | sort -z | xargs -0 sha256sum
} > "$workspace/source-state.txt"
printf 'AIRSIM_SENSOR_WORKSPACE=%q\n' "$workspace"
