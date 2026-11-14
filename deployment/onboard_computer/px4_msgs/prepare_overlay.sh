#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd "$script_dir/../../.." && pwd -P)"
source_dir="$repo_root/as2/px4_msgs"
overrides_dir="$script_dir/overrides"
source_commit="86d8239e962f6939e05c3737784f60c02fa884db"
firmware_commit="61907962c9220437ec01a98fa26bfbf5cbb254b5"
image="${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}"
workspace="${PX4_MSGS_OVERLAY_WORKSPACE:-$repo_root/log/workspaces/px4_msgs-$firmware_commit}"

if [[ "$workspace" != /* ]]; then
  workspace="$repo_root/$workspace"
fi

if [[ ! -d "$source_dir" ]] ||
   [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$source_commit" ]]; then
  echo "px4_msgs must be initialized at $source_commit" >&2
  exit 1
fi
if [[ -n "$(git -C "$source_dir" status --porcelain)" ]]; then
  echo "px4_msgs source has local changes" >&2
  exit 1
fi

(cd "$overrides_dir" && sha256sum -c SHA256SUMS)
docker image inspect "$image" >/dev/null

if [[ -e "$workspace" ]] &&
   [[ -n "$(find "$workspace" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "overlay workspace is not empty: $workspace" >&2
  echo "choose a fresh PX4_MSGS_OVERLAY_WORKSPACE to preserve existing evidence" >&2
  exit 1
fi
mkdir -p "$workspace"
workspace="$(cd "$workspace" && pwd -P)"

container="aerohub-px4-msgs-overlay-$$"
cleanup() {
  docker rm -f "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker run --rm --name "$container" --network none --entrypoint /bin/bash -v "$repo_root:/repo:ro" -v "$workspace:/workspace" "$image" -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    mkdir -p /workspace/src
    cp -a /repo/as2/px4_msgs /workspace/src/px4_msgs
    (
      cd /repo/deployment/onboard_computer/px4_msgs/overrides
      sha256sum -c SHA256SUMS
    )
    cp /repo/deployment/onboard_computer/px4_msgs/overrides/*.msg /workspace/src/px4_msgs/msg/
    cd /workspace
    MAKEFLAGS=-j2 colcon build --merge-install --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    source /workspace/install/setup.bash
    python3 - <<"PY"
from px4_msgs.msg import FailsafeFlags, VehicleStatus

failsafe = FailsafeFlags.get_fields_and_field_types()
status = VehicleStatus.get_fields_and_field_types()
required_failsafe = {
    "fd_alt_loss",
    "parachute_unhealthy",
    "traffic_avoidance_unhealthy",
    "remote_id_unhealthy",
    "gnss_lost",
}
required_status = {
    "nav_state_display",
    "accepts_offboard_setpoints",
    "traffic_avoidance_system_present",
}
assert required_failsafe <= failsafe.keys()
assert required_status <= status.keys()
print("validated PX4 1.18 diagnostic message fields")
PY
  '

image_id="$(docker image inspect "$image" --format '{{.Id}}')"
{
  printf 'onboard_image=%s\n' "$image"
  printf 'onboard_image_id=%s\n' "$image_id"
  printf 'px4_msgs_commit=%s\n' "$source_commit"
  printf 'px4_firmware_commit=%s\n' "$firmware_commit"
} > "$workspace/source-state.env"
(cd "$overrides_dir" && sha256sum -c SHA256SUMS) \
  > "$workspace/overrides-sha256.txt"

echo "PX4 message overlay ready: $workspace/install/setup.bash"
