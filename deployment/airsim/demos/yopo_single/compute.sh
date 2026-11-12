#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
source /opt/ros/humble/setup.bash
# Stable onboard image's own AS2 runtime; do not mix another checkout's libraries.
source /root/aerostack2_ws/install/setup.bash
source "${YOPO_OVERLAY:-$repo/log/yopo-single/ros_ws/install}/setup.bash"
set -u
export AS2_CONFIG_FILE="$repo/deployment/airsim/config/as2/config.yaml"
export AS2_CONTROLLER_CONFIG_FILE="$repo/deployment/airsim/demos/yopo_single/controller.yaml"
export AS2_FLEET_FILE="${YOPO_FLEET_FILE:?set YOPO_FLEET_FILE}"
export AS2_EXECUTION_BACKEND=follow_path
export AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=0.5
unset AS2_TF_SCOPE  # Single vehicle in a private DDS domain; sensor bridge uses /tf_static.
exec python3 "$repo/deployment/airsim/demos/yopo_single/compute.py"
