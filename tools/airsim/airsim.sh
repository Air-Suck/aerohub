#!/usr/bin/env bash
# Host scene lifecycle; does not start PX4 or AS2.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/airsim/load_env.sh"
action=${1:-help}
if [[ "$action" == help || "$action" == --help || "$action" == -h ]]; then
  echo "Usage: tools/airsim/airsim.sh {check|start|status|stop} [manager options]"
  echo "Environment: AIRSIM_PROFILE, AIRSIM_BINARY, AIRSIM_RUN_DIR"
  exit 0
fi
shift
options=(--profile "${AIRSIM_PROFILE:-$root/deployment/airsim/config/airsim/nh.json}"
         --run-dir "${AIRSIM_RUN_DIR:-$root/log/airsim/airsim/scene}")
if [[ -n ${AIRSIM_BINARY:-} ]]; then
  options+=(--binary "$AIRSIM_BINARY")
fi
exec python3 "$root/tools/airsim/airsim/manager.py" "$action" "${options[@]}" "$@"
