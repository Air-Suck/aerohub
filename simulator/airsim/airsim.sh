#!/usr/bin/env bash
# Host scene lifecycle; does not start PX4 or AS2.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/robopi/load_env.sh"
action=${1:-help}
if [[ "$action" == help || "$action" == --help || "$action" == -h ]]; then
  echo "Usage: simulator/airsim/airsim.sh {check|start|status|stop} [manager options]"
  echo "Environment: AIRSIM_PROFILE, AIRSIM_BINARY, AIRSIM_RUN_DIR"
  exit 0
fi
shift
options=(--profile "${AIRSIM_PROFILE:-$root/simulator/airsim/config/nh.json}"
         --run-dir "${AIRSIM_RUN_DIR:-$root/log/airsim/airsim/scene}")
if [[ -n ${AIRSIM_BINARY:-} ]]; then
  options+=(--binary "$AIRSIM_BINARY")
fi
exec python3 "$root/simulator/airsim/manager/manager.py" "$action" "${options[@]}" "$@"
