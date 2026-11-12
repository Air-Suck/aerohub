#!/usr/bin/env bash
# Backend-neutral simulator lifecycle entrypoint.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
action=${1:-help}
if [[ "$action" == help || "$action" == --help || "$action" == -h ]]; then
  echo "Usage: simulator/simulator.sh {check|start|status|stop} [manager options]"
  echo "Environment: SIMULATOR_PROFILE, SIMULATOR_BINARY, SIMULATOR_RUN_DIR"
  exit 0
fi
shift
options=(--run-dir "${SIMULATOR_RUN_DIR:-$root/log/simulator/scene}")
if [[ "$action" == start || "$action" == check || -n ${SIMULATOR_PROFILE:-} ]]; then
  options+=(--profile "${SIMULATOR_PROFILE:-$root/simulator/airsim/config/nh.json}")
fi
if [[ -n ${SIMULATOR_BINARY:-} ]]; then
  options+=(--binary "$SIMULATOR_BINARY")
fi
exec python3 "$root/simulator/manager/manager.py" "$action" "${options[@]}" "$@"
