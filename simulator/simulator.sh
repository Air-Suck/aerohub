#!/usr/bin/env bash
# Backend-neutral simulator lifecycle entrypoint.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
action=${1:-help}
if [[ "$action" == help || "$action" == --help || "$action" == -h ]]; then
  echo "Usage: simulator/simulator.sh {check|start|attach|status|stop|display} [--config config.yaml]"
  echo "Environment: SIMULATOR_PROFILE, SIMULATOR_BINARY, SIMULATOR_RUN_DIR"
  exit 0
fi
shift
options=(--run-dir "${SIMULATOR_RUN_DIR:-$root/log/simulator/scene}")
if [[ "$action" == start || "$action" == attach || "$action" == check || -n ${SIMULATOR_PROFILE:-} ]]; then
  config="$root/config/simulator/default.yaml"
  options+=(--profile "${SIMULATOR_PROFILE:-$config}")
fi
if [[ -n ${SIMULATOR_BINARY:-} ]]; then
  options+=(--binary "$SIMULATOR_BINARY")
fi
exec python3 "$root/simulator/manager/manager.py" "$action" "${options[@]}" "$@"
