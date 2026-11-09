#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/robopi/load_env.sh"
component=${1:-all}
case "$component" in as2|catora|swarm-formation|all) ;; *) echo 'Usage: verify_lib.sh [as2|catora|swarm-formation|all]' >&2; exit 2 ;; esac
mkdir -p "$root/log/airsim/robopi-runtime"
exec 9>"$root/log/airsim/robopi-runtime/cli.lock"
flock -n 9 || { echo 'Another RoboPi operation is running' >&2; exit 1; }
exec python3 "$root/deployment/robopi/run_verification.py" "$component"
