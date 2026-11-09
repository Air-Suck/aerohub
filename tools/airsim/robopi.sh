#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/airsim/load_env.sh"
mkdir -p "$root/log/airsim/robopi-runtime"
exec 9>"$root/log/airsim/robopi-runtime/cli.lock"
flock -n 9 || { echo "Another RoboPi operation is running" >&2; exit 1; }
exec python3 "$root/tools/airsim/robopi.py" "$@"
