#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/load_env.sh"
mkdir -p "$root/artifacts/robopi-runtime"
exec 9>"$root/artifacts/robopi-runtime/cli.lock"
flock -n 9 || { echo 'Another RoboPi operation is running' >&2; exit 1; }
if docker container inspect aerohub-oh-ros-build >/dev/null 2>&1; then
  echo 'Shared build workspace is occupied; retry after the build finishes' >&2
  exit 1
fi
exec python3 "$root/deployment/robopi/deploy_lib.py" "$@"
