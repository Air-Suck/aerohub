#!/usr/bin/env bash
# First-time project adaptation after the official OHOS environment setup.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
source "$root/tools/robopi/load_env.sh"
exec python3 "$root/tools/robopi/prepare_ohos_workspace.py" "$@"
