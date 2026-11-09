#!/usr/bin/env bash
# First-time project adaptation after the official OHOS environment setup.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
source "$root/tools/airsim/load_env.sh"
exec python3 "$root/tools/airsim/prepare_ohos_workspace.py" "$@"
