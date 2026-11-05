#!/usr/bin/env bash
# First-time project adaptation after the official OHOS environment setup.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
source "$root/scripts/load_env.sh"
exec python3 "$root/scripts/prepare_ohos_workspace.py" "$@"
