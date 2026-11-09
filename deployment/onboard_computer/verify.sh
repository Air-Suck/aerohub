#!/usr/bin/env bash
# Real AirSim/PX4/AS2 interaction; only dedicated verification resources.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
export VERIFY_AS2=1
export VERIFY_OUTPUT="${VERIFY_OUTPUT:-$root/log/airsim/as2-interaction-verification}"
bash "$root/deployment/flight_board/verify.sh"
