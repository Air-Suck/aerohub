#!/usr/bin/env bash
# Build a local development tag without replacing the shared flight_board image.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
base=${FLIGHT_BOARD_BASE_IMAGE:-flight_board:v0.1.0}
tag=${FLIGHT_BOARD_DEV_IMAGE:-flight_board:sim-manager-dev}
exec docker build --pull=false --build-arg "BASE_IMAGE=$base" \
  -f "$root/deployment/flight_board/Dockerfile.sim-manager" -t "$tag" "$root"
