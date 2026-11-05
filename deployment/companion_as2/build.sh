#!/usr/bin/env bash
# Build fixed PX4 interfaces once, then compile the editable AS2 forks.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
bash "$root/scripts/prepare_sources.sh"
args=(--network host)
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  if [[ -v $variable ]]; then
    args+=(--build-arg "$variable")
  fi
done
docker build "${args[@]}" -f "$root/deployment/companion_as2/px4_msgs/Dockerfile" \
  -t companion_as2_dependencies:humble-px4-1.18 "$root"
docker build "${args[@]}" -f "$root/deployment/companion_as2/Dockerfile" \
  -t companion_as2:airsim-px4 "$root"
