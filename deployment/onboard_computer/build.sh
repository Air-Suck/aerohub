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
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}
deps=${ONBOARD_DEPS_IMAGE:-onboard_computer_dependencies:humble-px4-1.18}
if [[ ${BUILD_PX4_DEPS:-1} == 1 ]]; then
  docker build "${args[@]}" -f "$root/deployment/onboard_computer/px4_msgs/Dockerfile" \
    -t "$deps" "$root"
else
  # Caller must first verify source commit and overridden message hashes.
  docker image inspect "$deps" >/dev/null
fi
docker build "${args[@]}" --build-arg BASE_IMAGE="$deps" \
  --build-arg IMAGE_VERSION=v0.1.0 --build-arg SOURCE_REVISION="$(git -C "$root" rev-parse HEAD)" \
  -f "$root/deployment/onboard_computer/Dockerfile" -t "$image" "$root"
