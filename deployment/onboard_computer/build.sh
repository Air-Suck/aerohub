#!/usr/bin/env bash
# Build fixed PX4 interfaces once, then compile the editable AS2 forks.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
bash "$root/tools/robopi/prepare_sources.sh"
args=(--network host)
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  if [[ -v $variable ]]; then
    args+=(--build-arg "$variable")
  fi
done
image=${ONBOARD_COMPUTER_IMAGE:-${MIGRATION_ONBOARD_IMAGE:-onboard_computer:v0.1.1}}
version=${ONBOARD_IMAGE_VERSION:-v0.1.1}
deps=${ONBOARD_DEPS_IMAGE:-onboard_computer_dependencies:humble-px4-1.18}
if [[ ${BUILD_PX4_DEPS:-1} == 1 ]]; then
  docker build "${args[@]}" -f "$root/deployment/onboard_computer/px4_msgs/Dockerfile" \
    -t "$deps" "$root"
else
  # Caller must first verify source commit and overridden message hashes.
  docker image inspect "$deps" >/dev/null
fi
docker build "${args[@]}" --build-arg BASE_IMAGE="$deps" \
  --build-arg IMAGE_VERSION="$version" --build-arg SOURCE_REVISION="$(git -C "$root" rev-parse HEAD)" \
  --label "io.aerohub.as2.source=$(git -C "$root/as2/aerostack2" remote get-url origin)" \
  --label "io.aerohub.as2.revision=$(git -C "$root/as2/aerostack2" rev-parse HEAD)" \
  --label "io.aerohub.as2.dirty=$(test -z "$(git -C "$root/as2/aerostack2" status --porcelain)" && echo false || echo true)" \
  --label "io.aerohub.platform.source=$(git -C "$root/as2/as2_platform_pixhawk" remote get-url origin)" \
  --label "io.aerohub.platform.revision=$(git -C "$root/as2/as2_platform_pixhawk" rev-parse HEAD)" \
  --label "io.aerohub.platform.dirty=$(test -z "$(git -C "$root/as2/as2_platform_pixhawk" status --porcelain)" && echo false || echo true)" \
  --label "io.aerohub.source.dirty=$(test -z "$(git -C "$root" status --porcelain)" && echo false || echo true)" \
  --label "io.aerohub.px4-msgs.overrides=$(sha256sum "$root/deployment/onboard_computer/px4_msgs/overrides/SHA256SUMS" | cut -d ' ' -f 1)" \
  -f "$root/deployment/onboard_computer/Dockerfile" -t "$image" "$root"
