#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/scripts/load_env.sh"
if [[ ${1:---help} == --help || ${1:-} == -h ]]; then
  echo 'Usage: deployment/oh-ros-build/run.sh {check|build [package ...]|shell}'
  exit 0
fi
: "${OHOS_SYSDEPS:?Set OHOS_SYSDEPS in .env to the prepared target deps/out directory}"
work=${OHOS_WORK_ROOT:-$root/artifacts/ohos-as2}
[[ "$work" == /* ]] || work="$root/$work"
deps=$OHOS_SYSDEPS
[[ "$deps" == /* ]] || deps="$root/$deps"
[[ -d "$work/root" && -d "$work/ws" && -d "$deps" ]] || {
  echo 'Prepared OHOS workspace or target dependency directory is missing; see deployment/oh-ros-build/README.md' >&2
  exit 1
}
terminal=()
if [[ ${1:-} == shell ]]; then
  terminal=(-i)
  [[ ! -t 0 ]] || terminal=(-it)
fi
image=${OHOS_BUILDER_IMAGE:-voxelsky/ohos-ros-humble-builder@sha256:3312006a50a2a7f4a26e1c92afcfbb0fd2296989b7e6d22e71b1f77caad0d25f}
if ! docker image inspect "$image" >/dev/null 2>&1; then
  echo "Official builder is missing: $image; prepare it per the official guide before continuing" >&2
  exit 1
fi
exec docker run --rm --pull never "${terminal[@]}" --name aerohub-oh-ros-build --network none \
  -v "$work:/work" -v "$deps:/target-out:ro" \
  -v "$root/deployment/oh-ros-build:/opt/aerohub/ohos:ro" \
  -e WS_ROOT=/work/root -e OHOS_SDK=/work/root/ohos-robot-toolchain/18 \
  -e OHOS_WORKSPACE=/work/ws -e OHOS_SYSDEPS=/target-out \
  -e OHOS_INSTALL_PREFIX=/data/local/tmp/aerohub-as2/install \
  -e OHOS_HOST_PYTHON=/root/conda/envs/ros_env/bin/python3 \
  -e OHOS_PYTHON_SOABI=cpython-312-aarch64-linux-ohos -e OHOS_CPU=aarch64 \
  --workdir /work --entrypoint /bin/bash "$image" \
  /opt/aerohub/ohos/build_workspace.sh "$@"
