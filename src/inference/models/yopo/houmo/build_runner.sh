#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../../../../.." && pwd)
source "$repo/tools/robopi/load_env.sh"
sdk=${OHOS_NATIVE_SDK:?Set OHOS_NATIVE_SDK in .env to the OHOS native SDK}
tcim=${TCIM_RUNTIME_PATH:-/opt/houmo/tcim}
if [[ -z "$tcim" ]]; then
  echo 'Set TCIM_RUNTIME_PATH in .env to the OHOS AArch64 TCIM development package' >&2
  exit 1
fi
build="$here/artifacts/runner-build"
cmake --fresh -S "$here/runner" -B "$build" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$sdk/build/cmake/ohos.toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release -DTCIM_RUNTIME_PATH="$tcim"
cmake --build "$build" --parallel
