#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source "$root/tools/robopi/load_env.sh"
model=${1:?Usage: run.sh MODEL [COMMAND ...]}
shift
[[ "$model" =~ ^[a-z0-9][a-z0-9-]*$ ]] || { echo 'Invalid model name' >&2; exit 2; }
name="houmo-migrate-$model"
image=${HOUMO_MIGRATE_IMAGE:-harbor.houmo.ai/toolchain/release:Dadao-xh2-v1.5.0-ubuntu24.04-x86.64}
sdk=${OHOS_NATIVE_SDK:-}
tcim=${TCIM_RUNTIME_PATH:-}
sysdeps=${OHOS_SYSDEPS:-}
if [[ -z "$sdk" && -n "${OHOS_WORK_ROOT:-}" ]]; then
  work=$OHOS_WORK_ROOT
  [[ "$work" == /* ]] || work="$root/$work"
  sdk="$work/root/ohos-robot-toolchain/18/native"
fi
[[ -f "$sdk/build/cmake/ohos.toolchain.cmake" ]] || {
  echo 'Set OHOS_NATIVE_SDK in .env to an existing OHOS native SDK directory' >&2; exit 1;
}
docker image inspect "$image" >/dev/null 2>&1 || {
  echo "Pull the pinned toolchain first: docker pull $image" >&2; exit 1;
}
tcim_mount=()
tcim_env=()
if [[ -n "$tcim" ]]; then
  [[ -d "$tcim/include/tcim" && -d "$tcim/lib" ]] || {
    echo 'TCIM_RUNTIME_PATH must contain include/tcim and lib' >&2; exit 1;
  }
  tcim_mount=(-v "$tcim:/opt/houmo/tcim:ro")
  tcim_env=(-e TCIM_RUNTIME_PATH=/opt/houmo/tcim)
fi
sysdeps_mount=()
sysdeps_env=()
if [[ -n "$sysdeps" ]]; then
  [[ -f "$sysdeps/include/python3.12/Python.h" ]] || {
    echo 'OHOS_SYSDEPS must contain include/python3.12/Python.h' >&2; exit 1;
  }
  sysdeps_mount=(-v "$sysdeps:/target-out:ro")
  sysdeps_env=(-e OHOS_SYSDEPS=/target-out)
fi
if docker container inspect "$name" >/dev/null 2>&1; then
  actual=$(docker inspect --format '{{ index .Config.Labels "aerohub.workspace" }}' "$name")
  [[ "$actual" == "$root" ]] || { echo "$name belongs to another workspace; preserving it" >&2; exit 1; }
  actual_sdk=$(docker inspect --format '{{ index .Config.Labels "aerohub.ohos-sdk" }}' "$name")
  actual_tcim=$(docker inspect --format '{{ index .Config.Labels "aerohub.tcim" }}' "$name")
  actual_sysdeps=$(docker inspect --format '{{ index .Config.Labels "aerohub.sysdeps" }}' "$name")
  [[ "$actual_sdk" == "$sdk" && "$actual_tcim" == "$tcim" && "$actual_sysdeps" == "$sysdeps" ]] || {
    echo "$name uses different SDK/TCIM mounts; remove this workspace-owned container after checking its contents, then rerun with the intended .env" >&2; exit 1;
  }
  actual_image=$(docker inspect --format '{{.Image}}' "$name")
  [[ "$actual_image" == "$(docker image inspect --format '{{.Id}}' "$image")" ]] || {
    echo "$name uses another image; preserving it" >&2; exit 1;
  }
  [[ "$(docker inspect --format '{{.State.Running}}' "$name")" == true ]] || docker start "$name" >/dev/null
else
  gpu=()
  [[ ${HOUMO_MIGRATE_GPU:-0} == 0 ]] || gpu=(--gpus all)
  docker run -d --pull never --name "$name" --network none "${gpu[@]}" \
    --label "aerohub.workspace=$root" --label "aerohub.ohos-sdk=$sdk" \
    --label "aerohub.tcim=$tcim" --label "aerohub.sysdeps=$sysdeps" \
    --user "$(id -u):$(id -g)" \
    --tmpfs /tmp:rw,size=4g -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 \
    -e OHOS_NATIVE_SDK=/opt/ohos-sdk/native "${tcim_env[@]}" "${sysdeps_env[@]}" \
    -v "$root:/workspace" -v "$sdk:/opt/ohos-sdk/native:ro" \
    "${tcim_mount[@]}" "${sysdeps_mount[@]}" \
    -w /workspace --entrypoint /bin/bash "$image" -c 'exec tail -f /dev/null' >/dev/null
fi
if (( $# )); then
  exec docker exec -e USER=aerohub -e TORCHINDUCTOR_CACHE_DIR=/tmp/torchinductor "$name" "$@"
else
  exec docker exec -it -e USER=aerohub -e TORCHINDUCTOR_CACHE_DIR=/tmp/torchinductor "$name" bash
fi
