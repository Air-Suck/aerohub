#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

bash ./deployment/prepare_sources.sh

build_args=(--network host -f deployment/platform/Dockerfile)
if [[ "${NO_CACHE:-0}" == "1" ]]; then
  build_args+=(--no-cache)
fi
if [[ -n "${HTTP_PROXY:-}" ]]; then
  build_args+=(--build-arg "HTTP_PROXY=$HTTP_PROXY")
fi
if [[ -n "${HTTPS_PROXY:-}" ]]; then
  build_args+=(--build-arg "HTTPS_PROXY=$HTTPS_PROXY")
fi

docker build "${build_args[@]}" -t uav/aerostack2-px4:humble-v1.17 .
docker compose -f deployment/integration/compose.yaml build
