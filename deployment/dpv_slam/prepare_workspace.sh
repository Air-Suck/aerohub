#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${1:-$root/log/workspaces/dpv_slam}
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}
source_rel=src/state_estimator/dpv_slam/upstream
expected_source=0ac95b656d1fda91c271d2a106460d19ad966fc7

docker image inspect "$image" >/dev/null
dpvo_source="$root/$source_rel"
if [[ ! -e $dpvo_source/.git ]]; then
  printf 'DPVO submodule is not initialized: %s\n' "$source_rel" >&2
  printf 'Run: git submodule update --init -- %s\n' "$source_rel" >&2
  exit 2
fi
if [[ $(git -C "$dpvo_source" rev-parse --show-toplevel) != "$dpvo_source" ]]; then
  printf 'DPVO upstream path is not an independent Git worktree: %s\n' "$dpvo_source" >&2
  exit 2
fi
gitlink=$(git -C "$root" ls-files -s "$source_rel" | awk '$1 == "160000" {print $2}')
dpvo_commit=$(git -C "$dpvo_source" rev-parse HEAD)
if [[ $gitlink != "$expected_source" || $dpvo_commit != "$expected_source" ]]; then
  printf 'DPVO source mismatch: expected=%s gitlink=%s checkout=%s\n' \
    "$expected_source" "${gitlink:-missing}" "$dpvo_commit" >&2
  exit 2
fi
if [[ -n $(git -C "$dpvo_source" status --short --untracked-files=no) ]]; then
  printf 'DPVO upstream submodule has tracked modifications\n' >&2
  git -C "$dpvo_source" status --short --untracked-files=no >&2
  exit 2
fi
mkdir -p "$workspace"
workspace=$(cd "$workspace" && pwd)
wheelhouse="$workspace/wheelhouse"
wheel_manifest="$root/deployment/dpv_slam/python-wheels.sha256"
mkdir -p "$wheelhouse"
docker image inspect "$image" > "$workspace/shared-image.json"

download_in_ranges() {
  local checksum=$1 url=$2 output=$3
  local partial="$output.partial"
  local chunk="$output.chunk"
  local total start end expected actual
  total=$(curl --fail --silent --show-error --location --head \
    --retry 10 --retry-all-errors "$url" \
    | tr -d '\r' | awk 'tolower($1) == "content-length:" {value=$2} END {print value}')
  [[ $total =~ ^[0-9]+$ ]] && ((total > 0))
  if [[ -f $output ]] && ! echo "$checksum  $output" | sha256sum --check --status; then
    mv "$output" "$partial"
  fi
  start=0
  if [[ -f $partial ]]; then
    start=$(stat -c %s "$partial")
  fi
  ((start <= total))
  while ((start < total)); do
    end=$((start + 8 * 1024 * 1024 - 1))
    if ((end >= total)); then
      end=$((total - 1))
    fi
    expected=$((end - start + 1))
    curl --fail --silent --show-error --location \
      --retry 20 --retry-all-errors --speed-limit 1024 --speed-time 180 \
      --range "$start-$end" --output "$chunk" "$url"
    actual=$(stat -c %s "$chunk")
    if ((actual != expected)); then
      printf 'range %s-%s returned %s bytes, expected %s\n' \
        "$start" "$end" "$actual" "$expected" >&2
      exit 1
    fi
    cat "$chunk" >> "$partial"
    start=$((end + 1))
    printf 'downloaded %s/%s bytes: %s\n' "$start" "$total" "$output"
  done
  mv "$partial" "$output"
  echo "$checksum  $output" | sha256sum --check
  rm -f "$chunk"
}

while read -r checksum filename url; do
  target="$wheelhouse/$filename"
  if ! echo "$checksum  $target" | sha256sum --check --status; then
    download_in_ranges "$checksum" "$url" "$target"
  fi
done < "$wheel_manifest"
find "$wheelhouse" -maxdepth 1 -type f -name '*.chunk' -delete

proxy_args=()
for variable in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  if [[ -n ${!variable:-} ]]; then
    proxy_args+=(--env "$variable=${!variable}")
  fi
done

prepare_container="aerohub-dpv-slam-prepare-$$"
cleanup_prepare_container() {
  docker rm --force "$prepare_container" >/dev/null 2>&1 || true
}
trap cleanup_prepare_container EXIT INT TERM

docker run --rm --gpus all --network host --init \
  --name "$prepare_container" \
  --label aerohub.task=dpv-slam-workspace-prepare \
  --label "aerohub.workspace=$workspace" \
  --entrypoint bash \
  "${proxy_args[@]}" \
  --env HOST_UID="$(id -u)" \
  --env HOST_GID="$(id -g)" \
  --env DPVO_SOURCE=/workspace/aerohub/src/state_estimator/dpv_slam/upstream \
  --env DPVO_COMMIT="$dpvo_commit" \
  --volume "$root:/workspace/aerohub:ro" \
  --volume "$workspace:/workspace/dpv" \
  "$image" \
  /workspace/aerohub/deployment/dpv_slam/build_workspace.sh
trap - EXIT INT TERM

{
  printf 'repository=%s\n' "$(git -C "$root" rev-parse HEAD)"
  printf 'upstream_path=%s\n' "$source_rel"
  printf 'upstream=%s\n' "$dpvo_commit"
  printf 'upstream_url=%s\n' "$(git -C "$dpvo_source" remote get-url origin)"
  printf 'upstream_clean=true\n'
  printf 'shared_image=%s\n' "$(docker image inspect "$image" --format '{{.Id}}')"
  printf 'python_wheels_manifest=%s\n' "$(sha256sum "$wheel_manifest" | cut -d' ' -f1)"
  printf 'cuda_packages_manifest=%s\n' \
    "$(sha256sum "$root/deployment/dpv_slam/cuda-build-packages.sha256" | cut -d' ' -f1)"
  printf 'model=%s\n' \
    "$(sha256sum "$root/deployment/dpv_slam/models/dpvo.pth" | cut -d' ' -f1)"
  printf 'workspace=%s\n' "$workspace"
  git -C "$root" status --short
} > "$workspace/source-state.txt"

printf 'DPV_SLAM_WORKSPACE=%q\n' "$workspace"
