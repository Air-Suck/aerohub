#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

clone_pinned() {
  local name="$1"
  local url="$2"
  local commit="$3"
  local target="$root_dir/$name"

  if [[ -d "$target/.git" ]]; then
    local actual
    actual="$(git -C "$target" rev-parse HEAD)"
    if [[ "$actual" != "$commit" ]]; then
      echo "$name is at $actual, expected $commit" >&2
      return 2
    fi
    printf '%s\n' "$commit" > "$target/.source-commit"
    echo "$name already prepared at $commit"
    return
  fi

  if [[ -e "$target" ]]; then
    echo "$target exists but is not a Git checkout" >&2
    return 2
  fi

  git init "$target"
  git -C "$target" remote add origin "$url"
  git -C "$target" fetch --depth 1 origin "$commit"
  git -C "$target" checkout --detach FETCH_HEAD
  test "$(git -C "$target" rev-parse HEAD)" = "$commit"
  printf '%s\n' "$commit" > "$target/.source-commit"
  echo "$name prepared at $commit"
}

clone_pinned \
  aerostack2 \
  https://github.com/aerostack2/aerostack2.git \
  19c397309c6a41b279e6b551818ec12605cfcecc
clone_pinned \
  as2_platform_pixhawk \
  https://github.com/aerostack2/as2_platform_pixhawk.git \
  fcba062219935acef69dfa009c8be376c707ee8a
clone_pinned \
  px4_msgs \
  https://github.com/PX4/px4_msgs.git \
  86d8239e962f6939e05c3737784f60c02fa884db
