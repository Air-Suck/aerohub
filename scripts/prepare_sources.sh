#!/usr/bin/env bash
# Initialize active AS2 and SUPER forks without resetting local work.
set -euo pipefail
root_dir=$(cd "$(dirname "$0")/.." && pwd)
case "${1:-}" in
  '') ;;
  *) echo "Usage: $0" >&2; exit 2 ;;
esac
for name in aerostack2 as2_platform_pixhawk SUPER-sim; do
  target="$root_dir/thirdparty/$name"
  if [[ ! -e "$target/.git" ]]; then
    git -C "$root_dir" submodule update --init -- "thirdparty/$name"
  fi
  git -C "$target" rev-parse --show-toplevel
  git -C "$target" log -1 --format='%H %s'
done
