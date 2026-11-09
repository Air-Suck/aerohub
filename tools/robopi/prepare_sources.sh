#!/usr/bin/env bash
# Initialize pinned AS2 and SUPER sources without resetting local work.
set -euo pipefail
root_dir=$(cd "$(dirname "$0")/../.." && pwd)
case "${1:-}" in
  '') ;;
  *) echo "Usage: $0" >&2; exit 2 ;;
esac
for name in aerostack2 as2_platform_pixhawk SUPER-sim; do
  relative="as2/$name"
  [[ "$name" != as2_platform_pixhawk ]] || relative="as2/as2_platform_pixhawk"
  [[ "$name" != SUPER-sim ]] || relative="src/planner/super/upstream"
  target="$root_dir/$relative"
  if [[ ! -e "$target/.git" ]]; then
    git -C "$root_dir" submodule update --init -- "$relative"
  fi
  git -C "$target" rev-parse --show-toplevel
  git -C "$target" log -1 --format='%H %s'
done
