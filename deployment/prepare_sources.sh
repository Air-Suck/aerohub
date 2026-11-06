#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"
git submodule update --init --recursive
for name in aerostack2 as2_platform_pixhawk px4_msgs; do
  git -C "as2/$name" rev-parse HEAD > "as2/$name/.source-commit"
done
echo "Pinned submodules prepared. Build products belong in log/."
