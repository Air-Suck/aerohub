#!/usr/bin/env bash
# Build a standalone UE plugin; install its package into a selected project.
set -euo pipefail
if [[ $# != 3 ]]; then
  echo "Usage: $0 UE_ROOT PROJECT.uproject PACKAGE_DIR" >&2
  exit 2
fi
ue_root=$1
project=$2
package=$3
script_dir=$(cd "$(dirname "$0")" && pwd)
"$ue_root/Engine/Build/BatchFiles/RunUAT.sh" BuildPlugin \
  "-Plugin=$script_dir/unreal/AerohubDisplay/AerohubDisplay.uplugin" \
  "-Package=$package" -TargetPlatforms=Linux
python3 - "$package" "$project" <<'PY'
from pathlib import Path
import shutil
import sys
package, project = map(Path, sys.argv[1:])
target = project.parent / 'Plugins/AerohubDisplay'
# Only update this owned plugin, preserving all other project assets.
shutil.copytree(package, target, dirs_exist_ok=True,
                ignore=shutil.ignore_patterns('HostProject'))
print(f'Installed display plugin: {target}')
PY
