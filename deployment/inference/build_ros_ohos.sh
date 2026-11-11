#!/usr/bin/env bash
# Use an isolated, prepared OHOS workspace; see deployment/oh-ros-build/README.md.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/robopi/load_env.sh"
: "${OHOS_WORK_ROOT:?Set OHOS_WORK_ROOT to an isolated prepared workspace}"
: "${OHOS_INSTALL_PREFIX:?Set OHOS_INSTALL_PREFIX to the final board overlay install path}"
work=$OHOS_WORK_ROOT
[[ "$work" == /* ]] || work="$root/$work"
python3 - "$root" "$work" <<'PY'
import hashlib
import json
from pathlib import Path
import shutil
import sys
root, work = map(Path, sys.argv[1:])
manifest = {}
for name, relative in [('uav_inference_interfaces', 'interfaces'), ('uav_inference', 'ros2_adapter')]:
    source = root / 'src/inference' / relative
    target = work / 'ws/src' / name
    if target.exists():
        raise FileExistsError('Choose a clean isolated workspace: ' + str(target))
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    manifest[name] = {str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in source.rglob('*') if path.is_file() and '__pycache__' not in path.parts}
(work / 'inference-inputs.json').write_text(json.dumps(manifest, indent=2) + '\n')
PY
exec bash "$root/deployment/oh-ros-build/run.sh" build uav_inference_interfaces uav_inference
