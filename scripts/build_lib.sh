#!/usr/bin/env bash
# Build the selected native algorithm and its application interfaces for RoboPi.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/load_env.sh"
component=${1:-all}
case "$component" in catora|swarm-formation|all) ;; *) echo 'Usage: build_lib.sh [catora|swarm-formation|all]' >&2; exit 2 ;; esac
mkdir -p "$root/artifacts/robopi-runtime"
exec 9>"$root/artifacts/robopi-runtime/cli.lock"
flock -n 9 || { echo 'Another RoboPi operation is running' >&2; exit 1; }
if docker container inspect aerohub-oh-ros-build >/dev/null 2>&1; then
  echo 'Shared builder is occupied; leave its inputs unchanged' >&2; exit 1
fi
work=${OHOS_WORK_ROOT:-artifacts/ohos-as2}
[[ "$work" == /* ]] || work="$root/$work"
[[ -d "$work/root" && -d "$work/ws/src" ]] || { echo 'Prepare the OHOS workspace first' >&2; exit 1; }
python3 - "$root" "$work" "$component" <<'PY'
import hashlib, json, shutil, sys, os, subprocess
from pathlib import Path
root, work = map(Path, sys.argv[1:3])
component=sys.argv[3]
packages={'uav_planner_interfaces':'src/planner/interfaces', 'uav_planner_framework':'src/planner/framework',
          'uav_as2_setpoint_adapter':'src/behavior/legacy_setpoint'}
if component in ('all','catora'):
 packages.update(mrs_formation_reshaping='src/planner/cat_ora/humble',uav_cat_ora_adapter='src/planner/cat_ora/ros2_adapter')
if component in ('all','swarm-formation'):
 for directory in (root/'src/planner/swarm_formation/humble').iterdir():
  if (directory/'package.xml').exists():
   import xml.etree.ElementTree as ET
   packages[ET.parse(directory/'package.xml').getroot().findtext('name')]=str(directory.relative_to(root))
 packages['uav_swarm_formation_adapter']='src/planner/swarm_formation/ros2_adapter'
 packages['as2_navigation_inputs']='runtime/navigation_inputs'
manifest={}
stale_paths=[]
for name,relative in packages.items():
 dest=work/'ws/src'/name
 if dest.is_symlink(): raise RuntimeError('Refuse symlink workspace input: '+str(dest))
 if dest.exists(): shutil.rmtree(dest)
 shutil.copytree(root/relative,dest,ignore=shutil.ignore_patterns('__pycache__','build','install','log','.git'))
 # The builder rewrites installed text timestamps. setuptools can then skip newer content
 # carrying older source mtimes; compare bytes and refresh only the affected Python package.
 if (dest/'setup.py').exists():
  module=dest/name
  installed=work/'ws/install'/name/'lib/python3.12/site-packages'/name
  stale=any(not (installed/p.relative_to(module)).is_file() or
            (installed/p.relative_to(module)).read_bytes()!=p.read_bytes()
            for p in module.rglob('*.py'))
  if stale:
   for directory in (work/'ws/build'/name,work/'ws/install'/name):
    if directory.exists(): stale_paths.append("/work/"+str(directory.relative_to(work)))
 manifest[name]={str(p.relative_to(dest)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dest.rglob('*')) if p.is_file()}
(work/('lib-'+component+'-inputs.json')).write_text(json.dumps(manifest,indent=2)+'\n')
if stale_paths:
 image=os.environ.get('OHOS_BUILDER_IMAGE','voxelsky/ohos-ros-humble-builder@sha256:3312006a50a2a7f4a26e1c92afcfbb0fd2296989b7e6d22e71b1f77caad0d25f')
 subprocess.run(['docker','run','--rm','--pull','never','--network','none','-v',str(work)+':/work',
                 '--entrypoint','/bin/rm',image,'-rf',*stale_paths],check=True)

PY
packages=(uav_planner_framework uav_as2_setpoint_adapter)
[[ "$component" == swarm-formation ]] || packages+=(mrs_formation_reshaping uav_cat_ora_adapter)
[[ "$component" == catora ]] || packages+=(ego_planner uav_swarm_formation_adapter as2_navigation_inputs)
bash "$root/deployment/oh-ros-build/run.sh" build "${packages[@]}"
python3 - "$work" "$component" <<'PYVERIFY'
import json,sys
from pathlib import Path
work=Path(sys.argv[1])
for name in json.loads((work/('lib-'+sys.argv[2]+'-inputs.json')).read_text()):
 source=work/'ws/src'/name
 if not (source/'setup.py').exists():continue
 for file in (source/name).rglob('*.py'):
  installed=work/'ws/install'/name/'lib/python3.12/site-packages'/file.relative_to(source)
  if not installed.exists() or file.read_bytes()!=installed.read_bytes():
   raise RuntimeError('Installed Python differs from source: '+str(installed))
print('Python installation content matches selected sources')
PYVERIFY
