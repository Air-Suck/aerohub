#!/usr/bin/env bash
# Build the selected native algorithm and its application interfaces for RoboPi.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/robopi/load_env.sh"
component=${1:-all}
case "$component" in catora|swarm-formation|super|ego-planner-v2|all) ;; *) echo 'Usage: build_lib.sh [catora|swarm-formation|super|ego-planner-v2|all]' >&2; exit 2 ;; esac
mkdir -p "$root/log/airsim/robopi-runtime"
exec 9>"$root/log/airsim/robopi-runtime/cli.lock"
flock -n 9 || { echo 'Another RoboPi operation is running' >&2; exit 1; }
docker info >/dev/null 2>&1 || { echo 'Docker unavailable; cannot check shared builder' >&2; exit 1; }
if docker container inspect aerohub-oh-ros-build >/dev/null 2>&1; then
  echo 'Shared builder is occupied; leave its inputs unchanged' >&2; exit 1
fi
work=${OHOS_WORK_ROOT:-log/airsim/ohos-as2}
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
 packages.update(mrs_formation_reshaping='src/planner/cat_ora/ros2_adapter/native',uav_cat_ora_adapter='src/planner/cat_ora/ros2_adapter/bridge')
if component in ('all','swarm-formation'):
 for directory in (root/'src/planner/swarm_formation/ros2_adapter/native').iterdir():
  if (directory/'package.xml').exists():
   import xml.etree.ElementTree as ET
   packages[ET.parse(directory/'package.xml').getroot().findtext('name')]=str(directory.relative_to(root))
 packages['uav_swarm_formation_adapter']='src/planner/swarm_formation/ros2_adapter/bridge'
 packages['as2_navigation_inputs']='src/perception/navigation_inputs'
if component == 'super':
 packages.pop('uav_as2_setpoint_adapter')
 packages.update(super_planner='src/planner/super/ros2_adapter/native/super_planner',rog_map='src/planner/super/ros2_adapter/native/rog_map',mars_quadrotor_msgs='src/planner/super/ros2_adapter/native/mars_uav_sim/mars_quadrotor_msgs',uav_super_adapter='src/planner/super/ros2_adapter/bridge',as2_navigation_inputs='src/perception/navigation_inputs')
if component == 'ego-planner-v2':
 if not any('ego' in part.lower() for part in work.parts) or (work/'ws/src/uav_swarm_formation_adapter').exists():
  raise RuntimeError('EGO requires a dedicated ego-named OHOS_WORK_ROOT; never share Swarm sources/build/install')
 packages.pop('uav_as2_setpoint_adapter')
 for directory in (root/'src/planner/ego_planner_v2/ros2_adapter/native').iterdir():
  if (directory/'package.xml').exists():
   import xml.etree.ElementTree as ET
   name=ET.parse(directory/'package.xml').getroot().findtext('name')
   packages[name]=str(directory.relative_to(root))
 packages.update(uav_ego_planner_adapter='src/planner/ego_planner_v2/ros2_adapter/bridge',as2_navigation_inputs='src/perception/navigation_inputs',uav_follow_path_execution='src/behavior/follow_path')
 marker=work/'algorithm-owner.json'
 if not marker.exists() and (work/'ws/src/ego_planner').exists():
  raise RuntimeError('Existing ego_planner without EGO ownership marker; use a new prepared EGO workspace')
 if marker.exists() and json.loads(marker.read_text())['component'] != component:
  raise RuntimeError('OHOS workspace belongs to another algorithm')
 marker.write_text(json.dumps({'component':component}))
elif (work/'algorithm-owner.json').exists() and json.loads((work/'algorithm-owner.json').read_text())['component']=='ego-planner-v2':
 raise RuntimeError('Do not overwrite the isolated EGO workspace with another algorithm')
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
packages=(uav_planner_framework)
case "$component" in
 catora) packages+=(uav_as2_setpoint_adapter mrs_formation_reshaping uav_cat_ora_adapter) ;;
 swarm-formation) packages+=(uav_as2_setpoint_adapter ego_planner uav_swarm_formation_adapter as2_navigation_inputs) ;;
 all) packages+=(uav_as2_setpoint_adapter mrs_formation_reshaping uav_cat_ora_adapter ego_planner uav_swarm_formation_adapter as2_navigation_inputs) ;;
 super) packages+=(super_planner uav_super_adapter as2_navigation_inputs) ;;
 ego-planner-v2) packages+=(ego_planner uav_ego_planner_adapter as2_navigation_inputs uav_follow_path_execution) ;;
esac
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
