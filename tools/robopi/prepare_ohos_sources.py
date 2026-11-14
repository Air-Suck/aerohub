"""Sync locked AS2/Pixhawk fork sources into the already prepared isolated OHOS workspace."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import tarfile
import io

root = Path(__file__).resolve().parents[2]
work_root = Path(os.environ.get('OHOS_WORK_ROOT', 'log/airsim/ohos-as2')).expanduser()
if not work_root.is_absolute():
    work_root = root / work_root
workspace = work_root / 'ws/src'
if not (workspace / 'rmw_fastrtps_shared_cpp/package.xml').exists():
    raise SystemExit('Run deployment/oh-ros-build/prepare.sh first; see deployment/oh-ros-build/README.md')
for dependency in ['px4_msgs', 'vision_msgs', 'mocap4r2_msgs']:
    if not any((workspace / dependency).glob('**/package.xml')):
        raise SystemExit(f'Missing {dependency}; initialize a complete workspace with prepare.sh first')
# Export isolated fork copies; never modify the source submodules.
temporary = tempfile.TemporaryDirectory(prefix='as2-source-', dir=work_root)
as2_copy = Path(temporary.name) / 'aerostack2'
def export_locked(relative, destination):
    commit = subprocess.check_output(['git', '-C', str(root), 'ls-tree', 'HEAD', relative], text=True).split()[2]
    data = subprocess.check_output(['git', '-C', str(root/relative), 'archive', commit])
    destination.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        archive.extractall(destination)
    return commit

as2_commit = export_locked('as2/aerostack2', as2_copy)
pixhawk_copy = Path(temporary.name)/'pixhawk'
pixhawk_commit = export_locked('as2/as2_platform_pixhawk', pixhawk_copy)
paths = {name: as2_copy / name for name in
         ['as2_core', 'as2_msgs', 'as2_motion_reference_handlers',
          'as2_state_estimator', 'as2_motion_controller']}
for name in ['as2_behavior', 'as2_behaviors_motion']:
    paths[name] = as2_copy / 'as2_behaviors' / name
paths['as2_platform_pixhawk'] = pixhawk_copy
paths['uav_planner_interfaces'] = root / 'src/tool/navigation/interfaces'
paths['uav_follow_path_execution'] = root / 'src/behavior/follow_path'
# px4_msgs base sources were prepared with the isolated workspace; preserve them.
for name, source in paths.items():
    shutil.copytree(source, workspace / name, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('.git', '__pycache__', 'build', 'install', 'log'))
if not (workspace / 'as2_motion_controller/thirdparties/pid_controller/CMakeLists.txt').exists():
    raise SystemExit('Missing offline PID source; initialize a complete workspace with prepare.sh first')
for source in (root / 'deployment/onboard_computer/px4_msgs/overrides').glob('*.msg'):
    shutil.copyfile(source, workspace / 'px4_msgs/msg' / source.name)
record = {'locked_sources': {'as2/aerostack2': as2_commit, 'as2/as2_platform_pixhawk': pixhawk_commit}, 'firmware_commit': '61907962c9220437ec01a98fa26bfbf5cbb254b5', 'source_hashes': {}}
for name in [*paths, 'px4_msgs', 'vision_msgs', 'mocap4r2_msgs', 'rmw_fastrtps_shared_cpp']:
    record['source_hashes'][name] = {
        str(p.relative_to(workspace / name)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (workspace / name).rglob('*') if p.is_file() and '.git' not in p.parts}
(work_root / 'current-sources.json').write_text(json.dumps(record, indent=2) + '\n')
print('Locked AS2/Pixhawk fork sources and PX4 1.18 definitions synchronized into OHOS workspace')
temporary.cleanup()
