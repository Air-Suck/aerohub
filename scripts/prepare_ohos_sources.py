"""Sync active forks into the already prepared isolated OHOS workspace."""
import hashlib
import json
import os
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
work_root = Path(os.environ.get('OHOS_WORK_ROOT', 'artifacts/ohos-as2')).expanduser()
if not work_root.is_absolute():
    work_root = root / work_root
workspace = work_root / 'ws/src'
if not (workspace / 'rmw_fastrtps_shared_cpp/package.xml').exists():
    raise SystemExit('Run deployment/oh-ros-build/prepare.sh first; see deployment/oh-ros-build/README.md')
for dependency in ['px4_msgs', 'vision_msgs', 'mocap4r2_msgs']:
    if not any((workspace / dependency).glob('**/package.xml')):
        raise SystemExit(f'Missing {dependency}; initialize a complete workspace with prepare.sh first')
paths = {name: root / 'as2/aerostack2' / name for name in
         ['as2_core', 'as2_msgs', 'as2_motion_reference_handlers',
          'as2_state_estimator', 'as2_motion_controller']}
for name in ['as2_behavior', 'as2_behaviors_motion']:
    paths[name] = root / 'as2/aerostack2/as2_behaviors' / name
paths['as2_platform_pixhawk'] = root / 'as2/as2_platform_pixhawk'
paths['uav_planner_interfaces'] = root / 'src/planner/interfaces'
paths['uav_follow_path_execution'] = root / 'src/behavior/follow_path'
# px4_msgs base sources were prepared with the isolated workspace; preserve them.
for name, source in paths.items():
    shutil.copytree(source, workspace / name, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('.git', '__pycache__', 'build', 'install', 'log'))
if not (workspace / 'as2_motion_controller/thirdparties/pid_controller/CMakeLists.txt').exists():
    raise SystemExit('Missing offline PID source; initialize a complete workspace with prepare.sh first')
for source in (root / 'deployment/onboard_computer/px4_msgs/overrides').glob('*.msg'):
    shutil.copyfile(source, workspace / 'px4_msgs/msg' / source.name)
record = {'firmware_commit': '61907962c9220437ec01a98fa26bfbf5cbb254b5', 'source_hashes': {}}
for name in [*paths, 'px4_msgs', 'vision_msgs', 'mocap4r2_msgs', 'rmw_fastrtps_shared_cpp']:
    record['source_hashes'][name] = {
        str(p.relative_to(workspace / name)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (workspace / name).rglob('*') if p.is_file() and '.git' not in p.parts}
(work_root / 'current-sources.json').write_text(json.dumps(record, indent=2) + '\n')
print('Current AS2 forks and PX4 1.18 definitions synchronized into OHOS workspace')
