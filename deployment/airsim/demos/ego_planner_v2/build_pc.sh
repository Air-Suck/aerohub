#!/usr/bin/env bash
# Run in the pinned onboard tool image, repository /input:ro, external work /work.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/local_setup.bash
source /root/aerostack2_ws/install/local_setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS='-j2 -l2'
if [[ -e /work/component && $(cat /work/component) != ego-planner-v2 ]]; then
  echo 'External workspace belongs to another component' >&2; exit 1
fi
printf '%s\n' ego-planner-v2 > /work/component
colcon --log-base /work/log/native build \
  --base-paths /input/src/planner/ego_planner_v2/ros2_adapter/native \
  --build-base /work/build/humble --install-base /work/install/humble \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /work/install/humble/local_setup.bash
colcon --log-base /work/log/execution build \
  --base-paths /input/src/planner/interfaces /input/src/behavior/follow_path \
  --build-base /work/ground_overlay/build --install-base /work/ground_overlay/install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /work/ground_overlay/install/local_setup.bash
colcon --log-base /work/log/adapter build \
  --base-paths /input/src/planner/framework /input/src/planner/ego_planner_v2/ros2_adapter/bridge \
  --build-base /work/ego_runtime/build --install-base /work/ego_runtime/install \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF
source /work/ego_runtime/install/local_setup.bash
python3 - <<'PY'
import hashlib,json
from pathlib import Path
root=Path('/input')
paths=['src/planner/ego_planner_v2/ros2_adapter/native','src/planner/ego_planner_v2/ros2_adapter/bridge',
       'src/planner/interfaces','src/planner/framework','src/behavior/follow_path']
files={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
       for rel in paths for p in (root/rel).rglob('*') if p.is_file() and '__pycache__' not in p.parts}
Path('/work/source-manifest.json').write_text(json.dumps(files,indent=2))
from ament_index_python.packages import get_package_prefix
from traj_utils.msg import PolyTraj
from uav_planner_interfaces.action import FollowTimedPath
from uav_ego_planner_adapter import node,conversion,scope
from rclpy.serialization import serialize_message,deserialize_message
assert get_package_prefix('ego_planner').startswith('/work/install/humble/')
assert node.__file__.startswith('/work/ego_runtime/')
assert isinstance(deserialize_message(serialize_message(FollowTimedPath.Goal()),FollowTimedPath.Goal),FollowTimedPath.Goal)
print('Current-source EGO native/Adapter/messages installed and loaded')
PY
