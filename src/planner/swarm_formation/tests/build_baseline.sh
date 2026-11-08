#!/usr/bin/env bash
# Run inside the Noetic dependency image. Source files are copied before any build-only adjustment.
set -eo pipefail
source /opt/ros/noetic/setup.bash
repo=${1:-/work}
workspace=${2:-/results/noetic}
mkdir -p "$workspace/src"
for package in plan_env path_searching traj_opt traj_utils swarm_graph plan_manage; do
  cp -r "$repo/thirdparty/swarm_formation_ros1/$package" "$workspace/src/"
done
python3 - "$workspace" "$repo" <<'PY'
import sys
from pathlib import Path
workspace, repo = map(Path, sys.argv[1:])
p = workspace / 'src/plan_manage/CMakeLists.txt'
s = p.read_text().replace('  quadrotor_msgs\n', '')
# Current adapter samples PolyTraj directly; the PositionCommand controller is outside this migration.
p.write_text(s[:s.index('add_executable(traj_server')])
p = workspace / 'src/traj_opt/CMakeLists.txt'
s = p.read_text()
s += '\nadd_executable(optimizer_benchmark "' + str(repo / 'src/planner/swarm_formation/humble/traj_opt/tests/optimizer_benchmark.cpp') + '")\n'
s += 'target_link_libraries(optimizer_benchmark traj_opt ${catkin_LIBRARIES})\n'
p.write_text(s)
PY
cd "$workspace"
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release > build-output.log 2>&1
