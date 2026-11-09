#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/catora_ws/install/setup.bash
ros2 run mrs_formation_reshaping cat_ora_node &
planner_pid=$!
ros2 run mrs_formation_reshaping ros2_proxy.py &
proxy_pid=$!
trap 'kill "$planner_pid" "$proxy_pid" 2>/dev/null || true; wait || true' EXIT
trap 'exit 0' TERM INT
set +e
wait -n "$planner_pid" "$proxy_pid"
result=$?
set -e
# Both processes are long-running; an unexpected clean exit also needs a restart.
if [ "$result" -eq 0 ]; then result=1; fi
exit "$result"
