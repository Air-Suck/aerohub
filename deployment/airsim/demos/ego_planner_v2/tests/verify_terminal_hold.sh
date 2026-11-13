#!/usr/bin/env bash
# Isolated fixed-image regression; source and /work overlays stay read-only.
set -eo pipefail
project=$(cd "$(dirname "$0")/../../../../.." && pwd)
out=${1:?evidence directory required}
stage=${2:-all}
mkdir -p "$out"
source /opt/ros/humble/local_setup.bash
source /opt/px4_ws/install/local_setup.bash
source /work/install/humble/local_setup.bash
source /root/aerostack2_ws/install/local_setup.bash
source /work/ground_overlay/install/local_setup.bash
source /work/ego_runtime/install/local_setup.bash
export ROS_DOMAIN_ID=104 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR=/tmp/ego-terminal-hold-ros-log
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
workspace=$(mktemp -d /tmp/ego-terminal-hold.XXXXXX)
colcon --log-base "$out/build-log" build --base-paths "$project/src/planner/ego_planner_v2/ros2_adapter/bridge" \
    --build-base "$workspace/build" --install-base "$workspace/install" \
    --packages-select uav_ego_planner_adapter > "$out/build.txt" 2>&1
source "$workspace/install/local_setup.bash"
tests="$project/src/planner/ego_planner_v2/ros2_adapter/tests"
python3 -B "$tests/test_terminal_hold.py"
python3 -B "$tests/test_as2_conversion.py"
python3 -B "$tests/test_adapter_scope.py"
if [[ "$stage" == unit ]]; then exit 0; fi
failed=0
for mode in normal late; do
    python3 -B "$tests/verify_tail_replay.py" "$out/replay-$mode" "$mode" || failed=1
done
if [[ "$stage" == replay ]]; then exit "$failed"; fi
for index in $(seq -w 1 12); do
    python3 -B "$tests/verify_split.py" "$out/complete-$index" complete || failed=1
done
for mode in cancel stale; do
    python3 -B "$tests/verify_split.py" "$out/$mode" "$mode" || failed=1
done
python3 -B "$tests/verify_as2.py" "$out/legacy-cancel" cancel || failed=1
exit "$failed"
