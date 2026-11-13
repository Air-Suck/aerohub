#!/usr/bin/env bash
# Run inside the pinned onboard image with /work mounted read-only and network none.
set -eo pipefail
project=$(cd "$(dirname "$0")/../../../../.." && pwd)
out=${1:?new evidence root required}
mkdir -p "$out"
source /opt/ros/humble/local_setup.bash
source /opt/px4_ws/install/local_setup.bash
source /work/install/humble/local_setup.bash
source /root/aerostack2_ws/install/local_setup.bash
source /work/ground_overlay/install/local_setup.bash
source /work/ego_runtime/install/local_setup.bash
export ROS_DOMAIN_ID=103 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR=/tmp/ego-adapter-split-ros-log
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
workspace=$(mktemp -d /tmp/ego-adapter-split.XXXXXX)
colcon --log-base "$out/build-log" build \
    --base-paths "$project/src/planner/ego_planner_v2/ros2_adapter/bridge" \
    --build-base "$workspace/build" --install-base "$workspace/install" \
    --packages-select uav_ego_planner_adapter --event-handlers console_direct+
source "$workspace/install/local_setup.bash"
python3 -B "$project/src/planner/ego_planner_v2/ros2_adapter/tests/test_adapter_scope.py"
failed=0
for mode in complete cancel stale; do
    python3 -B "$project/src/planner/ego_planner_v2/ros2_adapter/tests/verify_split.py" "$out/$mode" "$mode" || failed=1
done
python3 -B "$project/src/planner/ego_planner_v2/ros2_adapter/tests/verify_as2.py" "$out/legacy-cancel" cancel || failed=1
exit "$failed"
