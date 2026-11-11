#!/system/bin/sh
# Usage: sh run_houmo_ros_board.sh DEPLOY_ROOT [ROS_BASE_ENV]
set -eu
base=${1:?Specify the deployment root containing board-runtime/ and ros/install/}
ros_env=${2:-/data/aerohub/as2/board_env.sh}
. "$ros_env"
export PATH="$PATH:/system/bin"
export ROS_DOMAIN_ID="${INFERENCE_TEST_ROS_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/data/aerohub/as2/fastdds_udp.xml}"
export PYTHONDONTWRITEBYTECODE=1
for prefix in "$base"/ros/install/*; do
 [ -d "$prefix/share" ] || continue
 export AMENT_PREFIX_PATH="$prefix:$AMENT_PREFIX_PATH"
 export LD_LIBRARY_PATH="$prefix/lib:$LD_LIBRARY_PATH"
 export PYTHONPATH="$prefix/lib/python3.12/site-packages:$PYTHONPATH"
done
exec /sys_prod/robot/out/bin/python3 "$base/verify_ros.py" \
 --runtime "$base/board-runtime" --log-dir "$base/ros-results"
