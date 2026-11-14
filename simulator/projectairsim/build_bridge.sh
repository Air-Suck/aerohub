#!/usr/bin/env bash
# Build the upstream ROS bridge in the existing ROS tool image and an external overlay.
set -euo pipefail
if [[ $# -lt 2 ]]; then
  echo "Usage: bash $0 PROJECTAIRSIM_SOURCE OVERLAY_WORKSPACE [extra CMake arguments...]" >&2
  exit 2
fi
source_root=$(realpath "$1")
script_root=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$2"
workspace=$(realpath "$2")
shift 2
test -f "$source_root/ros/projectairsim_ros2_cpp/package.xml"
image=${PROJECTAIRSIM_ROS_IMAGE:-onboard_computer_dependencies:humble-px4-1.18}
docker image inspect "$image" --format '{{.Id}}'
docker run --rm --init --user "$(id -u):$(id -g)" \
  -e ROS_HOME=/overlay/.ros -e CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}" \
  -v "$source_root:$source_root:ro" -v "$workspace:/overlay" -w /overlay \
  -v "$script_root/patches:/patches:ro" \
  --entrypoint bash "$image" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source_root=$1
    shift
    python3 - "$source_root" <<"PY"
import pathlib, shutil, sys
source = pathlib.Path(sys.argv[1]) / "ros/projectairsim_ros2_cpp"
target = pathlib.Path("src/projectairsim_ros2_cpp")
target.mkdir(parents=True, exist_ok=True)
shutil.copytree(source, target, dirs_exist_ok=True)
PY
    for patch in /patches/*.patch; do
      git -C src/projectairsim_ros2_cpp apply --check "$patch"
      git -C src/projectairsim_ros2_cpp apply "$patch"
    done
    colcon build --base-paths src/projectairsim_ros2_cpp \
      --packages-select projectairsim_ros2_cpp --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
        "-DPROJECTAIRSIM_CPP_CLIENT_DIR=$source_root/client/cpp" "$@"
    source install/setup.bash
    colcon test --base-paths src/projectairsim_ros2_cpp \
      --packages-select projectairsim_ros2_cpp --executor sequential
    colcon test-result --all --verbose
  ' _ "$source_root" "$@"
