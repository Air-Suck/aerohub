#!/usr/bin/env bash
# AS2 preset on the shared OHOS builder entrypoint.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
if [[ ${1:-as2_core} == runtime ]]; then
  shift
  set -- rmw_fastrtps_shared_cpp as2_platform_pixhawk as2_state_estimator as2_motion_controller as2_behaviors_motion "$@"
elif (( $# == 0 )); then
  set -- as2_core
fi
exec bash "$root/deployment/oh-ros-build/run.sh" build "$@"
