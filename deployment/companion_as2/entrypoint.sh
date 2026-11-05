#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
if [[ ${1:-run} == run ]]; then
  exec python3 /opt/companion_as2/run.py
fi
exec "$@"
