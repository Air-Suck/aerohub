#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/catora_app_ws/install/setup.bash
if [[ ${1:-worker} == worker ]]; then
 exec python3 /opt/catora_hybrid/worker.py
fi
exec "$@"
