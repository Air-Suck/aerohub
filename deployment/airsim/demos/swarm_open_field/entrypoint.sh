#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
source /root/aerostack2_ws/install/setup.bash
if [[ -f /opt/uav_execution_ws/install/setup.bash ]]; then
  source /opt/uav_execution_ws/install/setup.bash
fi
source /opt/swarm_ws/install/setup.bash
if [[ ${1:-worker} == worker ]]; then exec python3 /opt/swarm_app/worker.py; fi
exec "$@"
