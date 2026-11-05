#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/platform_sensors/install/setup.bash
exec python3 /opt/flight_board/supervisor.py "$@"
