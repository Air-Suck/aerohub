#!/usr/bin/env bash
# Offline module/version probe. Does not start a simulator or connect to a vehicle.
set -euo pipefail
image=${PX4_IMAGE:-px4io/px4-sitl-gazebo@sha256:54a7a15ba35b3f4786256296d89edd144e000f40084cb6ed692fcbf74c9168f2}
docker run --rm --network none --read-only --tmpfs /tmp --workdir /tmp \
  --entrypoint /bin/bash "$image" -lc '
    printf "%s\n" \
      "export PATH=/opt/px4-gazebo/bin:\$PATH" \
      ". /opt/px4-gazebo/bin/px4-alias.sh" \
      "ver all" "uxrce_dds_client status" \
      "simulator_mavlink status" "shutdown" > /tmp/probe.rc
    /opt/px4-gazebo/bin/px4 -d -s /tmp/probe.rc /opt/px4-gazebo
  '
