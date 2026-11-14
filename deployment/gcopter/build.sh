#!/usr/bin/env bash
set -euo pipefail
GCOPTER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GCOPTER_IMAGE=3e5d82c5dce5
test "$(git -C "$GCOPTER_ROOT/src/planner/gcopter/upstream" rev-parse HEAD)" = e0444f6d47b84f972ced91746b05feb36ce1fd4f
test -z "$(git -C "$GCOPTER_ROOT/src/planner/gcopter/upstream" status --porcelain)"
docker run --rm --entrypoint bash -v "$GCOPTER_ROOT:/workspace" "$GCOPTER_IMAGE" -lc '
set -euo pipefail
cd /workspace
mkdir -p log/gcopter/deps
if ! (cd log/gcopter/deps && sha256sum -c /workspace/deployment/gcopter/dependencies.sha256 --status); then
  cd log/gcopter/deps
  apt-get update -qq
  apt-get download libompl-dev=1.5.2+ds1-1 libompl16=1.5.2+ds1-1 libode-dev=2:0.16.2-1 libode8=2:0.16.2-1 libccd2=2.1-2 libboost-serialization1.74.0=1.74.0-14ubuntu3 libboost-filesystem1.74.0=1.74.0-14ubuntu3 python3-msgpack=1.0.3-1build1
  apt-get download ros-humble-geographic-msgs=1.0.6-1jammy.20260907.212142
  cd /workspace
fi
(cd log/gcopter/deps && sha256sum -c /workspace/deployment/gcopter/dependencies.sha256)
for package in log/gcopter/deps/*.deb; do dpkg-deb -x "$package" /; done
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base log/gcopter/colcon build --base-paths as2/aerostack2/as2_msgs src/planner/interfaces src/planner/framework src/planner/gcopter/ros2_adapter --build-base log/gcopter/ros-build --install-base log/gcopter/install --cmake-args -DGCOPTER_WITH_ROS=ON -DCMAKE_BUILD_TYPE=Release
ctest --test-dir log/gcopter/ros-build/uav_gcopter_core --output-on-failure
'
