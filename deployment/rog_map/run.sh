#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=${ROG_MAP_WORKSPACE:-$root/log/workspaces/rog_map}
image=${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}
namespace=${ROG_MAP_NAMESPACE:-drone0}
domain=${ROS_DOMAIN_ID:-0}
container=${ROG_MAP_CONTAINER:-aerohub-rog-map-$namespace}

test -f "$workspace/install/setup.bash" || {
  printf 'ROG-Map overlay is missing; run deployment/rog_map/prepare_workspace.sh\n' >&2
  exit 2
}
docker image inspect "$image" >/dev/null

exec docker run --rm --init --network host \
  --name "$container" \
  --label aerohub.task=rog-map-runtime \
  --label "aerohub.namespace=$namespace" \
  --entrypoint bash \
  --env ROS_DOMAIN_ID="$domain" \
  --volume "$workspace:/workspace:ro" \
  "$image" -lc '
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /workspace/install/setup.bash
exec ros2 launch uav_rog_map rog_map.launch.py \
  namespace:='"$namespace"' \
  global_frame:='"$namespace"'/odom \
  sensor_frame:='"$namespace"'/Lidar
'
