#!/usr/bin/env bash
# Optional PC development check; requires companion message interfaces.
# Formal delivery verification uses verify_robopi.sh.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
out="${VERIFY_OUTPUT:-$root/artifacts/flight-board-verification}"
source "$root/scripts/load_env.sh"
mkdir -p "$out"
image=${FLIGHT_BOARD_IMAGE:-flight_board:airsim-px4-humble-threaded-20261105}
image=$(docker image inspect "$image" --format '{{.Id}}')
printf '%s\n' "$image" > "$out/flight-board-image.txt"
names=(aerohub-fb-vehicle aerohub-fb-clock)
if [[ ${VERIFY_AS2:-0} == 1 ]]; then
  names+=(aerohub-fb-as2)
  as2_image=$(docker image inspect companion_as2:airsim-px4 --format '{{.Id}}')
  printf '%s\n' "$as2_image" > "$out/as2-image.txt"
else
  ros_image=$(docker image inspect "${VERIFY_ROS_IMAGE:-companion_as2_dependencies:humble-px4-1.18}" --format '{{.Id}}')
  printf '%s\n' "$ros_image" > "$out/ros-image.txt"
fi
for name in "${names[@]}"; do
  if docker container inspect "$name" >/dev/null 2>&1; then
    echo "Verification container already exists: $name" >&2
    exit 1
  fi
done
cleanup() {
  docker stop -t 7 "${names[@]}" >/dev/null 2>&1 || true
  python3 "$root/scripts/airsim/manager.py" stop --run-dir "$out/scene" || true
}
trap cleanup EXIT
# Resolve the dedicated NH profile through the same resource mapping as normal runs.
bash "$root/scripts/airsim.sh" check --profile "$root/config/airsim/nh.json" \
  > "$out/scene-config.json"
python3 - "$out/scene-config.json" "$out/profile.json" <<'PYPROFILE'
import json, sys
with open(sys.argv[1]) as stream:
    profile = json.load(stream)['profile']
with open(sys.argv[2], 'w') as stream:
    json.dump(profile, stream, indent=2)
PYPROFILE
python3 "$root/scripts/airsim/manager.py" start --profile "$out/profile.json" \
  --run-dir "$out/scene" > "$out/scene-start.json"
docker run --rm --name aerohub-fb-clock --network host \
  -e ROS_DOMAIN_ID=81 "$image" clock > "$out/clock.log" 2>&1 &
docker run --rm --name aerohub-fb-vehicle --network host \
  -e ROS_DOMAIN_ID=81 -e PX4_INSTANCE=20 -e PX4_SYSTEM_ID=21 -e XRCE_AGENT_PORT=8891 \
  -v "$root/config/airsim/settings/px4-sensors.json:/config/settings.json:ro" \
  "$image" vehicle > "$out/vehicle.log" 2>&1 &
for ((i=0; i<45; i++)); do
  if rg -q 'Startup script returned successfully' "$out/vehicle.log"; then
    break
  fi
  sleep 1
done
for ((i=0; i<15; i++)); do
  docker exec aerohub-fb-vehicle /opt/px4-gazebo/bin/px4-uxrce_dds_client --instance 20 status \
    > "$out/dds.log"
  if grep -q 'Running, connected' "$out/dds.log"; then
    break
  fi
  sleep 1
done
grep -q 'Running, connected' "$out/dds.log"
if [[ ${VERIFY_AS2:-0} == 1 ]]; then
  docker run --rm --name aerohub-fb-as2 --network host \
    -e ROS_DOMAIN_ID=81 \
    -e VERIFY_AS2_FLIGHT="${VERIFY_AS2_FLIGHT:-0}" \
    -v "$root/runtime/as2/scripts:/verification:ro" \
    -v "$root/runtime/airsim_sensors/scripts:/sensor-verification:ro" \
    -v "$root/config/airsim/settings/px4-sensors.json:/config/settings.json:ro" \
    "$as2_image" > "$out/as2.log" 2>&1 &
  for ((i=0; i<15; i++)); do
    if [[ $(docker inspect -f '{{.State.Running}}' aerohub-fb-as2 2>/dev/null || true) == true ]]; then
      break
    fi
    sleep 1
  done
  if ! docker exec aerohub-fb-as2 /usr/local/bin/companion-as2 \
    python3 /verification/verify_interaction.py > "$out/as2-interaction.json" 2> "$out/as2-check.log"; then
    for topic in offboard_control_mode failsafe_flags timesync_status vehicle_status; do
      docker exec aerohub-fb-vehicle /opt/px4-gazebo/bin/px4-listener \
        --instance 20 "$topic" -n 1 > "$out/px4-$topic.log" 2>&1 || true
    done
    exit 1
  fi
  docker exec aerohub-fb-as2 /usr/local/bin/companion-as2 \
    python3 /sensor-verification/verify_ros2.py > "$out/as2-sensors.json"
else
# Observe topics with the documented fixed Humble/PX4 message dependency image.
docker run --rm --network host -e ROS_DOMAIN_ID=81 \
  -v "$root/config/airsim/settings/px4-sensors.json:/config/settings.json:ro" \
  -v "$root/config/dds/fastdds_udp.xml:/opt/fastdds_udp.xml:ro" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_udp.xml \
  -v "$root/runtime/airsim_sensors/scripts:/verification:ro" \
  --entrypoint bash "$ros_image" -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/px4_ws/install/setup.bash
    python3 /verification/verify_ros2.py
    timeout 15 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --qos-reliability best_effort
    timeout 15 ros2 topic echo /drone0/fmu/out/vehicle_odometry px4_msgs/msg/VehicleOdometry --once --qos-reliability best_effort
  ' > "$out/ros-humble.log" 2>&1
fi
grep -q 'Running, connected' "$out/dds.log"
docker exec aerohub-fb-vehicle python3 /opt/flight_board/supervisor.py status > "$out/status.json"
docker exec aerohub-fb-clock python3 /opt/flight_board/supervisor.py healthcheck
for ((i=0; i<10; i++)); do
  health=$(docker inspect aerohub-fb-vehicle --format '{{.State.Health.Status}}')
  [[ "$health" == healthy ]] && break
  sleep 1
done
[[ "$health" == healthy ]]
echo "AirSim/PX4 acceptance passed (AS2=${VERIFY_AS2:-0}); evidence: $out"
