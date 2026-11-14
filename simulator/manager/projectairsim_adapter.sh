#!/usr/bin/env bash
# Start the native ProjectAirSim bridge after the manager has loaded the scene.
set -eo pipefail
if [[ -z ${PROJECTAIRSIM_ROS_SETUP:-} || ! -f ${PROJECTAIRSIM_ROS_SETUP} ]]; then
  echo "PROJECTAIRSIM_ROS_SETUP must name an installed overlay setup.bash" >&2
  exit 2
fi
if [[ -n ${PROJECTAIRSIM_ROS_IMAGE:-} ]]; then
  # Reuse the fixed ROS tool/runtime image; the overlay remains external.
  overlay=$(dirname "$PROJECTAIRSIM_ROS_SETUP")
  bridge_script=$(cd "$(dirname "$0")" && pwd)/$(basename "$0")
  mounts=()
  if [[ -n ${PROJECTAIRSIM_SENSOR_RUNTIME:-} ]]; then
    simulator_root=$(cd "$(dirname "$0")/.." && pwd)
    mounts=(-v "$PROJECTAIRSIM_SENSOR_RUNTIME:/sim-runtime:ro"
      -v "$simulator_root:/simulator:ro"
      -e PROJECTAIRSIM_SENSOR_RUNTIME=/sim-runtime)
  fi
  dds_mounts=()
  if [[ -n ${PROJECTAIRSIM_DDS_PROFILE:-} ]]; then
    dds_mounts=(-v "$PROJECTAIRSIM_DDS_PROFILE:/dds-profile.xml:ro"
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/dds-profile.xml)
  fi
  exec docker run --rm --init --network host --ipc host \
    --label aerohub.component=projectairsim-adapter \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    -e "PROJECTAIRSIM_ADDRESS=${PROJECTAIRSIM_ADDRESS:-127.0.0.1}" \
    -e "PROJECTAIRSIM_TOPIC_PORT=${PROJECTAIRSIM_TOPIC_PORT:-8989}" \
    -e "PROJECTAIRSIM_SERVICE_PORT=${PROJECTAIRSIM_SERVICE_PORT:-8990}" \
    -e "PROJECTAIRSIM_ROS_ARGS=${PROJECTAIRSIM_ROS_ARGS:-[]}" \
    -e "PROJECTAIRSIM_SENSOR_TOPIC_ROOT=${PROJECTAIRSIM_SENSOR_TOPIC_ROOT:-/ProjectAirsim}" \
    -e PROJECTAIRSIM_ROS_SETUP=/overlay/setup.bash \
    "${mounts[@]}" "${dds_mounts[@]}" \
    -v "$overlay:/overlay:ro" -v "$bridge_script:/adapter.sh:ro" \
    --entrypoint bash "$PROJECTAIRSIM_ROS_IMAGE" /adapter.sh
fi
set +u
source /opt/ros/humble/setup.bash
source "$PROJECTAIRSIM_ROS_SETUP"
set -u
mapfile -t extra < <(python3 -c 'import json,os; print("\n".join(json.loads(os.environ.get("PROJECTAIRSIM_ROS_ARGS", "[]"))))')
bridge_command=(ros2 run projectairsim_ros2_cpp projectairsim_ros2_cpp_node --ros-args \
  -p "address:=${PROJECTAIRSIM_ADDRESS:-127.0.0.1}" \
  -p "port_topics:=${PROJECTAIRSIM_TOPIC_PORT:-8989}" \
  -p "port_services:=${PROJECTAIRSIM_SERVICE_PORT:-8990}" \
  -p publish_tf:=false "${extra[@]}")
if [[ -z ${PROJECTAIRSIM_SENSOR_RUNTIME:-} ]]; then
  exec "${bridge_command[@]}"
fi
children=()
cleanup() {
  trap - EXIT INT TERM
  kill -TERM "${children[@]}" 2>/dev/null || true
  wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
"${bridge_command[@]}" &
children+=("$!")
if [[ -f /simulator/projectairsim/sensor_node.py ]]; then
  sensor_script=/simulator/projectairsim/sensor_node.py
else
  sensor_script=$(cd "$(dirname "$0")/../projectairsim" && pwd)/sensor_node.py
fi
python3 "$sensor_script" --scene "$PROJECTAIRSIM_SENSOR_RUNTIME/resolved_scene.json" \
  --runtime "$PROJECTAIRSIM_SENSOR_RUNTIME/runtime.json" \
  --topic-root "${PROJECTAIRSIM_SENSOR_TOPIC_ROOT:-/ProjectAirsim}" &
children+=("$!")
# Either child's exit makes this adapter unhealthy; manager status observes it.
wait -n "${children[@]}"
exit 1
