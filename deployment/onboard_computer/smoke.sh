#!/usr/bin/env bash
# Offline process/configuration test; no PX4, Agent, simulator or flight command.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
out="$root/artifacts/onboard-computer-verification"
mkdir -p "$out"
name=aerohub-onboard-computer-smoke
if docker container inspect "$name" >/dev/null 2>&1; then
  echo "Verification container already exists: $name" >&2
  exit 1
fi
cleanup() {
  docker stop -t 7 "$name" >/dev/null 2>&1 || true
}
trap cleanup EXIT
docker run --rm --name "$name" --network none -e ROS_LOCALHOST_ONLY=1 \
  -e ROS_DOMAIN_ID=82 onboard_computer:airsim-px4 > "$out/startup.log" 2>&1 &
for ((i=0; i<15; i++)); do
  docker exec "$name" /usr/local/bin/onboard-computer ros2 node list --no-daemon \
    > "$out/nodes.log" 2>/dev/null || true
  if grep -qx '/drone0/platform' "$out/nodes.log" \
    && grep -qx '/drone0/state_estimator' "$out/nodes.log" \
    && grep -qx '/drone0/controller_manager' "$out/nodes.log"; then
    break
  fi
  sleep 1
done
for node in platform state_estimator controller_manager; do
  grep -qx "/drone0/$node" "$out/nodes.log"
  docker exec "$name" /usr/local/bin/onboard-computer ros2 param get \
    "/drone0/$node" use_sim_time > "$out/$node-time.log"
  grep -q 'True' "$out/$node-time.log"
done
docker exec "$name" bash -c '! command -v MicroXRCEAgent'
if grep -E 'process has died|Caught exception|Traceback' "$out/startup.log"; then
  exit 1
fi
echo "onboard_computer offline smoke passed; evidence: $out"
