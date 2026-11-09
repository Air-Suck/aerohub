#!/usr/bin/env bash
# Offline process/configuration test; no PX4, Agent, simulator or flight command.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
out="${VERIFY_OUTPUT:-$root/log/airsim/onboard-computer-verification}"
image="${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}"
mkdir -p "$out"
name=aerohub-onboard-computer-smoke
required=(platform state_estimator controller_manager)
IFS=, read -r -a behaviors <<< "${AS2_BEHAVIORS:-takeoff,land,go_to,follow_path}"
for behavior in "${behaviors[@]}"; do
  case "$behavior" in
    takeoff) required+=(TakeoffBehavior) ;;
    land) required+=(LandBehavior) ;;
    go_to) required+=(GoToBehavior) ;;
    follow_path) required+=(FollowPathBehavior) ;;
    *) echo "Unsupported behavior: $behavior" >&2; exit 2 ;;
  esac
done
if docker container inspect "$name" >/dev/null 2>&1; then
  echo "Verification container already exists: $name" >&2
  exit 1
fi
cleanup() {
  docker stop -t 7 "$name" >/dev/null 2>&1 || true
  if [[ -n ${runtime_pid:-} ]]; then wait "$runtime_pid" 2>/dev/null || true; fi
  # --rm removal may finish just after stop returns; serialize repeated checks.
  for ((attempt=0; attempt<50; attempt++)); do
    if ! docker container inspect "$name" >/dev/null 2>&1; then return; fi
    sleep .1
  done
  echo "Verification container cleanup incomplete: $name" >&2
  return 1
}
trap cleanup EXIT
docker run --rm --name "$name" --network none -e ROS_LOCALHOST_ONLY=1 \
  -e ROS_DOMAIN_ID=82 -e AS2_TF_SCOPE="${AS2_TF_SCOPE:-}" \
  -e AS2_BEHAVIORS="${AS2_BEHAVIORS:-}" \
  -v "$root/deployment/onboard_computer/check_nodes.py:/verification/check_nodes.py:ro" "$image" > "$out/startup.log" 2>&1 &
runtime_pid=$!
for ((attempt=0; attempt<50; attempt++)); do
  if [[ $(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true) == true ]]; then break; fi
  sleep .1
done
docker exec "$name" /usr/local/bin/onboard-computer python3 /verification/check_nodes.py \
  --tf-scope "${AS2_TF_SCOPE:-shared}" "${required[@]}" > "$out/node-check.json"
docker exec "$name" bash -c '! command -v MicroXRCEAgent'
if grep -E 'process has died|Caught exception|Traceback' "$out/startup.log"; then
  exit 1
fi
echo "onboard_computer offline smoke passed; evidence: $out"
