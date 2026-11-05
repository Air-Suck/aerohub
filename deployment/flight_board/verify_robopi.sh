#!/usr/bin/env bash
# Dedicated SITL verification with the already deployed, running RoboPi AS2.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
source "$root/scripts/load_env.sh"
out=${VERIFY_OUTPUT:-$root/artifacts/flight-board-robopi-verification}
image=${FLIGHT_BOARD_IMAGE:-flight_board:airsim-px4-humble-threaded-20261105}
memory_seconds=${VERIFY_MEMORY_SECONDS:-180}
[[ "$memory_seconds" =~ ^[0-9]+$ && "$memory_seconds" -gt 15 ]] || {
  echo 'VERIFY_MEMORY_SECONDS must be an integer greater than 15' >&2; exit 2;
}
mkdir -p "$out"
out=$(cd "$out" && pwd)
image=$(docker image inspect "$image" --format '{{.Id}}')
printf '%s\n' "$image" > "$out/image.txt"
names=(aerohub-verify-robopi-vehicle aerohub-verify-robopi-clock)
for name in "${names[@]}"; do
  if docker container inspect "$name" >/dev/null 2>&1; then
    echo "Verification container already exists: $name" >&2; exit 1
  fi
done
ping -c 1 -W 2 "${ROBOPI_IP:?Configure .env first}"
python3 - "$root" "$out" <<'PY'
import json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1])/'scripts'))
from robopi import board, board_environment, clock_offset, profile
state = json.loads(board(board_environment()+'python3 manage_runtime.py status'))
offset = clock_offset()
if abs(offset) > 0.5:
    raise RuntimeError('Board/PC clocks differ; run scripts/robopi.sh sync-time first')
out = Path(sys.argv[2])
profile(out/'pc-dds.xml', os.environ['PC_IP'])
(out/'preflight.json').write_text(json.dumps(dict(board=state, clock_offset_s=offset), indent=2)+'\n')
PY
monitor_pid=''
cleanup() {
  if [[ -n "$monitor_pid" ]]; then
    kill "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
  fi
  docker logs "${names[0]}" > "$out/vehicle.log" 2>&1 || true
  docker logs "${names[1]}" > "$out/clock.log" 2>&1 || true
  docker stop -t 7 "${names[@]}" >/dev/null 2>&1 || true
  bash "$root/scripts/airsim.sh" stop --run-dir "$out/scene" || true
}
trap cleanup EXIT
bash "$root/scripts/airsim.sh" start --profile "$root/config/airsim/nh.json" \
  --run-dir "$out/scene" > "$out/scene-start.json"
docker run -d --rm --name "${names[0]}" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:?Configure .env first}" \
  -e PX4_INSTANCE=20 -e PX4_SYSTEM_ID=21 -e XRCE_AGENT_PORT=8891 \
  -v "$root/config/airsim/settings/px4-sensors.json:/config/settings.json:ro" \
  -v "$out/pc-dds.xml:/opt/fastdds_udp.xml:ro" "$image" vehicle
docker run -d --rm --name "${names[1]}" --network host \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -v "$out/pc-dds.xml:/opt/fastdds_udp.xml:ro" "$image" clock
sleep 10
python3 "$root/scripts/diagnostics/verify_sensor_memory.py" --duration "$memory_seconds" \
  --output "$out/memory.json" > "$out/memory.log" &
monitor_pid=$!
python3 - "$root" "$out" <<'PY'
import json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1])/'scripts'))
from robopi import board, board_environment, report_json
out = Path(sys.argv[2])
flight = os.environ.get('VERIFY_AS2_FLIGHT', '0') == '1'
try:
    result = board(board_environment(flight=flight)+'python3 verify_interaction.py', timeout=240)
except Exception as error:
    (out/'interaction.log').write_text(str(error)+'\n')
    raise
(out/'interaction.log').write_text(result+'\n')
(out/'interaction.json').write_text(report_json(result))
result = board(board_environment()+'python3 verify_sensors.py', timeout=90)
(out/'sensors.json').write_text(report_json(result))
print('RoboPi interaction and sensors passed (flight='+str(flight)+')', flush=True)
PY
wait "$monitor_pid"
monitor_pid=''
docker exec "${names[0]}" python3 /opt/flight_board/supervisor.py status > "$out/status.json"
docker exec "${names[1]}" python3 /opt/flight_board/supervisor.py healthcheck
printf 'RoboPi acceptance passed; memory window=%ss; evidence=%s\n' "$memory_seconds" "$out"
