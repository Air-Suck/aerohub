#!/usr/bin/env bash
set -euo pipefail

demo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PLANNER_DEMO_DIR="$demo_dir"
project_dir=${CATORA_STAGE2_DIR:?Set CATORA_STAGE2_DIR to the historical stage-2 deployment directory}
[[ "$project_dir" = /* && -f "$project_dir/compose.yaml" && -f "$project_dir/verify.sh" ]] || { echo "Invalid CATORA_STAGE2_DIR" >&2; exit 2; }
base=$project_dir/compose.yaml
override=$demo_dir/compose.catora-demo.yaml
frames=$demo_dir/.frames_catora
output=$demo_dir/catora_px4_gazebo_headless.mp4
intermediate=$demo_dir/.catora_mp4v.mp4

cleanup() {
  rc=$?
  if ((rc != 0)); then
    docker logs --tail 200 catora-planner-px4 2>&1 || true
    docker logs --tail 200 catora-px4-gazebo 2>&1 || true
  fi
  docker exec catora-px4-gazebo pkill -f '^/demo/capture_gz_images ' 2>/dev/null || true
  docker compose -f "$base" -f "$override" down --remove-orphans 2>/dev/null || true
}
trap cleanup EXIT

mkdir -p "$frames"
find "$frames" -maxdepth 1 -type f -name 'frame_*.ppm' -delete
cd "$project_dir"
docker compose -f "$base" -f "$override" down --remove-orphans
docker compose -f "$base" -f "$override" up -d px4

for _ in $(seq 1 90); do
  docker exec catora-px4-gazebo gz topic -l 2>/dev/null | grep -Fxq /demo/camera/image && break
  sleep 1
done
docker exec catora-px4-gazebo gz topic -l | grep -Fx /demo/camera/image
sleep 25
docker compose -f "$base" -f "$override" up -d planner
for _ in $(seq 1 240); do
  status=$(docker exec catora-planner-px4 bash -lc \
    'source /opt/catora_ws/devel/setup.bash; timeout 2 rostopic echo -n 1 /catora_px4/status 2>/dev/null' || true)
  grep -q 'phase.*ready' <<<"$status" && break
  sleep 1
done
grep -q 'phase.*ready' <<<"$status"
docker exec -d catora-px4-gazebo /demo/capture_gz_images /demo/camera/image /demo/.frames_catora 1800
./verify.sh
sleep 3
docker exec catora-px4-gazebo pkill -f '^/demo/capture_gz_images ' 2>/dev/null || true
docker compose -f "$base" -f "$override" down

docker run --rm --user 1000:1000 -v "$demo_dir:/demo" --entrypoint python3 \
  uav/catora:noetic-px4 /demo/encode_video.py /demo/.frames_catora /demo/$(basename "$intermediate") \
  --fps 10 --title 'CAT-ORA | 16 UAV square-to-circle reshape | PX4 SITL + Gazebo Harmonic'
"$demo_dir/encode_compatible_mp4.sh" "$intermediate" "$output"
test -s "$output"
rm -f "$intermediate"
find "$frames" -maxdepth 1 -type f -name 'frame_*.ppm' -delete
rmdir "$frames"
echo "Demo written to $output"
