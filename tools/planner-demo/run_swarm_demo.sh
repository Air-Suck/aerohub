#!/usr/bin/env bash
set -euo pipefail

demo_dir=/home/noah/code/UAV/demo
project_dir=/home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2
base=$project_dir/compose.seven.yaml
override=$demo_dir/compose.swarm-demo.yaml
frames=$demo_dir/.frames_swarm
output=$demo_dir/swarm_formation_px4_gazebo_headless.mp4
intermediate=$demo_dir/.swarm_mp4v.mp4
record=${SWARM_RECORD:-1}

cleanup() {
  docker exec px4-gazebo-seven pkill -f '^/demo/capture_gz_images ' 2>/dev/null || true
  docker compose -f "$base" -f "$override" down --remove-orphans 2>/dev/null || true
}
trap cleanup EXIT

mkdir -p "$frames"
rm -f "$demo_dir/swarm_forest_metrics.json"
find "$frames" -maxdepth 1 -type f -name 'frame_*.ppm' -delete
python3 "$demo_dir/sync_forest_sdf.py" "$demo_dir/swarm_formation_demo.sdf"
cd "$project_dir"
docker compose -f "$base" -f "$override" down --remove-orphans
docker compose -f "$base" -f "$override" up -d

for _ in $(seq 1 120); do
  docker exec px4-gazebo-seven gz topic -l 2>/dev/null | grep -Fxq /demo/camera/image && break
  sleep 1
done
docker exec px4-gazebo-seven gz topic -l | grep -Fx /demo/camera/image

# Seven MAVROS instances synchronize parameters sequentially on a slow
# headless simulation. Warm the complete flight stack before recording and
# before verify_seven.sh takes its first snapshot.
ready=0
for _ in $(seq 1 120); do
  ready=0
  for i in $(seq 0 6); do
    state=$(docker exec sf-px4-seven bash -lc \
      "source /workspace/devel/setup.bash; timeout 3 rostopic echo -n 1 /uav${i}/mavros/state 2>/dev/null" || true)
    if grep -q 'connected: True' <<<"$state" && grep -q 'armed: True' <<<"$state" && grep -q 'mode: "OFFBOARD"' <<<"$state"; then
      ready=$((ready + 1))
    fi
  done
  ((ready == 7)) && break
  sleep 1
done
((ready == 7))
sleep 10

# Replace the empty stage-2 map with the deterministic point cloud matching
# the Gazebo cylinders before issuing the traversal goal.
docker exec sf-px4-seven bash -lc \
  'source /workspace/devel/setup.bash; rosnode kill /random_forest >/dev/null 2>&1 || true'
docker exec -d sf-px4-seven bash -lc \
  'source /workspace/devel/setup.bash; exec python3 /demo/forest_map_publisher.py'
for _ in $(seq 1 30); do
  width=$(docker exec sf-px4-seven bash -lc \
    'source /workspace/devel/setup.bash; timeout 3 rostopic echo -n 1 /map_generator/global_cloud/width 2>/dev/null' \
    | awk '/^[0-9]+$/{print; exit}' || true)
  [[ ${width:-0} -gt 1000 ]] && break
  sleep 1
done
[[ ${width:-0} -gt 1000 ]]

if ((record)); then
  docker exec -d px4-gazebo-seven /demo/capture_gz_images /demo/camera/image /demo/.frames_swarm 5000
fi
docker exec sf-px4-seven bash -lc \
  'source /workspace/devel/setup.bash; exec python3 /demo/swarm_forest_mission.py'
docker exec sf-px4-seven python3 -c \
  'import json; r=json.load(open("/demo/swarm_forest_metrics.json")); assert r["complete"], r; assert r["configured_waypoints"] == [[24.0, 0.0]], r; assert r["final_center_x_m"] > 22.0, r; assert r["pillar_count"] == 60, r; assert r["ring_count"] == 20, r; assert r["blocked_nominal_path_tree_count"] >= 1, r; assert r["nominal_path_min_clearance_m"] < 0.0, r; assert r["max_local_avoidance_deviation_m"] > 0.2, r; assert r["min_vehicle_separation_m"] > 0.8, r; assert r["min_command_vehicle_separation_m"] > 0.8, r; assert r["min_obstacle_clearance_m"] > 0.1, r; assert r["min_command_obstacle_clearance_m"] > 0.1, r; assert r["min_altitude_m"] > 0.5, r; assert r["min_command_altitude_m"] > 0.6, r; assert r["max_formation_rms_m"] < 1.5, r; assert r["final_formation_rms_m"] < 0.3, r; print(json.dumps({k:v for k,v in r.items() if k != "vehicle_traces"}, indent=2))'
if ((record)); then sleep 2; fi
docker exec px4-gazebo-seven pkill -f '^/demo/capture_gz_images ' 2>/dev/null || true
docker compose -f "$base" -f "$override" down

if ((record)); then
  docker run --rm --user 1000:1000 -v "$demo_dir:/demo" --entrypoint python3 \
    uav/swarm-formation:noetic-px4 /demo/encode_video.py /demo/.frames_swarm /demo/$(basename "$intermediate") \
    --fps 3 --title 'Swarm-Formation | official random map | PX4 + Gazebo' \
    --trails /demo/swarm_forest_metrics.json
  "$demo_dir/encode_compatible_mp4.sh" "$intermediate" "$output" 12
  test -s "$output"
  rm -f "$intermediate"
fi
find "$frames" -maxdepth 1 -type f -name 'frame_*.ppm' -delete
rmdir "$frames"
if ((record)); then echo "Demo written to $output"; fi
