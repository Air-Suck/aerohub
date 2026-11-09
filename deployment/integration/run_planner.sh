#!/usr/bin/env bash
set -euo pipefail

planner="${1:-}"
case "$planner" in
  cat_ora)
    task_config=/opt/uav_planner_ws/install/share/uav_planner_framework/examples/cat_ora_four_px4.yaml
    deployment_config=/opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_four_px4.yaml
    timeout_seconds=240
    ;;
  swarm_formation)
    task_config=/opt/uav_planner_ws/install/share/uav_planner_framework/examples/swarm_formation_seven_px4.yaml
    deployment_config=/opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_seven_px4.yaml
    timeout_seconds=300
    ;;
  *)
    echo "Usage: $0 {cat_ora|swarm_formation}" >&2
    exit 2
    ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "$script_dir/../.." && pwd)"
compose_file="$script_dir/compose.yaml"
cd "$project_dir"

ros_setup='source /opt/ros/humble/setup.bash; source /root/aerostack2_ws/install/setup.bash; source /opt/uav_planner_ws/install/setup.bash'
ros_exec() {
  docker exec planner-runtime bash -lc "$ros_setup; $*"
}

dump_logs() {
  docker compose -f "$compose_file" ps || true
  for container in planner-runtime planner-as2-seven planner-px4-seven \
    planner-cat-ora planner-swarm-formation planner-clock; do
    echo "===== $container =====" >&2
    docker logs --tail 80 "$container" 2>&1 || true
  done
}
trap 'rc=$?; if ((rc != 0)); then dump_logs; fi' EXIT

docker compose -f "$compose_file" down --remove-orphans
docker compose -f "$compose_file" up -d --no-build

deadline=$((SECONDS + 180))
until ros_exec 'ros2 service type /planner/control 2>/dev/null' \
    | grep -qx 'uav_planner_interfaces/srv/ControlPlanner'; do
  if ((SECONDS >= deadline)); then
    echo "planner/control did not become ready" >&2
    exit 1
  fi
  sleep 2
done

ros_exec 'fleet_takeoff.py --vehicles drone0 drone1 drone2 drone3 drone4 drone5 drone6 --height 1.5 --speed 0.5 --timeout 90'

start_output="$(ros_exec "ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner '{command: 0, task_config_path: $task_config, deployment_config_path: $deployment_config}'")"
echo "$start_output"
grep -q 'accepted=True' <<<"$start_output"

deadline=$((SECONDS + timeout_seconds))
while ((SECONDS < deadline)); do
  status="$(ros_exec "ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner '{command: 4}'")"
  if grep -q 'overall_status=3' <<<"$status" && grep -q "failure_reason='NONE'" <<<"$status"; then
    echo "$status"
    echo "PASS: $planner completed with SUCCEEDED/NONE"
    exit 0
  fi
  if grep -Eq 'overall_status=(4|5)' <<<"$status"; then
    echo "$status" >&2
    exit 1
  fi
  sleep 2
done

echo "$planner did not finish within ${timeout_seconds}s" >&2
exit 1
