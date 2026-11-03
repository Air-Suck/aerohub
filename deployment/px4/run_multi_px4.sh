#!/usr/bin/env bash
set -euo pipefail

px4_dir=${PX4_DIR:-/workspace/PX4-Autopilot}
build_dir="$px4_dir/build/px4_sitl_default"
vehicle_count=${VEHICLE_COUNT:-4}
instance_base=${PX4_INSTANCE_BASE:-10}
xrce_agent_port=${XRCE_AGENT_PORT:-8889}
gz_partition=${PX4_GZ_PARTITION:-as2_multi_domain_${ROS_DOMAIN_ID:-0}}
log_dir=${PX4_MULTI_LOG_DIR:-/tmp/as2-px4-multi}
poses=(
  "-12.0,-4.5,0.2" "-12.0,-1.5,0.2"
  "-12.0,1.5,0.2"  "-12.0,4.5,0.2"
  "-15.0,-3.0,0.2" "-15.0,0.0,0.2" "-15.0,3.0,0.2"
)

if ((vehicle_count < 1 || vehicle_count > ${#poses[@]})); then
  echo "VEHICLE_COUNT must be between 1 and ${#poses[@]}" >&2
  exit 2
fi
[[ -x "$build_dir/bin/px4" ]] || { echo "PX4 SITL is not built at $build_dir" >&2; exit 2; }

mkdir -p "$log_dir"
pids=()
cleanup() {
  trap - EXIT INT TERM
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

for ((i = 0; i < vehicle_count; ++i)); do
  instance=$((instance_base + i))
  working_dir="$log_dir/instance_$instance"
  mkdir -p "$working_dir"
  standalone=()
  ((i > 0)) && standalone=(PX4_GZ_STANDALONE=1)
  (
    cd "$working_dir"
    set +u
    source "$build_dir/rootfs/gz_env.sh"
    set -u
    exec env "${standalone[@]}" \
      PX4_SYS_AUTOSTART=4001 \
      PX4_SIM_MODEL=gz_x500 \
      PX4_GZ_MODEL_POSE="${poses[$i]}" \
      PX4_PARAM_SDLOG_MODE=0 \
      PX4_UXRCE_DDS_PORT="$xrce_agent_port" \
      GZ_PARTITION="$gz_partition" \
      GZ_IP=127.0.0.1 \
      "$build_dir/bin/px4" -i "$instance" -d "$build_dir/etc"
  ) >"$log_dir/px4_$i.log" 2>&1 &
  pids+=("$!")
  ((i == 0)) && sleep 8 || sleep 1
  kill -0 "${pids[$i]}" 2>/dev/null || { echo "PX4 $i failed" >&2; exit 1; }
done

# These parameters are set after rcS completes because the airframe startup
# scripts overwrite environment-based parameter values during initialization.
sleep 8
for ((i = 0; i < vehicle_count; ++i)); do
  instance=$((instance_base + i))
  "$build_dir/bin/px4-param" --instance "$instance" set NAV_DLL_ACT 0
  "$build_dir/bin/px4-param" --instance "$instance" set COM_DISARM_PRFLT 60
done

(
  set +u
  source "$build_dir/rootfs/gz_env.sh"
  set -u
  export GZ_PARTITION="$gz_partition" GZ_IP=127.0.0.1
  exec python3 /integration/deployment/px4/gazebo_clock_proxy.py
) >"$log_dir/gazebo_clock_proxy.log" 2>&1 &
pids+=("$!")

echo "Started $vehicle_count PX4/Gazebo vehicles"
wait -n "${pids[@]}"
echo "A PX4 instance or the Gazebo clock proxy exited unexpectedly" >&2
exit 1
