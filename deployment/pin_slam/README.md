# PIN-SLAM deployment

This deployment runs one PIN-SLAM GPU process and one native AS2 State
Estimator plugin host. It consumes the existing AirSim ROS 2 LiDAR bridge; it
does not add another AirSim RPC reader.

## Prepare the shared workspace

From the repository root on the Linux development host:

PIN-SLAM does not build an algorithm-specific Docker image. Both the algorithm
and the AS2 plugin use the fixed `onboard_computer:v0.1.0` environment. Python,
CUDA-wheel and colcon outputs live in the ignored host workspace
`log/workspaces/pin_slam/` and are mounted into the shared container:

```bash
export HTTP_PROXY=<the development host proxy>
export HTTPS_PROXY="$HTTP_PROXY"
bash deployment/pin_slam/download_wheels.sh
bash deployment/pin_slam/prepare_workspace.sh
PX4_MSGS_OVERLAY_WORKSPACE="$PWD/log/workspaces/px4_msgs-61907962" \
  bash deployment/onboard_computer/px4_msgs/prepare_overlay.sh

AEROHUB_ROOT="$PWD" \
PIN_SLAM_WORKSPACE="$PWD/log/workspaces/pin_slam" \
  docker compose -f deployment/pin_slam/compose.yaml config --quiet
```

`download_wheels.sh` is the only networked preparation step. It uses the
caller's proxy environment, downloads the complete CPython 3.10/Linux x86_64
wheel set, and verifies every artifact against `requirements.lock` or
`bootstrap-wheels.sha256`. `prepare_workspace.sh` then works offline: it
verifies the immutable shared image, bootstraps a host-persistent Python
environment from the checked wheelhouse, installs `requirements.lock`, and
builds `uav_pin_slam_ros2` plus
`uav_pin_slam` into one ROS overlay. It records the source, upstream commit,
shared image ID, resolved Python packages and build logs in that workspace.
It never invokes `docker build` and never creates a new image or tag.

Set the host and Docker daemon proxy before the build when required. Do not
store a personal proxy endpoint in this repository.

## Runtime

Start visible AirSim (`DISPLAY=:3`, `AIRSIM_RENDER_MODE=windowed`) with the
PIN-SLAM profile. The profile preserves the platform LiDAR model at 10
rotations/s and 80000 points/s:

```bash
DISPLAY=:3 AIRSIM_RENDER_MODE=windowed \
AIRSIM_PROFILE=deployment/pin_slam/airsim-profile.json \
AIRSIM_RUN_DIR=log/pin_slam/airsim-scene \
  simulator/airsim/airsim.sh start
```

Start PX4 SITL and the sensor bridge with the matching override. It limits the
ROS LiDAR stream to the declared 4 Hz acceptance input while preserving the
selected AirSim sample timestamp:

```bash
docker compose \
  -f deployment/flight_board/compose.yaml \
  -f deployment/pin_slam/flight-board.compose.yaml up -d
```

Then start PIN-SLAM with the same ROS domain and the prepared workspace:

```bash
AEROHUB_ROOT="$PWD" \
PIN_SLAM_WORKSPACE="$PWD/log/workspaces/pin_slam" \
PIN_SLAM_DOMAIN=81 \
  docker compose -f deployment/pin_slam/compose.yaml up -d --no-build algorithm
```

The standalone `estimator` service is assigned to the explicit
`plugin-smoke` profile. It is only for isolated plugin checks and must not run
beside the normal onboard estimator during shadow validation. Shadow scoring
uses the raw `/drone0/pin_slam/odometry` candidate while the previously
validated provider remains the sole canonical state and TF owner.

The algorithm subscribes to
`/drone0/sensor_measurements/Lidar/points`. The configured AirSim frame is
`drone0/Lidar`, range is 25 m, and per-point time is unavailable, so deskew is
disabled. The plugin consumes both `/drone0/pin_slam/odometry` and the
platform's `/drone0/sensor_measurements/odom`. PIN-SLAM supplies the low-rate
`map -> odom` global correction; PX4 local odometry supplies high-rate
`odom -> base_link` and body twist. The latest global correction is held at
each new local sample time so the complete TF chain remains queryable by the
controller. Only the AS2 host owns canonical `self_localization` and TF
output.

The reproducible shadow runner is `deployment/pin_slam/shadow_validation.py`.
It records every observed LiDAR input stamp and pairs candidate output by the
same preserved stamp. Coverage is the number of paired candidate stamps divided
by the observed LiDAR stamps between the first and last candidate output. Truth
is sampled for each input from the same AirSim clock and pairs exceeding 50 ms
are excluded from accuracy scoring and reported as a gate failure.
Run `airsim_truth_bridge.py` in a ROS Humble environment with the repository
`simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py` available as
`airsim_rpc`. It publishes only
`/pin_slam/validation/airsim_truth`; verify that no control component
subscribes to this scoring topic.

The validator subscribes to versioned PX4 status, command ACK and failsafe
diagnostics. Mount the prepared PX4 message workspace read-only at
`/px4-overlay`, then source `/px4-overlay/install/setup.bash` after
`/opt/px4_ws/install/setup.bash`. The fixed flight-board PX4 remains the
runtime firmware. In this no-RC topology, establish and confirm Offboard mode
before requiring `pre_flight_checks_pass` and arming; the validator enforces
that order and still requires a fresh accepted ACK plus actual firmware state.

Evaluate its result against the versioned thresholds with:

```bash
python3 deployment/pin_slam/gate_result.py \
  deployment/pin_slam/validation.yaml \
  log/pin_slam/<run>/final-run/result.json \
  --output log/pin_slam/<run>/final-run/gate-report.json
```

After every shadow gate passes in one run, replace the normal onboard service
with the PIN-SLAM authority override. Do not run the normal onboard Compose at
the same time:

```bash
PIN_SLAM_WORKSPACE="$PWD/log/workspaces/pin_slam" docker compose \
  -f deployment/onboard_computer/compose.yaml \
  -f deployment/pin_slam/compose.yaml \
  -f deployment/pin_slam/authority.compose.yaml \
  up -d --no-build algorithm as2
```

The override runs the full AS2 platform, controller and motion Behavior stack
from the fixed shared image, sources `/workspace/ros/install`, and selects
`pin_slam` instead of `raw_odometry`. Before arming, verify that the selected
state estimator is the only publisher of canonical pose, twist and TF.

Start the dedicated canonical-state freshness probe before arming and stop it
only after landing:

```bash
python3 deployment/state_estimator/canonical_freshness.py \
  --namespace drone0 --output <canonical-freshness.json>

python3 deployment/pin_slam/gate_result.py \
  deployment/pin_slam/validation.yaml <result.json> --authority \
  --authority-freshness <canonical-freshness.json> \
  --output <authority-gate-report.json>
```

The authority report must cover a closed airborne window and require monotonic
canonical source stamps plus source-stamp and local-arrival maximum gaps no
greater than 0.1 seconds. Inject LiDAR interruption and algorithm-process exit
in separate runs; stale global corrections must be rejected while fresh local
state remains available for controlled landing and disarm.

Do not wire a LiDAR-rate SLAM pose directly to `odom -> base_link`: passing a
shadow accuracy gate does not make a low-rate global state suitable for a
controller with a 100 ms TF freshness threshold. Measure unique sample stamps,
not only topic message rate. The current AirSim/PX4 chain publishes about 95
local odometry messages/s but about 42 unique state stamps/s.

Do not select this estimator as flight authority merely because both containers
are healthy. First run the shadow gate described in the validation Skill and
record its threshold and result under `log/pin_slam/`. Stop active motion and
remove state authority before stopping or resetting the shared algorithm.
