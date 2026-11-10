# PIN-SLAM deployment

This deployment runs one PIN-SLAM GPU process and one native AS2 State
Estimator plugin host. It consumes the existing AirSim ROS 2 LiDAR bridge; it
does not add another AirSim RPC reader.

## Build

From the repository root on the Linux development host:

```bash
docker compose -f deployment/pin_slam/compose.yaml build algorithm estimator
docker image inspect uav/pin-slam:humble uav/pin-slam-as2:humble \
  > log/pin_slam/image-inspect.json
```

The current validated AS2 base is `onboard_computer:main-e2e` (revision label
`a9ddf15006a6a36d53fa0316b87a8da590fd7253`). Override
`ONBOARD_COMPUTER_IMAGE` only with an ABI-compatible image built from the same
AirSim/PX4 integration chain.

The algorithm image reuses
`onboard_computer_dependencies:humble-px4-1.18-msgfix`, the PR-provided Humble
and PX4 dependency environment, then installs PyTorch 2.5.1 CUDA 12.4 and the
fully resolved `requirements.lock` snapshot.

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
ROS LiDAR stream to the declared sustainable 3 Hz while preserving the selected
AirSim sample timestamp:

```bash
docker compose \
  -f deployment/flight_board/compose.yaml \
  -f deployment/pin_slam/flight-board.compose.yaml up -d
```

Then start PIN-SLAM with the same ROS domain:

```bash
PIN_SLAM_DOMAIN=81 docker compose -f deployment/pin_slam/compose.yaml up -d
```

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
docker compose \
  -f deployment/onboard_computer/compose.yaml \
  -f deployment/pin_slam/authority.compose.yaml up -d
```

The override runs the full AS2 platform, controller and motion Behavior stack
from `uav/pin-slam-as2:humble`, sources the plugin overlay, and selects
`pin_slam` instead of `raw_odometry`. Before arming, verify that the selected
state estimator is the only publisher of canonical pose, twist and TF.

Do not wire a LiDAR-rate SLAM pose directly to `odom -> base_link`: passing a
shadow accuracy gate does not make a 3 Hz state suitable for a controller with
a 100 ms TF freshness threshold. Measure unique sample stamps, not only topic
message rate. The current AirSim/PX4 chain publishes about 95 local odometry
messages/s but about 42 unique state stamps/s.

Do not select this estimator as flight authority merely because both containers
are healthy. First run the shadow gate described in the validation Skill and
record its threshold and result under `log/pin_slam/`. Stop active motion and
remove state authority before stopping or resetting the shared algorithm.
