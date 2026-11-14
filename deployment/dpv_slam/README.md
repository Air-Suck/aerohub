# DPV-SLAM deployment

This deployment integrates the official DPV-SLAM configuration into AeroHub
without modifying AS2 or the upstream algorithm. One GPU process consumes the
existing AirSim RGB/CameraInfo/local-odometry topics and exposes visual state
and sparse landmarks. A native AS2 State Estimator plugin is the sole owner of
canonical pose, twist and TF when authority mode is explicitly selected.

## Prepare and build

DPVO is the official, unmodified Git submodule at
`src/basic_function/state_estimator/dpv_slam/upstream/`. The ROS 2 adapter, AS2 plugin and
sparse-point interface are maintained in AeroHub; no upstream Fork or
build-time patch is used.

Development and validation reuse `flight_board:v0.1.0` and
`onboard_computer:v0.1.0`. Prepare DPVO's locked Python/CUDA dependencies and
the ROS overlays in an ignored, host-mounted external workspace:

```bash
bash deployment/dpv_slam/prepare_models.sh
deployment/dpv_slam/prepare_workspace.sh
export DPV_SLAM_WORKSPACE="$PWD/log/workspaces/dpv_slam"
export PX4_MSGS_OVERLAY_WORKSPACE="$PWD/log/workspaces/px4_msgs-61907962"
export AS2_EXECUTION_WORKSPACE="$PWD/log/workspaces/as2_execution"
PX4_MSGS_OVERLAY_WORKSPACE="$PX4_MSGS_OVERLAY_WORKSPACE" \
  bash deployment/onboard_computer/px4_msgs/prepare_overlay.sh
PX4_MSGS_OVERLAY_WORKSPACE="$PX4_MSGS_OVERLAY_WORKSPACE" \
AS2_EXECUTION_WORKSPACE="$AS2_EXECUTION_WORKSPACE" \
  bash deployment/onboard_computer/prepare_execution_overlay.sh
```

`python-wheels.sha256` locks every Python/CUDA runtime wheel and the pip/ninja
build tools by URL and SHA256. The preparation script verifies the official
DPVO gitlink and clean submodule, downloads large wheels in resumable verified
ranges, and builds the three CUDA extensions directly from the read-only
submodule without installing untracked system packages. It rejects a Python
distribution set that differs from the wheel manifest, then builds the Adapter
and plugin overlays. Load those overlays after the shared onboard baseline. Do
not build algorithm-specific images. The historical Dockerfiles and Compose
entries were removed; `shared.compose.yaml` and
`authority.shared.compose.yaml` are the only supported DPV-SLAM runtime
entries.

The current shared-environment matrix passes shadow, authority, RGB-loss and
algorithm-exit scenarios. The component is reproduced for the locked AirSim +
PX4 SITL scope described below; this does not claim real-flight validation.

## Runtime model

The four project roles are:

- simulator: visible AirSimNH on the host;
- flight board: PX4 SITL plus clock, RGB, CameraInfo and local odometry bridges;
- compute: DPV-SLAM GPU container and the AS2 plugin/flight stack;
- workstation: mission requests, shadow scoring and evidence collection.

The algorithm subscribes to:

```text
/drone0/sensor_measurements/FrontCamera/rgb/image
/drone0/sensor_measurements/FrontCamera/rgb/camera_info
/drone0/sensor_measurements/odom
```

It publishes `/drone0/dpv_slam/odometry`, `/path`, `/sparse_points` and
`/status`. RGB and CameraInfo must have the same preserved AirSim sample stamp.
Local odometry is paired within 50 ms. The first 30 initialized visual poses
are fitted once to local metric positions using Sim(3); the fit requires at
least 1 m baseline and at most 0.35 m RMSE. The frozen transform prevents a
visual correction from being silently refitted to local odometry on every
frame. AirSim truth is used only by validation.

The DPV-SLAM AirSim profile uses a 320x240 RGB image at 90 degrees horizontal
FOV and requests 15 Hz from the shared flight-board bridge. This is the locked
input for this model and acceptance matrix, not a default camera profile for
other capabilities. The validator still requires at least 8 Hz of unique image
sample stamps; lowering that gate after a slow run is not an accepted fix.

## Visible AirSim and shadow validation

Use the required visible mode:

```bash
DISPLAY=:3 AIRSIM_RENDER_MODE=windowed \
AIRSIM_PROFILE=deployment/dpv_slam/airsim-profile.json \
AIRSIM_RUN_DIR=log/dpv_slam/airsim-scene \
  simulator/airsim/airsim.sh start

DPV_SLAM_DOMAIN=82 docker compose \
  -f deployment/flight_board/compose.yaml \
  -f deployment/dpv_slam/flight-board.compose.yaml up -d

DPV_SLAM_WORKSPACE="$PWD/log/workspaces/dpv_slam" \
DPV_SLAM_DOMAIN=82 docker compose \
  -f deployment/dpv_slam/shared.compose.yaml up -d --no-build algorithm
```

The `estimator` service is behind the explicit `plugin-smoke` profile and is
not part of shadow startup. Running it beside the normal onboard estimator
would create competing canonical state and TF publishers. Shadow validation
scores `/drone0/dpv_slam/odometry`; authority mode loads the plugin in the
single onboard AS2 host through `authority.shared.compose.yaml`.

Before movement, verify that the actual image is nonempty, the CameraInfo stamp
matches, `/clock` advances, GPU inference is active, and only the existing
validated provider owns canonical state. Run the predeclared matrix in
`validation.yaml`: real GPU algorithm, real AirSim input, DPV-SLAM output,
independent sparse-point subscriber, native plugin, truth-scored shadow flight,
then cleanup. Save source/dirty state, image IDs, ROS graph, unique stamps,
metrics, visible screenshot, landing/disarm result and checksums under
`log/dpv_slam/<run>/`.

The shadow/authority validator additionally mounts the prepared PX4 message
workspace at `/px4-overlay` and sources its install after
`/opt/px4_ws/install/setup.bash`. This supplies the fixed flight-board
firmware's versioned status and failsafe schemas without rebuilding the shared
image. The no-RC startup order is Offboard confirmation, mode-specific
preflight confirmation, then arming with a fresh accepted ACK.

Do not enable authority unless every shadow gate passes in the same run. Native
monocular output is scale-ambiguous; successful image processing or a
scale-aligned trajectory plot is not sufficient evidence.

The candidate gap gate applies to low-rate DPV-SLAM global corrections. It is
not a controller freshness allowance. In authority mode, PX4 local odometry
must keep canonical pose, twist and `odom -> base_link` gaps below 0.1 seconds
while the plugin holds the most recent accepted `map -> odom` correction.
Gate schema 0.2 allows at most 2.5 seconds between loop-closing global
corrections and retains the separate 0.1-second canonical-state gate. Adapter
diagnostics carry the processed image stamp, so coverage is measured at the
algorithm boundary rather than inferred from a second image subscriber.

Run the isolated matrix through the owned entry point:

```bash
export DISPLAY=:3 AIRSIM_RENDER_MODE=windowed
python3 deployment/dpv_slam/run_airsim.py run --scenario shadow --output log/dpv_slam/shadow
python3 deployment/dpv_slam/run_airsim.py run --scenario authority --output log/dpv_slam/authority
python3 deployment/dpv_slam/run_airsim.py run --scenario sensor_fault --output log/dpv_slam/sensor-fault
python3 deployment/dpv_slam/run_airsim.py run --scenario algorithm_fault --output log/dpv_slam/algorithm-fault
```

## Official TUM sequence validation

The historical official-data gate used TUM RGB-D `freiburg1_xyz`, the official
`dpvo.pth`, all 798 RGB frames and a real CUDA GPU. The validated run processed
798 frames, associated 796 poses and obtained 0.01043 m translation ATE RMSE
against the fixed 0.05 m gate. That evidence predates the shared-workspace
migration and cannot be rerun through the removed dedicated image.

Future official-data revalidation must run `evaluate_tum_sequence.py` in
`onboard_computer:v0.1.0` with `DPV_SLAM_WORKSPACE` mounted read-only and the
same Python/CUDA paths used by `shared.compose.yaml`. Record the exact command
and output with the new evidence rather than restoring a dedicated image.

The evaluator forces `LOOP_CLOSURE=True` and
`CLASSIC_LOOP_CLOSURE=False`, associates output to ground truth within 20 ms,
and performs a monocular Sim(3) alignment. `result.json`, aligned trajectories,
image metadata and the dataset checksum form the evidence; a scale-aligned
dataset result does not replace the AirSim authority and failure gates.

## Authority and failure tests

After the machine-readable shadow report passes, start the full onboard stack
from an external overlay in `onboard_computer:v0.1.0` and select
`plugin_name=dpv_slam`. The plugin
uses visual state only for `map -> odom`; PX4 local odometry continues to
provide high-rate `odom -> base_link` and body twist. Verify unique canonical
stamps and TF freshness before Takeoff/GoTo/Land.

```bash
DPV_SLAM_WORKSPACE="$DPV_SLAM_WORKSPACE" \
PX4_MSGS_OVERLAY_WORKSPACE="$PX4_MSGS_OVERLAY_WORKSPACE" \
AS2_EXECUTION_WORKSPACE="$AS2_EXECUTION_WORKSPACE" \
DPV_SLAM_DOMAIN=82 ROS_DOMAIN_ID=82 docker compose \
  -f deployment/onboard_computer/compose.yaml \
  -f deployment/dpv_slam/shared.compose.yaml \
  -f deployment/dpv_slam/authority.shared.compose.yaml \
  up -d algorithm as2
```

Evaluate an authority run with:

```bash
# Start before arming in a separate lightweight process and stop after landing.
python3 deployment/state_estimator/canonical_freshness.py \
  --namespace drone0 --output <canonical-freshness.json>

python3 deployment/dpv_slam/gate_result.py \
  deployment/dpv_slam/validation.yaml <result.json> --authority \
  --authority-freshness <canonical-freshness.json> \
  --output <authority-gate-report.json>
```

The authority report must pass the normal shadow gates and contain at least
two canonical samples from a closed FLYING window whose source stamps are
monotonic and whose source-stamp and local-arrival maximum gaps are both at
most 0.1 seconds. The dedicated probe intentionally does not subscribe to RGB,
point clouds or truth, so recorder load cannot hide a freshness failure. The
2026-11-11 authority evidence failed the earlier rule (`1.197 s` canonical gap
and `1.695 s > 1.5 s` candidate gap) and remains historical failed evidence.
The 2026-11-14 schema 0.2 authority run passed with 0.024 s source and 0.033 s
arrival gaps, 0.162 m position RMSE and a 2.247 s global-correction gap. The
paired shadow run passed with 0.178 m RMSE and full input coverage.

Inject at least RGB interruption and algorithm-process exit in isolated runs.
Expected behavior is: no stale visual correction is accepted, fresh local
state remains available, the mission performs a controlled land and disarm,
and all owned containers/processes exit without residual publishers. This does
not cover failure of the PX4 local estimator itself.

Fault validation first flies an observable square until DPV-SLAM has a valid
metric alignment. It then stops the owned sensor or algorithm container,
requires the watched stream to remain stale for two seconds, verifies at least
20 fresh canonical samples during the hold, and invokes native Land. The
2026-11-14 final runs detected both faults in about 2.1 seconds, preserved 50
canonical samples during the hold, and landed/disarmed cleanly.

Stop in ownership order after landing: remove DPV-SLAM authority, stop the
algorithm/plugin containers, stop the flight-board containers, then stop the
owned AirSim instance. Never stop unrelated simulator or ROS deployments.

## Representation boundary

`dpv_slam/sparse_points` is a downsampled patch-landmark cloud. It is not an
occupancy map, dense reconstruction, ESDF, mesh, or proof of collision-free
space. Its capability card intentionally does not advertise navigation-map
support.
