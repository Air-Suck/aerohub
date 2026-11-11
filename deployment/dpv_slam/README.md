# DPV-SLAM deployment

This deployment integrates the official DPV-SLAM configuration into AeroHub
without modifying AS2 or the upstream algorithm. One GPU process consumes the
existing AirSim RGB/CameraInfo/local-odometry topics and exposes visual state
and sparse landmarks. A native AS2 State Estimator plugin is the sole owner of
canonical pose, twist and TF when authority mode is explicitly selected.

## Prepare and build

Run from the repository root on the Linux development host. Downloads must use
the host's configured proxy when required; no proxy endpoint belongs in the
repository.

```bash
bash deployment/dpv_slam/prepare_models.sh
docker compose -f deployment/dpv_slam/compose.yaml build algorithm estimator
docker image inspect uav/dpv-slam:humble uav/dpv-slam-as2:humble \
  > log/dpv_slam/image-inspect.json
```

`prepare_models.sh` verifies both the official archive and extracted
`dpvo.pth`, then prepares a resumable, hash-verified local wheelhouse. The
algorithm build uses `DPV_SLAM_PYTORCH_IMAGE` as a donor for the fixed
PyTorch 2.5.1/CUDA 12.4 Python installation; the current validated donor is
`uav/pin-slam:humble` image ID `6be2f8b0b5e7`. Only its generic `/usr/local`
installation is copied, not PIN-SLAM source, configuration or entrypoints.
The image then compiles the three official CUDA extensions for compute
capability 8.9 and installs the Humble adapter. The DPV-SLAM base
configuration is unchanged except that the selected paper mode is forced to
`LOOP_CLOSURE=True` and `CLASSIC_LOOP_CLOSURE=False`.

The flight-board image must contain the current sensor bridge and supervisor,
including `FLIGHT_BOARD_CAMERA_RATE_HZ`; rebuild it with its documented build
entry if the fixed image predates this integration.

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

DPV_SLAM_DOMAIN=82 docker compose -f deployment/dpv_slam/compose.yaml up -d
```

Before movement, verify that the actual image is nonempty, the CameraInfo stamp
matches, `/clock` advances, GPU inference is active, and only the existing
validated provider owns canonical state. Run the predeclared matrix in
`validation.yaml`: real GPU algorithm, real AirSim input, DPV-SLAM output,
independent sparse-point subscriber, native plugin, truth-scored shadow flight,
then cleanup. Save source/dirty state, image IDs, ROS graph, unique stamps,
metrics, visible screenshot, landing/disarm result and checksums under
`log/dpv_slam/<run>/`.

Do not enable authority unless every shadow gate passes in the same run. Native
monocular output is scale-ambiguous; successful image processing or a
scale-aligned trajectory plot is not sufficient evidence.

The candidate gap gate applies to low-rate DPV-SLAM global corrections. It is
not a controller freshness allowance. In authority mode, PX4 local odometry
must keep canonical pose, twist and `odom -> base_link` gaps below 0.1 seconds
while the plugin holds the most recent accepted `map -> odom` correction.

## Official TUM sequence validation

The reproduced official-data gate uses TUM RGB-D `freiburg1_xyz`, the official
`dpvo.pth`, all 798 RGB frames and a real CUDA GPU. Extract the sequence under
an ignored data directory and run:

```bash
mkdir -p log/dpv_slam/official-tum/evaluation
docker run --rm --gpus all --ipc=host \
  -v "$PWD/deployment/dpv_slam/evaluate_tum_sequence.py:/opt/evaluate_tum_sequence.py:ro" \
  -v "$PWD/log/dpv_slam/datasets/TUM_RGBD:/data:ro" \
  -v "$PWD/log/dpv_slam/official-tum/evaluation:/evidence" \
  uav/dpv-slam:humble python3 /opt/evaluate_tum_sequence.py \
  --sequence /data/rgbd_dataset_freiburg1_xyz \
  --network /opt/dpv_slam/dpvo.pth \
  --config /opt/dpv_slam/default.yaml \
  --output /evidence --max-rmse 0.05
```

The evaluator forces `LOOP_CLOSURE=True` and
`CLASSIC_LOOP_CLOSURE=False`, associates output to ground truth within 20 ms,
and performs a monocular Sim(3) alignment. The validated run processed 798
frames on an RTX 4090, associated 796 poses and obtained 0.01043 m translation
ATE RMSE against the fixed 0.05 m gate. `result.json`, aligned trajectories,
image metadata and the dataset checksum form the evidence; a scale-aligned
dataset result does not replace the AirSim authority and failure gates.

## Authority and failure tests

After the machine-readable shadow report passes, start the full onboard stack
with `uav/dpv-slam-as2:humble` and select `plugin_name=dpv_slam`. The plugin
uses visual state only for `map -> odom`; PX4 local odometry continues to
provide high-rate `odom -> base_link` and body twist. Verify unique canonical
stamps and TF freshness before Takeoff/GoTo/Land.

Inject at least RGB interruption and algorithm-process exit in isolated runs.
Expected behavior is: no stale visual correction is accepted, fresh local
state remains available, the mission performs a controlled land and disarm,
and all owned containers/processes exit without residual publishers. This does
not cover failure of the PX4 local estimator itself.

Stop in ownership order after landing: remove DPV-SLAM authority, stop the
algorithm/plugin containers, stop the flight-board containers, then stop the
owned AirSim instance. Never stop unrelated simulator or ROS deployments.

## Representation boundary

`dpv_slam/sparse_points` is a downsampled patch-landmark cloud. It is not an
occupancy map, dense reconstruction, ESDF, mesh, or proof of collision-free
space. Its capability card intentionally does not advertise navigation-map
support.
