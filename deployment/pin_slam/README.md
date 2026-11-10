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

Start visible AirSim (`DISPLAY=:3`, `AIRSIM_RENDER_MODE=windowed`), PX4 SITL,
and the existing flight-board sensor bridge first. With the same ROS domain:

```bash
PIN_SLAM_DOMAIN=81 docker compose -f deployment/pin_slam/compose.yaml up -d
```

The algorithm subscribes to
`/drone0/sensor_measurements/Lidar/points`. The configured AirSim frame is
`drone0/Lidar`, range is 25 m, and per-point time is unavailable, so deskew is
disabled. The plugin consumes `/drone0/pin_slam/odometry`; only the AS2 host
owns canonical `self_localization` and TF output.

Do not select this estimator as flight authority merely because both containers
are healthy. First run the shadow gate described in the validation Skill and
record its threshold and result under `log/pin_slam/`. Stop active motion and
remove state authority before stopping or resetting the shared algorithm.
