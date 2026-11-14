# PIN-SLAM integration

This component keeps the upstream PIN-SLAM process intact as one LiDAR SLAM
provider while exposing two distinct AeroHub capabilities:

- pose and body twist through an AS2 State Estimator plugin;
- neural-point anchors through a ROS 2 Perception topic.

The upstream source is locked at
`e2b7fbf328e5fac1d638e217d70a69cbe9f30a52` in `upstream/` and remains MIT
licensed. Project-owned code is split into:

| Path | Responsibility |
| --- | --- |
| `ros2_adapter/` | Run the upstream algorithm from ROS 2 Humble, preserve LiDAR sample time, derive finite-difference twist, and publish standard ROS messages |
| `plugin/` | Validate normalized odometry and submit pose/twist/TF to the native AS2 State Estimator host |
| `../../perception/pin_slam/` | Document the neural-point map interface without copying the algorithm |
| `../../../../deployment/pin_slam/` | Fixed shared-image workspace preparation, configuration, launch, and component manifest |

## Runtime interfaces

With namespace `drone0`, the algorithm subscribes to
`sensor_measurements/Lidar/points` (`sensor_msgs/PointCloud2`) and requires the
frame `drone0/Lidar`. AirSim supplies an instantaneous XYZ frame with no
per-point time. The configuration therefore disables deskewing; the adapter
does not invent ring, intensity, or point timestamps.

The algorithm publishes:

- `pin_slam/odometry` (`nav_msgs/Odometry`), frame `pin_slam_map`, child
  `pin_slam_lidar`; publication starts only after two valid samples permit a
  finite-difference twist;
- `pin_slam/path` (`nav_msgs/Path`), for diagnostics rather than control;
- `pin_slam/neural_points` (`sensor_msgs/PointCloud2`), a downsampled set of
  neural-point anchors in `pin_slam_map`;
- `pin_slam/status` (`diagnostic_msgs/DiagnosticArray`).

The point cloud is not an occupancy grid, ESDF, mesh, or complete serialized
implicit model. Full model and mesh persistence remains an algorithm service
and filesystem concern.

## State and reset semantics

PIN-SLAM estimates the LiDAR sensor pose. The plugin applies the configured
rigid `sensor_from_base` transform before publishing the AS2 base pose and
accounts for the lever arm in twist conversion. Identity values are only valid
for the co-located AirSim example and are not hardware calibration.

Loop closure may correct the global pose and neural map. The current adapter
does not expose a separate smooth local odometry chart, so consumers must treat
the output as a globally corrected local map frame that can jump at loop
closure. Restarting the process creates a new epoch; stale or non-monotonic
samples are rejected and hot switching of the authoritative estimator is not
supported.

## Build and validation

Workspace preparation commands and the exact shared-image input are documented in
`deployment/pin_slam/README.md`. Passing a build, import test, or synthetic
message test is not AirSim localization acceptance. AirSim validation must use
the real platform LiDAR topic, prove downstream consumption, compare shadow
pose against simulator truth, and pass the configured gate before authority.

