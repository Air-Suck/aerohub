# PIN-SLAM neural map capability

The PIN-SLAM algorithm and its only upstream checkout live in
`src/basic_function/state_estimator/pin_slam/`. This directory records the Perception-facing
capability of that same process; it does not contain a second algorithm.

`/<namespace>/pin_slam/neural_points` is a `sensor_msgs/PointCloud2` containing
XYZ neural-point anchors in `pin_slam_map`. The timestamp is the LiDAR sample
that updated the map. A nonempty anchor cloud is published after each accepted,
successfully processed LiDAR frame and may be downsampled to a configured maximum point count.

The anchors are useful for visualization and for consumers explicitly designed
for PIN-SLAM's support points. They are not equivalent to the learned implicit
SDF, an occupancy grid, an ESDF, a TSDF, or a collision map. Consumers needing
those representations require an explicit, separately validated converter or
query interface. The full implicit model can be persisted by the algorithm's
save service but is not transported through this PointCloud2 topic.

State and map capabilities share one GPU process and cannot be independently
started, stopped, reset, or upgraded. See the component manifest and capability
cards under `deployment/pin_slam/` and `cards/`.

