# ROG-Map

This directory exposes the locked ROS 2 ROG-Map core as an independent
Perception & Mapping capability. The algorithm source remains single-owned at
`src/planner/super/ros2_adapter/native/rog_map`; GCOPTER's private modified copy
is not used.

The Adapter consumes registered FLOAT32 XYZ points in `drone0/odom` and looks
up the configured LiDAR origin at the original scan time. It maintains raw and
inflated occupancy plus ESDF and publishes:

- `/<namespace>/map/occupied`
- `/<namespace>/map/inflated_occupied`
- `/<namespace>/map/esdf_slice`
- `/<namespace>/map/status`
- `/<namespace>/map/query_voxel`
- `/<namespace>/map/check_line`

The point topics are rolling snapshots for inspection and visualization. The
services are the functional query interface. A nonempty point cloud alone does
not imply that a planner can safely consume the map.

Use the deployment entry and validation procedure in
`deployment/rog_map/README.md`. Do not build an algorithm-specific Docker
image; build the packages into an external overlay using the fixed shared
`onboard_computer:v0.1.0` environment.
