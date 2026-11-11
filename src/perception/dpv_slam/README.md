# DPV-SLAM sparse visual map

The single DPV-SLAM process owned by `src/state_estimator/dpv_slam/` also
publishes metric-aligned patch landmarks on
`/<namespace>/dpv_slam/sparse_points` as `sensor_msgs/msg/PointCloud2`.

The frame is `dpv_slam_map`, the timestamp is the RGB sample that produced the
update, and XYZ values use the frozen metric alignment shared with the pose
output. Publication is capped by `map_max_points` through deterministic
striding. The points are sparse learned patch landmarks. They do not encode
free space, occupancy, a mesh, semantic labels, or collision distance, so a
navigation consumer must add and validate an explicit map conversion before
using them for obstacle avoidance.
