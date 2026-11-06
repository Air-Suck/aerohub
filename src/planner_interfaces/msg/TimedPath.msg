# External trajectory in a fixed, vehicle-local ENU frame. Never use a moving TF.
uint32 interface_version
string task_id
string vehicle_id
uint64 revision
string frame_id
builtin_interfaces/Time generated_at
builtin_interfaces/Time valid_from
bool is_final
# VELOCITY=1, ACCELERATION=2, YAW=4 (same as VehicleTrajectory).
uint8 valid_fields
uav_planner_interfaces/TimedTrajectoryPoint[] points
float64 max_speed
