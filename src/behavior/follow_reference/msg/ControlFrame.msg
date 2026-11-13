# One atomic control sample; mode selects body rates or desired attitude.
# Shared header: computation stamp and FLU body frame. Thrust in N.
std_msgs/Header header
string session_id
uint64 revision
uint64 sequence
geometry_msgs/Vector3 body_rates
float64 thrust_newtons
# Last successful solver time, not the publication time of a reused solution.
builtin_interfaces/Time last_successful_solve
bool solver_ok

# Explicit command kind. Zero preserves existing BODY_RATES clients.
uint8 BODY_RATES=0
uint8 ATTITUDE=1
uint8 control_mode
# Desired FLU body orientation in attitude_frame_id (fixed ENU planning frame).
# Used only in ATTITUDE mode; unused fields do not affect execution.
string attitude_frame_id
geometry_msgs/Quaternion attitude
