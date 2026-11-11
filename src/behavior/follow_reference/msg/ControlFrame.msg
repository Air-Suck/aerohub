# One atomic input frame, FLU body angular rates in rad/s and collective thrust in N.
std_msgs/Header header
string session_id
uint64 revision
uint64 sequence
geometry_msgs/Vector3 body_rates
float64 thrust_newtons
# Last successful solver time, not the publication time of a reused solution.
builtin_interfaces/Time last_successful_solve
bool solver_ok
