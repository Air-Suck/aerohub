# Published by the FollowReference plugin; the external algorithm echoes session/revision.
std_msgs/Header header
string session_id
uint64 revision
bool active
geometry_msgs/PointStamped target
as2_msgs/YawMode yaw
# Component bounds in target.header.frame_id; algorithm must acknowledge all fields.
geometry_msgs/Vector3 max_speed

# Immutable deployment-selected command mode, echoed by every control frame.
uint8 control_mode
