# One unfiltered RGB-D measurement; no target identity, velocity or prediction.
uav_inference_interfaces/Detection detection
bool position_valid
# Meters in ObjectDetectionFrame.header.frame_id at the depth capture timestamp.
# Ignore position/depth fields unless position_valid is true.
geometry_msgs/Point position
float64 depth_m
float64 depth_spread_m
uint32 valid_depth_pixels
string position_error
