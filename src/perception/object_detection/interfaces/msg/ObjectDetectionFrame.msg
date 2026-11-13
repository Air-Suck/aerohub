# Position frame and depth acquisition timestamp, not publication time.
std_msgs/Header header
std_msgs/Header image_header
std_msgs/Header depth_header
string request_id
uint64 generation
uint64 tool_generation
uint64 worker_generation
string model_version
ObjectDetection[] detections
# False on missing calibration/depth/TF or any invalid requested position.
# Empty detections with valid=true is an observed no-detection result.
bool valid
string error
string timings_json
