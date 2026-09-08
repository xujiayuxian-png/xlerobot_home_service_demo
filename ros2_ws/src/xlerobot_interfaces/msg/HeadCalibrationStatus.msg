# Durable, observational status for one local head-camera calibration session.
std_msgs/Header header
string unit_id
bool running
# IDLE, MOVING, WAITING, CAPTURING, SOLVING, PAUSED, COMPLETED, ERROR
string phase
string message
int32 pose_index
uint32 pose_count
uint32 sample_count
uint32 target_sample_count
# Parallel arrays indexed by the shared head-camera pose configuration.
string[] pose_states
float64[] pose_pan
float64[] pose_tilt
string result_uri
bool quality_passed
string[] metric_names
float64[] metric_values
