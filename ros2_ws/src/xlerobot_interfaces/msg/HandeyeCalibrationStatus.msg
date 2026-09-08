# Automatic fixed-camera / moving-Tag23 capture, including held-out validation.
std_msgs/Header header
string unit_id
bool running
string phase
string message
int32 pose_index
uint32 pose_count
uint32 sample_count
uint32 target_sample_count
string[] pose_states
string[] pose_roles
string[] pose_messages
string result_uri
string report_uri
bool quality_passed
# Names are prefixed fit. or validation.; these are separate populations.
string[] metric_names
float64[] metric_values
