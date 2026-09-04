bool valid
string observation_id
string backend
string method
# Finite backend-native ranking score. The scale is backend-specific: SAM2
# confidence is typically in [0, 1], while GPD emits an unbounded logit margin.
float32 score
geometry_msgs/PoseStamped pregrasp
geometry_msgs/PoseStamped grasp
geometry_msgs/PoseStamped lift
float32 table_height_m
float32 object_height_m
float32 grasp_width_m
float32 wrist_yaw_seed_rad
float32 wrist_yaw_min_rad
float32 wrist_yaw_max_rad
string selection_reason
string debug_asset_uri
