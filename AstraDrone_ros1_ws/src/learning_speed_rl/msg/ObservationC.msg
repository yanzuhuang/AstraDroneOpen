std_msgs/Header header
string version
bool valid
string[] diagnostics

float32[] lidar_surrogate
float32[] lidar_valid_mask
float32[] lidar_unknown_mask
uint8[] lidar_semantic

geometry_msgs/Point[] future_positions_body
float32[] future_sample_offsets
string trajectory_sampling_mode
float32 trajectory_sample_spacing
float32 trajectory_max_distance
int64 trajectory_id
time trajectory_start_time
string trajectory_source_frame

geometry_msgs/Vector3 actual_velocity_body
geometry_msgs/Vector3 tracking_error_body
float32 tracking_error_norm
float32 previous_v_max
