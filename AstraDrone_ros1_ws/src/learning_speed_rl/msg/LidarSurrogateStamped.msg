std_msgs/Header header
string version
bool valid
string[] diagnostics
# Temporal-generation and backend provenance. These fields are diagnostics and
# are excluded from the frozen 3200-value surrogate.
uint64 temporal_generation
time reset_barrier_stamp
string state_source_type
string state_contract_version
string cloud_frame_mode
time pose_source_stamp
bool pose_used_future
float32[] lidar_surrogate
float32[] lidar_valid_mask
float32[] unknown_mask
uint8[] semantic

uint32 input_points
uint32 finite_points
uint32 in_range_points
uint32 history_frames
float64 lidar_source_rate_hz
float64 build_duration_ms
