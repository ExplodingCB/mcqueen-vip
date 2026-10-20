# Pit summary at 10 Hz. Invalid scalar measurements are NaN, never a healthy zero.
std_msgs/Header header
string track_id
bool ego_valid
bool trajectory_valid
bool gateway_valid
bool planner_valid
bool command_valid
float32 ego_age
float32 trajectory_age
float32 gateway_age
float32 planner_age
float32 command_age
float32 speed
float32 target_speed
float32 speed_error             # actual - target, m/s
float32 lateral_deviation       # EgoState.d, m relative to active reference
string gnss_status              # NONE, FLOAT, FIXED, UNKNOWN
string gateway_mode
uint32 fault_flags              # only meaningful when gateway_valid
uint32 fault_latched
string active_faults           # decoded names, NONE or UNKNOWN
string latched_faults
string submode
float32 speed_cap
bool urgent_stop               # only meaningful when command_valid
uint32 completed_laps
bool lap_active
float64 current_lap_time
float64 last_lap_time
float64 best_lap_time
