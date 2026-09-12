# Output of the local planner: 2.5 s horizon sampled every 50 ms.

uint8 REFERENCE_RACELINE = 0
uint8 REFERENCE_CENTERLINE = 1
uint8 REFERENCE_BOUNDARY = 2

std_msgs/Header header          # frame_id = map
TrajectoryPoint[] points
uint8 reference                 # one of the REFERENCE_* constants
bool stop_requested             # profile is a braking ramp to zero
