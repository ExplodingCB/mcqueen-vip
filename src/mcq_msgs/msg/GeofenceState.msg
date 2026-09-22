# mcq_track's verdict on whether the current pose can be trusted and is on the track.

uint8 REASON_NONE = 0
uint8 REASON_OUTSIDE_TRACK = 1
uint8 REASON_COVARIANCE = 2
uint8 REASON_GNSS_BUDGET = 3
uint8 REASON_PERCEPTION_DISAGREEMENT = 4
uint8 REASON_POSE_INVALID = 5
uint8 REASON_POSE_STALE = 6

std_msgs/Header header
bool violation
uint8 reason
float32 distance_to_edge        # m, negative when outside the inflated polygon
float32 margin                  # m, inflation used for the check
