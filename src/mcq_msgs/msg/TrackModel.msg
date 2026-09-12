# The track as mcq_track currently believes it. Arrays over centerline points
# share one index; raceline arrays share another and are empty in BOUNDARY mode.

std_msgs/Header header          # frame_id = map
geometry_msgs/Point[] centerline
float32[] width_left            # m from the centerline to the left edge
float32[] width_right           # m from the centerline to the right edge
geometry_msgs/Point[] raceline
float32[] raceline_kappa        # 1/m, left positive
float32[] raceline_v            # m/s target speed
bool closed
string track_id
