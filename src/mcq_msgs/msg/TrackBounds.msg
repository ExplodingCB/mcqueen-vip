# Perceived track edges in base_link, from mcq_perception.

std_msgs/Header header          # frame_id = base_link
geometry_msgs/Point[] left
geometry_msgs/Point[] right
float32[] left_confidence       # 0..1 per point
float32[] right_confidence
