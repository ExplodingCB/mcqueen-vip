# Fused kart state from mcq_localization at 100 Hz, in the map frame.

uint8 GNSS_NONE = 0
uint8 GNSS_FLOAT = 1
uint8 GNSS_FIXED = 2

std_msgs/Header header          # frame_id = map
geometry_msgs/Pose pose         # base_link (rear axle center) in map
float32 v                       # longitudinal speed, m/s
float32 v_lat                   # lateral speed, m/s, positive left
float32 yaw_rate                # rad/s
float32 a_long                  # m/s^2
float32 a_lat                   # m/s^2
float64[36] covariance          # 6x6 row-major, x y z roll pitch yaw
uint8 gnss_status               # one of the GNSS_* constants
float32 s                       # Frenet arc length along the current reference, m
float32 d                       # Frenet lateral offset, m, positive left
