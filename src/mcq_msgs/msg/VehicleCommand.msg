# What the Jetson asks the gateway to do. Sent at 100 Hz; the gateway drops
# to URGENT_STOP if 50 ms pass without an accepted frame.

std_msgs/Header header
float32 steering_angle          # rad, target at the front wheels, left positive
float32 throttle                # 0..1
float32 brake                   # 0..1
bool lat_enable                 # gateway may apply steering_angle
bool long_enable                # gateway may apply throttle and brake
bool request_urgent_stop        # software urgent stop (AKS rule)
uint16 heartbeat                # increments by one every tick
