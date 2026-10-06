# Applied planner state, published periodically; not a command interface.
std_msgs/Header header
string submode                 # FOLLOW or BOUNDARY
float32 speed_cap              # effective cap for the active mode, m/s
