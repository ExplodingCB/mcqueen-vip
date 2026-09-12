uint8 CLASS_KART = 0
uint8 CLASS_PERSON = 1
uint8 CLASS_OBSTACLE = 2

uint8 object_class
geometry_msgs/Pose pose         # map frame
geometry_msgs/Vector3 velocity  # m/s, map frame
geometry_msgs/Vector3 size      # m, bounding box length width height
float32 confidence
