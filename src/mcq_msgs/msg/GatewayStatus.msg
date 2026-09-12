# Gateway health at 100 Hz, alongside VehicleState. Fault bit positions are
# shared with firmware/gateway/include/gateway/faults.h.

uint32 FAULT_HEARTBEAT_TIMEOUT = 1
uint32 FAULT_HEARTBEAT_COUNTER = 2
uint32 FAULT_CRC = 4
uint32 FAULT_INVALID_FIELD = 8
uint32 FAULT_STEER_ANGLE_LIMIT = 16
uint32 FAULT_STEER_RATE_LIMIT = 32
uint32 FAULT_SPEED_CAP = 64
uint32 FAULT_SPEED_CAP_MARGIN = 128
uint32 FAULT_POWER_CAP = 256
uint32 FAULT_BRAKE_THROTTLE_CONFLICT = 512
uint32 FAULT_RC_LINK_LOST = 1024
uint32 FAULT_REMOTE_ESTOP = 2048
uint32 FAULT_JETSON_REQUEST = 4096
uint32 FAULT_HANDOVER_REFUSED = 8192
uint32 FAULT_SELF_TEST = 16384
uint32 FAULT_RESET_TIMEOUT = 32768

std_msgs/Header header
uint8 mode                      # VehicleState MODE_* constants
uint32 fault_flags              # active fault bits
uint32 fault_latched            # every fault seen since the last operator reset
uint16 heartbeat_echo
uint16 frames_rejected          # running count of rejected command frames
bool rc_link_ok
bool remote_estop_asserted
bool contactor_closed
bool tx_auto_switch             # transmitter mode switch position
float32 speed                   # gateway's own speed estimate, m/s
