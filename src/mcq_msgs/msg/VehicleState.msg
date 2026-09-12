# What the safety gateway reports about the kart, decoded from CAN at 100 Hz.
# Frame layouts are defined once in mcq_vehicle/dbc/mcqueen.dbc.

uint8 MODE_INIT = 0
uint8 MODE_RC = 1
uint8 MODE_AUTO = 2
uint8 MODE_URGENT_STOP = 3
uint8 MODE_DRIVETRAIN_OFF = 4
uint8 MODE_FAULT = 5

std_msgs/Header header
float32 steering_angle          # rad at the front wheels, left positive
float32 steering_rate           # rad/s
float32[4] wheel_speed          # m/s, order fl fr rl rr
float32 motor_rpm
float32 motor_current           # A
float32 motor_power             # W, measured before the motor controller (AKS reporting)
float32 battery_voltage         # V, drivetrain pack
float32 brake_pressure          # normalized 0..1
uint8 mode                      # one of the MODE_* constants
uint32 fault_flags              # gateway fault bits, see GatewayStatus
uint16 heartbeat_echo           # last heartbeat counter the gateway accepted
