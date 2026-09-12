// Fault bit positions. Shared verbatim with mcq_msgs/msg/GatewayStatus.msg and
// the FAULT_FLAGS signal of GATEWAY_STATUS in mcq_vehicle/dbc/mcqueen.dbc.
#ifndef GATEWAY_FAULTS_H
#define GATEWAY_FAULTS_H

#include <stdint.h>

enum gw_fault {
  GW_FAULT_HEARTBEAT_TIMEOUT = 1u << 0,
  GW_FAULT_HEARTBEAT_COUNTER = 1u << 1,
  GW_FAULT_CRC = 1u << 2,
  GW_FAULT_INVALID_FIELD = 1u << 3,
  GW_FAULT_STEER_ANGLE_LIMIT = 1u << 4,
  GW_FAULT_STEER_RATE_LIMIT = 1u << 5,
  GW_FAULT_SPEED_CAP = 1u << 6,
  GW_FAULT_SPEED_CAP_MARGIN = 1u << 7,
  GW_FAULT_POWER_CAP = 1u << 8,
  GW_FAULT_BRAKE_THROTTLE_CONFLICT = 1u << 9,
  GW_FAULT_RC_LINK_LOST = 1u << 10,
  GW_FAULT_REMOTE_ESTOP = 1u << 11,
  GW_FAULT_JETSON_REQUEST = 1u << 12,
  GW_FAULT_HANDOVER_REFUSED = 1u << 13,
  GW_FAULT_SELF_TEST = 1u << 14,
  GW_FAULT_RESET_TIMEOUT = 1u << 15,
};

// Faults that take the gateway out of AUTO (or RC) into URGENT_STOP. The others
// are limiter activity reported for telemetry.
#define GW_FAULTS_STOPPING                                                          \
  (GW_FAULT_HEARTBEAT_TIMEOUT | GW_FAULT_SPEED_CAP_MARGIN | GW_FAULT_RC_LINK_LOST | \
   GW_FAULT_REMOTE_ESTOP | GW_FAULT_JETSON_REQUEST)

// Faults that block AUTO entry while active.
#define GW_FAULTS_BLOCKING_AUTO                                                              \
  (GW_FAULTS_STOPPING | GW_FAULT_HEARTBEAT_COUNTER | GW_FAULT_CRC | GW_FAULT_INVALID_FIELD | \
   GW_FAULT_STEER_ANGLE_LIMIT | GW_FAULT_SELF_TEST)

typedef uint16_t gw_faults_t;

#endif  // GATEWAY_FAULTS_H
