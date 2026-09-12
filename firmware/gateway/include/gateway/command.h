// JETSON_COMMAND (0x100) frame codec. Layout is defined in
// src/mcq_vehicle/dbc/mcqueen.dbc; test/can_vectors.h is generated from that
// file with tools/gen_can_vectors.py and pins this code to it.
#ifndef GATEWAY_COMMAND_H
#define GATEWAY_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#define GW_CMD_FRAME_ID 0x100u
#define GW_CMD_FRAME_LEN 8u

typedef struct
{
  float steering_angle;  // rad at the front wheels, left positive
  float throttle;        // 0..1
  float brake;           // 0..1
  bool lat_enable;
  bool long_enable;
  bool request_urgent_stop;
  uint16_t heartbeat;
} gw_command_t;

typedef enum {
  GW_CMD_OK = 0,
  GW_CMD_BAD_CRC,
  GW_CMD_OUT_OF_RANGE,  // a field outside its physical range or not finite
} gw_command_result_t;

// Unpacks the fields after checking the CRC. No range checks; *cmd is left
// untouched on a CRC failure.
bool gw_command_unpack(const uint8_t frame[GW_CMD_FRAME_LEN], gw_command_t * cmd);

// Unpacks and range-checks a raw frame. On any failure *cmd is left untouched.
// Range checks use the limits in config.h so that a command the gateway would
// have to clamp is rejected instead, as the safety document specifies.
gw_command_result_t gw_command_decode(const uint8_t frame[GW_CMD_FRAME_LEN], gw_command_t * cmd);

// Encodes a command into a frame, including the CRC. Used by the host tests and
// by the mcq_vehicle bridge's reference implementation.
void gw_command_encode(const gw_command_t * cmd, uint8_t frame[GW_CMD_FRAME_LEN]);

// Field-level validity without CAN framing (for commands arriving over USB).
bool gw_command_fields_valid(const gw_command_t * cmd);

#endif  // GATEWAY_COMMAND_H
