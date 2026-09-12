#include "gateway/command.h"

#include <math.h>

#include "gateway/config.h"
#include "gateway/crc8.h"

// Little-endian (Intel) bit field helpers matching the DBC "@1" byte order.
static uint32_t extract_le(const uint8_t * frame, unsigned start, unsigned len)
{
  uint32_t value = 0;
  for (unsigned i = 0; i < len; ++i) {
    unsigned bit = start + i;
    uint32_t b = (uint32_t)((frame[bit / 8u] >> (bit % 8u)) & 1u);
    value |= b << i;
  }
  return value;
}

static void insert_le(uint8_t * frame, unsigned start, unsigned len, uint32_t value)
{
  for (unsigned i = 0; i < len; ++i) {
    unsigned bit = start + i;
    uint8_t mask = (uint8_t)(1u << (bit % 8u));
    if ((value >> i) & 1u) {
      frame[bit / 8u] |= mask;
    } else {
      frame[bit / 8u] &= (uint8_t)~mask;
    }
  }
}

static int32_t sign_extend(uint32_t value, unsigned len)
{
  uint32_t sign = 1u << (len - 1u);
  return (int32_t)((value ^ sign) - sign);
}

bool gw_command_fields_valid(const gw_command_t * cmd)
{
  if (isnan(cmd->steering_angle) || isnan(cmd->throttle) || isnan(cmd->brake)) {
    return false;
  }
  if (isinf(cmd->steering_angle) || isinf(cmd->throttle) || isinf(cmd->brake)) {
    return false;
  }
  if (fabsf(cmd->steering_angle) > GW_STEER_ANGLE_MAX_RAD) {
    return false;
  }
  if (cmd->throttle < 0.0f || cmd->throttle > 1.0f) {
    return false;
  }
  if (cmd->brake < 0.0f || cmd->brake > 1.0f) {
    return false;
  }
  return true;
}

bool gw_command_unpack(const uint8_t frame[GW_CMD_FRAME_LEN], gw_command_t * cmd)
{
  if (gw_crc8(frame, 7) != frame[7]) {
    return false;
  }
  cmd->steering_angle = (float)sign_extend(extract_le(frame, 0, 16), 16) * 0.0001f;
  cmd->throttle = (float)extract_le(frame, 16, 10) * 0.001f;
  cmd->brake = (float)extract_le(frame, 26, 10) * 0.001f;
  cmd->lat_enable = extract_le(frame, 36, 1) != 0u;
  cmd->long_enable = extract_le(frame, 37, 1) != 0u;
  cmd->request_urgent_stop = extract_le(frame, 38, 1) != 0u;
  cmd->heartbeat = (uint16_t)extract_le(frame, 40, 16);
  return true;
}

gw_command_result_t gw_command_decode(const uint8_t frame[GW_CMD_FRAME_LEN], gw_command_t * cmd)
{
  gw_command_t out;
  if (!gw_command_unpack(frame, &out)) {
    return GW_CMD_BAD_CRC;
  }
  if (!gw_command_fields_valid(&out)) {
    return GW_CMD_OUT_OF_RANGE;
  }
  *cmd = out;
  return GW_CMD_OK;
}

static uint32_t quantize(float value, float factor, float lo, float hi)
{
  if (value < lo) {
    value = lo;
  }
  if (value > hi) {
    value = hi;
  }
  return (uint32_t)(int32_t)lroundf(value / factor);
}

void gw_command_encode(const gw_command_t * cmd, uint8_t frame[GW_CMD_FRAME_LEN])
{
  for (unsigned i = 0; i < GW_CMD_FRAME_LEN; ++i) {
    frame[i] = 0u;
  }
  insert_le(frame, 0, 16, quantize(cmd->steering_angle, 0.0001f, -3.2768f, 3.2767f));
  insert_le(frame, 16, 10, quantize(cmd->throttle, 0.001f, 0.0f, 1.0f));
  insert_le(frame, 26, 10, quantize(cmd->brake, 0.001f, 0.0f, 1.0f));
  insert_le(frame, 36, 1, cmd->lat_enable ? 1u : 0u);
  insert_le(frame, 37, 1, cmd->long_enable ? 1u : 0u);
  insert_le(frame, 38, 1, cmd->request_urgent_stop ? 1u : 0u);
  insert_le(frame, 40, 16, cmd->heartbeat);
  frame[7] = gw_crc8(frame, 7);
}
