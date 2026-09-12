#include <math.h>
#include <string.h>

#include "can_vectors.h"
#include "check.h"
#include "gateway/command.h"
#include "gateway/config.h"

static int decodes_generated_vectors(void)
{
  for (size_t i = 0; i < CAN_VECTOR_COUNT; ++i) {
    const can_vector_t * v = &CAN_VECTORS[i];
    gw_command_t cmd;
    CHECK(gw_command_decode(v->frame, &cmd) == GW_CMD_OK);
    CHECK_NEAR(cmd.steering_angle, v->steering_angle, 1e-4);
    CHECK_NEAR(cmd.throttle, v->throttle, 1e-3);
    CHECK_NEAR(cmd.brake, v->brake, 1e-3);
    CHECK(cmd.lat_enable == (v->lat_enable != 0));
    CHECK(cmd.long_enable == (v->long_enable != 0));
    CHECK(cmd.request_urgent_stop == (v->request_urgent_stop != 0));
    CHECK(cmd.heartbeat == v->heartbeat);
  }
  return 0;
}

static int encodes_generated_vectors(void)
{
  for (size_t i = 0; i < CAN_VECTOR_COUNT; ++i) {
    const can_vector_t * v = &CAN_VECTORS[i];
    gw_command_t cmd = {
      .steering_angle = v->steering_angle,
      .throttle = v->throttle,
      .brake = v->brake,
      .lat_enable = v->lat_enable != 0,
      .long_enable = v->long_enable != 0,
      .request_urgent_stop = v->request_urgent_stop != 0,
      .heartbeat = v->heartbeat,
    };
    uint8_t frame[8];
    gw_command_encode(&cmd, frame);
    CHECK(memcmp(frame, v->frame, 8) == 0);
  }
  return 0;
}

static int rejects_bad_crc(void)
{
  uint8_t frame[8];
  memcpy(frame, CAN_VECTORS[1].frame, 8);
  frame[7] ^= 0x01u;
  gw_command_t cmd;
  memset(&cmd, 0x55, sizeof(cmd));
  CHECK(gw_command_decode(frame, &cmd) == GW_CMD_BAD_CRC);
  CHECK(cmd.heartbeat == 0x5555u);  // untouched
  return 0;
}

static int rejects_out_of_range(void)
{
  gw_command_t cmd = {.steering_angle = GW_STEER_ANGLE_MAX_RAD + 0.01f, .throttle = 0.0f};
  uint8_t frame[8];
  gw_command_encode(&cmd, frame);  // encode clamps only to the field range, not the limit
  gw_command_t out;
  CHECK(gw_command_decode(frame, &out) == GW_CMD_OUT_OF_RANGE);

  // Raw throttle 1023 decodes to 1.023, above the physical range.
  memcpy(frame, CAN_VECTORS[0].frame, 8);
  frame[2] = 0xFFu;
  frame[3] |= 0x03u;
  frame[7] = 0;
  gw_command_t tmp;
  gw_command_encode(&(gw_command_t){0}, frame);  // recompute a valid CRC on a zero frame
  frame[2] = 0xFFu;
  frame[3] = 0x03u;
  // Recompute CRC over the modified payload.
  extern uint8_t gw_crc8(const uint8_t *, size_t);
  frame[7] = gw_crc8(frame, 7);
  CHECK(gw_command_unpack(frame, &tmp));
  CHECK(tmp.throttle > 1.0f);
  CHECK(gw_command_decode(frame, &out) == GW_CMD_OUT_OF_RANGE);
  return 0;
}

static int rejects_nan_and_inf(void)
{
  gw_command_t cmd = {.steering_angle = NAN, .throttle = 0.5f, .brake = 0.0f};
  CHECK(!gw_command_fields_valid(&cmd));
  cmd.steering_angle = 0.0f;
  cmd.throttle = INFINITY;
  CHECK(!gw_command_fields_valid(&cmd));
  cmd.throttle = 0.5f;
  cmd.brake = -0.1f;
  CHECK(!gw_command_fields_valid(&cmd));
  cmd.brake = 0.0f;
  CHECK(gw_command_fields_valid(&cmd));
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(decodes_generated_vectors);
  RUN(encodes_generated_vectors);
  RUN(rejects_bad_crc);
  RUN(rejects_out_of_range);
  RUN(rejects_nan_and_inf);
  return failures ? 1 : 0;
}
