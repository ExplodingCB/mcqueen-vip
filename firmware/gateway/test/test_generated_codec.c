// The hand-written JETSON_COMMAND codec must agree byte for byte with the code
// cantools generates from the DBC (src/mcq_vehicle/generated/mcqueen.c).
// Raw integers are compared: the generated *_encode helpers truncate while the
// gateway rounds, so the bridge on the Jetson must round before packing too.
#include <math.h>
#include <string.h>

#include "can_vectors.h"
#include "check.h"
#include "gateway/command.h"
#include "gateway/crc8.h"
#include "mcqueen.h"

static int pack_matches_hand_written(void)
{
  for (size_t i = 0; i < CAN_VECTOR_COUNT; ++i) {
    const can_vector_t * v = &CAN_VECTORS[i];
    struct mcqueen_jetson_command_t gen;
    memset(&gen, 0, sizeof(gen));
    gen.steering_angle = (int16_t)lroundf(v->steering_angle / 0.0001f);
    gen.throttle = (uint16_t)lroundf(v->throttle / 0.001f);
    gen.brake = (uint16_t)lroundf(v->brake / 0.001f);
    gen.lat_enable = (uint8_t)v->lat_enable;
    gen.long_enable = (uint8_t)v->long_enable;
    gen.request_urgent_stop = (uint8_t)v->request_urgent_stop;
    gen.heartbeat = v->heartbeat;
    uint8_t frame[8];
    CHECK(mcqueen_jetson_command_pack(frame, &gen, sizeof(frame)) == 8);
    frame[7] = gw_crc8(frame, 7);

    gw_command_t cmd = {
      .steering_angle = v->steering_angle,
      .throttle = v->throttle,
      .brake = v->brake,
      .lat_enable = v->lat_enable != 0,
      .long_enable = v->long_enable != 0,
      .request_urgent_stop = v->request_urgent_stop != 0,
      .heartbeat = v->heartbeat,
    };
    uint8_t ours[8];
    gw_command_encode(&cmd, ours);
    CHECK(memcmp(frame, ours, 8) == 0);
    CHECK(memcmp(frame, v->frame, 8) == 0);
  }
  return 0;
}

static int unpack_matches_hand_written(void)
{
  for (size_t i = 0; i < CAN_VECTOR_COUNT; ++i) {
    const can_vector_t * v = &CAN_VECTORS[i];
    struct mcqueen_jetson_command_t gen;
    CHECK(mcqueen_jetson_command_unpack(&gen, v->frame, 8) == 0);
    gw_command_t cmd;
    CHECK(gw_command_decode(v->frame, &cmd) == GW_CMD_OK);
    CHECK_NEAR(
      mcqueen_jetson_command_steering_angle_decode(gen.steering_angle), cmd.steering_angle, 1e-6);
    CHECK_NEAR(mcqueen_jetson_command_throttle_decode(gen.throttle), cmd.throttle, 1e-6);
    CHECK_NEAR(mcqueen_jetson_command_brake_decode(gen.brake), cmd.brake, 1e-6);
    CHECK(gen.heartbeat == cmd.heartbeat);
    CHECK((gen.lat_enable != 0) == cmd.lat_enable);
    CHECK((gen.long_enable != 0) == cmd.long_enable);
    CHECK((gen.request_urgent_stop != 0) == cmd.request_urgent_stop);
    CHECK(gen.crc == v->frame[7]);
  }
  return 0;
}

static int gateway_status_roundtrip(void)
{
  struct mcqueen_gateway_status_t st;
  memset(&st, 0, sizeof(st));
  st.mode = MCQUEEN_GATEWAY_STATUS_MODE_URGENT_STOP_CHOICE;
  st.tx_auto_switch = 1;
  st.rc_link_ok = 1;
  st.fault_flags = 0x0405;
  st.heartbeat_echo = 65000;
  st.frames_rejected = 7;
  st.speed = mcqueen_gateway_status_speed_encode(12.34);
  uint8_t frame[8];
  CHECK(mcqueen_gateway_status_pack(frame, &st, sizeof(frame)) == 8);
  struct mcqueen_gateway_status_t back;
  CHECK(mcqueen_gateway_status_unpack(&back, frame, 8) == 0);
  CHECK(back.mode == 3 && back.tx_auto_switch == 1 && back.rc_link_ok == 1);
  CHECK(back.fault_flags == 0x0405 && back.heartbeat_echo == 65000 && back.frames_rejected == 7);
  CHECK_NEAR(mcqueen_gateway_status_speed_decode(back.speed), 12.34, 0.01);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(pack_matches_hand_written);
  RUN(unpack_matches_hand_written);
  RUN(gateway_status_roundtrip);
  return failures ? 1 : 0;
}
