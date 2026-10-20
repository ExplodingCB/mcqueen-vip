#include <string.h>

#include "check.h"
#include "crsf_test.h"
#include "gateway/config.h"

static int golden_wire_frame_and_crc(void)
{
  // Fixed vector, packed separately with Python integer shifts and CRC division.
  const uint8_t frame[] = {0xC8, 0x18, 0x16, 0xE0, 0x63, 0xC5, 0xC4, 0x01, 0x30,
                           0x71, 0x56, 0x4C, 0xFC, 0xFF, 0x01, 0x00, 0xE0, 0x1E,
                           0x80, 0xFF, 0x0B, 0x80, 0xA3, 0xEF, 0xFF, 0x83};
  const uint16_t expected[] = {992, 172,  1811, 0,    1811, 172,  1811, 2047,
                               1,   1024, 123,  1984, 191,  1792, 1000, 2047};
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  CHECK(gw_crsf_init(&rx, &config));
  CHECK(gw_crsf_feed(&rx, frame, sizeof(frame), 10u) == sizeof(frame));
  CHECK(rx.have_channels);
  for (size_t i = 0u; i < GW_CRSF_CHANNEL_COUNT; ++i) {
    CHECK(rx.channels[i] == expected[i]);
  }
  CHECK(gw_crsf_crc8((const uint8_t *)"123456789", 9u) == 0xBCu);
  CHECK(gw_crsf_crc8(NULL, 0u) == 0u);
  return 0;
}

static int valid_zero_controls_and_scaling(void)
{
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  CHECK(gw_crsf_init(&rx, &config));
  gw_inputs_t in = {0};
  in.speed = 2.0f;
  gw_crsf_apply(&rx, 0u, &in);
  CHECK(!in.rc_link_ok);
  CHECK_NEAR(in.rc_brake, 1.0f, 0.0f);
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  test_send(&rx, channels, 0u);
  gw_crsf_apply(&rx, 0u, &in);
  CHECK(in.rc_link_ok);
  CHECK(!in.tx_auto_switch && !in.tx_reset);
  CHECK_NEAR(in.rc_steering, 0.0f, 0.0f);
  CHECK_NEAR(in.rc_throttle, 0.0f, 0.0f);
  CHECK_NEAR(in.rc_brake, 0.0f, 0.0f);
  CHECK_NEAR(in.speed, 2.0f, 0.0f);  // unrelated inputs survive
  channels[0] = GW_CRSF_CHANNEL_MIN;
  channels[1] = GW_CRSF_CHANNEL_MAX;
  channels[2] = GW_CRSF_CHANNEL_CENTER;
  channels[4] = GW_CRSF_CHANNEL_MAX;
  channels[5] = GW_CRSF_CHANNEL_MAX;
  test_send(&rx, channels, 10u);
  gw_crsf_apply(&rx, 10u, &in);
  CHECK(in.rc_link_ok && in.tx_auto_switch && in.tx_reset);
  CHECK_NEAR(in.rc_steering, -GW_STEER_ANGLE_MAX_RAD, 1e-6);
  CHECK_NEAR(in.rc_throttle, 1.0f, 1e-6);
  CHECK_NEAR(in.rc_brake, 820.0f / 1639.0f, 1e-6);
  channels[0] = 2047u;
  channels[1] = 0u;
  channels[2] = 2047u;
  test_send(&rx, channels, 20u);
  gw_crsf_apply(&rx, 20u, &in);
  CHECK_NEAR(in.rc_steering, GW_STEER_ANGLE_MAX_RAD, 1e-6);
  CHECK_NEAR(in.rc_throttle, 0.0f, 1e-6);
  CHECK_NEAR(in.rc_brake, 1.0f, 1e-6);
  return 0;
}

static int configured_failsafe_does_not_become_controls(void)
{
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  // Receiver-specific high sentinel on CH16, with a tolerance interval.
  config.failsafe_channel = 15u;
  config.failsafe_min = 1750u;
  config.failsafe_max = 1850u;
  CHECK(gw_crsf_init(&rx, &config));
  gw_inputs_t in = {0};
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  channels[4] = GW_CRSF_CHANNEL_MAX;
  test_send(&rx, channels, 10u);
  gw_crsf_apply(&rx, 10u, &in);
  CHECK(in.rc_link_ok && in.tx_auto_switch);
  for (uint16_t health = config.failsafe_min; health <= config.failsafe_max; ++health) {
    channels[15] = health;
    channels[4] = GW_CRSF_CHANNEL_MIN;  // failsafe must not synthesize an RC pass
    channels[5] = GW_CRSF_CHANNEL_MAX;  // or a reset
    channels[1] = GW_CRSF_CHANNEL_MAX;
    test_send(&rx, channels, 20u);
    gw_crsf_apply(&rx, 20u, &in);
    CHECK(!in.rc_link_ok && in.tx_auto_switch && !in.tx_reset);
    CHECK(rx.last_channels_ms == 10u);
    CHECK_NEAR(in.rc_throttle, 0.0f, 0.0f);
    CHECK_NEAR(in.rc_brake, 1.0f, 0.0f);
  }
  channels[15] = config.failsafe_max + 1u;
  test_send(&rx, channels, 30u);
  gw_crsf_apply(&rx, 30u, &in);
  CHECK(in.rc_link_ok);
  return 0;
}

static int timeout_crc_telemetry_and_wraparound(void)
{
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  CHECK(gw_crsf_init(&rx, &config));
  gw_inputs_t in = {0};
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  channels[4] = GW_CRSF_CHANNEL_MAX;
  uint32_t start = UINT32_MAX - 40u;
  test_send(&rx, channels, start);
  gw_crsf_apply(&rx, start, &in);
  CHECK(in.rc_link_ok);
  uint8_t bad[TEST_CRSF_FRAME_LEN];
  test_frame(channels, bad);
  bad[8] ^= 0x40u;
  // Valid link statistics, including LQ=100, never refresh RC channel freshness.
  uint8_t telemetry[] = {0xC8, 12, 0x14, 30, 30, 100, 0, 0, 0, 0, 0, 0, 0, 0};
  telemetry[13] = gw_crsf_crc8(telemetry + 2u, 11u);
  for (uint32_t elapsed = 10u; elapsed < GW_RC_TIMEOUT_MS; elapsed += 10u) {
    (void)gw_crsf_feed(&rx, bad, sizeof(bad), start + elapsed);
    (void)gw_crsf_feed(&rx, telemetry, sizeof(telemetry), start + elapsed);
    gw_crsf_apply(&rx, start + elapsed, &in);
    CHECK(in.rc_link_ok);
    CHECK(rx.last_channels_ms == start);
  }
  gw_crsf_apply(&rx, start + GW_RC_TIMEOUT_MS, &in);
  CHECK(!in.rc_link_ok && in.tx_auto_switch);
  CHECK_NEAR(in.rc_throttle, 0.0f, 0.0f);
  return 0;
}

static int every_split_and_interleaved_frames(void)
{
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  uint8_t frame[TEST_CRSF_FRAME_LEN];
  test_frame(channels, frame);
  gw_crsf_config_t config = gw_crsf_default_config();
  for (size_t split = 0u; split <= sizeof(frame); ++split) {
    gw_crsf_t rx;
    CHECK(gw_crsf_init(&rx, &config));
    gw_inputs_t in = {0};
    CHECK(gw_crsf_feed(&rx, frame, split, 10u) == split);
    gw_crsf_apply(&rx, 10u, &in);
    CHECK(in.rc_link_ok == (split == sizeof(frame)));
    CHECK(gw_crsf_feed(&rx, frame + split, sizeof(frame) - split, 20u) == sizeof(frame) - split);
    gw_crsf_apply(&rx, 20u, &in);
    CHECK(in.rc_link_ok);
  }
  gw_crsf_t rx;
  CHECK(gw_crsf_init(&rx, &config));
  uint8_t stream[2u * TEST_CRSF_FRAME_LEN + 4u];
  memcpy(stream, frame, sizeof(frame));
  const uint8_t unknown[] = {0xC8u, 2u, 0x55u, 0xE4u};
  memcpy(stream + sizeof(frame), unknown, sizeof(unknown));
  channels[4] = GW_CRSF_CHANNEL_MAX;
  test_frame(channels, stream + sizeof(frame) + sizeof(unknown));
  CHECK(gw_crsf_feed(&rx, stream, sizeof(stream), 30u) == sizeof(stream));
  gw_inputs_t in = {0};
  gw_crsf_apply(&rx, 30u, &in);
  CHECK(in.rc_link_ok && in.tx_auto_switch);
  return 0;
}

static int resynchronizes_after_noise_and_stalled_frames(void)
{
  gw_crsf_config_t config = gw_crsf_default_config();
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  uint8_t frame[TEST_CRSF_FRAME_LEN];
  test_frame(channels, frame);
  // Every truncated prefix followed by another valid channel frame.
  for (size_t prefix = 1u; prefix < sizeof(frame); ++prefix) {
    gw_crsf_t rx;
    CHECK(gw_crsf_init(&rx, &config));
    (void)gw_crsf_feed(&rx, frame, prefix, 10u);
    (void)gw_crsf_feed(&rx, frame, sizeof(frame), 11u);
    gw_inputs_t in = {0};
    gw_crsf_apply(&rx, 10u + GW_CRSF_FRAME_TIMEOUT_MS, &in);
    CHECK(in.rc_link_ok);
    CHECK(rx.last_channels_ms == 11u);
    CHECK(rx.buffered == 0u);
  }
  gw_crsf_t rx;
  CHECK(gw_crsf_init(&rx, &config));
  const uint8_t noise[] = {0xFF, 0xC8, 0, 0xC8, 1, 0xC8, 63, 0xC8, 255, 0xC8, 62, 0xAA};
  (void)gw_crsf_feed(&rx, noise, sizeof(noise), 0u);
  (void)gw_crsf_feed(&rx, frame, sizeof(frame), 1u);
  gw_inputs_t in = {0};
  gw_crsf_apply(&rx, GW_CRSF_FRAME_TIMEOUT_MS, &in);
  CHECK(in.rc_link_ok && rx.last_channels_ms == 1u);
  // A stalled frame must not be completed with fresh bytes after its deadline.
  CHECK(gw_crsf_init(&rx, &config));
  (void)gw_crsf_feed(&rx, frame, 10u, 0u);
  (void)gw_crsf_feed(&rx, frame + 10u, sizeof(frame) - 10u, GW_CRSF_FRAME_TIMEOUT_MS);
  gw_crsf_apply(&rx, GW_CRSF_FRAME_TIMEOUT_MS, &in);
  CHECK(!in.rc_link_ok);
  (void)gw_crsf_feed(&rx, frame, sizeof(frame), 40u);
  gw_crsf_apply(&rx, 60u, &in);
  CHECK(in.rc_link_ok);
  return 0;
}

static int extensions_and_nested_payloads(void)
{
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  CHECK(gw_crsf_init(&rx, &config));
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  uint8_t frame[GW_CRSF_FRAME_MAX];
  test_frame(channels, frame);
  frame[0] = 0u;  // broadcast sync accepted
  frame[1] = GW_CRSF_FRAME_MAX - 2u;
  memset(frame + 25u, 0xFF, sizeof(frame) - 26u);
  frame[sizeof(frame) - 1u] = gw_crsf_crc8(frame + 2u, sizeof(frame) - 3u);
  (void)gw_crsf_feed(&rx, frame, sizeof(frame), 0u);
  gw_inputs_t in = {0};
  gw_crsf_apply(&rx, 0u, &in);
  CHECK(in.rc_link_ok);
  // An unknown packet's opaque payload must not act as a channel packet.
  CHECK(gw_crsf_init(&rx, &config));
  frame[0] = 0xC8u;
  frame[2] = 0x7Fu;
  test_frame(channels, frame + 3u);
  frame[sizeof(frame) - 1u] = gw_crsf_crc8(frame + 2u, sizeof(frame) - 3u);
  (void)gw_crsf_feed(&rx, frame, 29u, 10u);
  gw_crsf_apply(&rx, 10u, &in);
  CHECK(!in.rc_link_ok);
  (void)gw_crsf_feed(&rx, frame + 29u, sizeof(frame) - 29u, 11u);
  gw_crsf_apply(&rx, 11u, &in);
  CHECK(!in.rc_link_ok);
  // A valid CRC with a too-short channel payload is still not a channel sample.
  const uint8_t short_frame[] = {0xC8, 2, 0x16, 0xD3};
  (void)gw_crsf_feed(&rx, short_frame, sizeof(short_frame), 12u);
  gw_crsf_apply(&rx, 12u, &in);
  CHECK(!in.rc_link_ok);
  return 0;
}

static int loss_is_retained_until_gateway_tick(void)
{
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  CHECK(gw_crsf_init(&rx, &config));
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  test_send(&rx, channels, 0u);
  gw_inputs_t in = {0};
  gw_crsf_apply(&rx, 0u, &in);
  channels[6] = GW_CRSF_CHANNEL_MIN;
  test_send(&rx, channels, 10u);
  channels[6] = GW_CRSF_CHANNEL_MAX;
  test_send(&rx, channels, 10u);
  gw_crsf_apply(&rx, 10u, &in);
  CHECK(!in.rc_link_ok);
  gw_crsf_apply(&rx, 20u, &in);
  CHECK(in.rc_link_ok);
  // A new valid packet arriving just at timeout cannot hide the elapsed outage.
  test_send(&rx, channels, 10u + GW_RC_TIMEOUT_MS);
  gw_crsf_apply(&rx, 10u + GW_RC_TIMEOUT_MS, &in);
  CHECK(!in.rc_link_ok);
  gw_crsf_apply(&rx, 20u + GW_RC_TIMEOUT_MS, &in);
  CHECK(in.rc_link_ok);
  return 0;
}

static int corrupt_frames_never_refresh_controls(void)
{
  gw_crsf_config_t config = gw_crsf_default_config();
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  uint8_t frame[TEST_CRSF_FRAME_LEN];
  channels[4] = GW_CRSF_CHANNEL_MAX;
  channels[1] = GW_CRSF_CHANNEL_MAX;
  test_frame(channels, frame);
  // Every single-bit corruption in the protected type/payload/CRC region.
  for (size_t byte = 2u; byte < sizeof(frame); ++byte) {
    for (unsigned bit = 0u; bit < 8u; ++bit) {
      gw_crsf_t rx;
      CHECK(gw_crsf_init(&rx, &config));
      uint16_t centered[GW_CRSF_CHANNEL_COUNT];
      test_channels(centered);
      test_send(&rx, centered, 1u);
      uint8_t damaged[TEST_CRSF_FRAME_LEN];
      memcpy(damaged, frame, sizeof(frame));
      damaged[byte] ^= (uint8_t)(1u << bit);
      (void)gw_crsf_feed(&rx, damaged, sizeof(damaged), 10u);
      gw_inputs_t in = {0};
      gw_crsf_apply(&rx, 10u + GW_CRSF_FRAME_TIMEOUT_MS, &in);
      CHECK(in.rc_link_ok && !in.tx_auto_switch);
      CHECK_NEAR(in.rc_throttle, 0.0f, 0.0f);
      CHECK(rx.last_channels_ms == 1u);
    }
  }
  return 0;
}

static int invalid_configuration_and_work_budget(void)
{
  gw_crsf_t rx;
  gw_crsf_config_t config = gw_crsf_default_config();
  CHECK(!gw_crsf_init(&rx, NULL));
  config.reset_channel = 16u;
  CHECK(!gw_crsf_init(&rx, &config));
  config = gw_crsf_default_config();
  config.failsafe_channel = config.auto_channel;
  CHECK(!gw_crsf_init(&rx, &config));
  config = gw_crsf_default_config();
  config.failsafe_max = 2048u;
  CHECK(!gw_crsf_init(&rx, &config));
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  test_channels(channels);
  test_send(&rx, channels, 0u);
  gw_inputs_t in = {0};
  gw_crsf_apply(&rx, 0u, &in);
  CHECK(!in.rc_link_ok);
  config = gw_crsf_default_config();
  CHECK(gw_crsf_init(&rx, &config));
  uint8_t noise[GW_CRSF_READ_BUDGET + 100u];
  memset(noise, 0xC8, sizeof(noise));
  for (uint32_t i = 0u; i < 100u; ++i) {
    CHECK(gw_crsf_feed(&rx, noise, sizeof(noise), i) == GW_CRSF_READ_BUDGET);
    CHECK(rx.buffered < GW_CRSF_FRAME_MAX);
  }
  test_send(&rx, channels, 100u);
  gw_crsf_apply(&rx, 120u, &in);
  CHECK(in.rc_link_ok);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(golden_wire_frame_and_crc);
  RUN(valid_zero_controls_and_scaling);
  RUN(configured_failsafe_does_not_become_controls);
  RUN(timeout_crc_telemetry_and_wraparound);
  RUN(every_split_and_interleaved_frames);
  RUN(resynchronizes_after_noise_and_stalled_frames);
  RUN(extensions_and_nested_payloads);
  RUN(loss_is_retained_until_gateway_tick);
  RUN(corrupt_frames_never_refresh_controls);
  RUN(invalid_configuration_and_work_budget);
  return failures ? 1 : 0;
}
