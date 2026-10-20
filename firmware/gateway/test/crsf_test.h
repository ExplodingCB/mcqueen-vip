#ifndef GATEWAY_TEST_CRSF_TEST_H
#define GATEWAY_TEST_CRSF_TEST_H

#include <string.h>

#include "gateway/crsf.h"

#define TEST_CRSF_FRAME_LEN 26u

static inline void test_channels(uint16_t channels[GW_CRSF_CHANNEL_COUNT])
{
  for (size_t i = 0u; i < GW_CRSF_CHANNEL_COUNT; ++i) {
    channels[i] = GW_CRSF_CHANNEL_MIN;
  }
  channels[0] = GW_CRSF_CHANNEL_CENTER;
  channels[6] = GW_CRSF_CHANNEL_MAX;
}

// Independent bit-by-bit writer, rather than the parser's accumulator algorithm.
static inline void test_frame(const uint16_t channels[GW_CRSF_CHANNEL_COUNT], uint8_t * frame)
{
  memset(frame, 0, TEST_CRSF_FRAME_LEN);
  frame[0] = 0xC8u;
  frame[1] = 24u;
  frame[2] = 0x16u;
  for (size_t i = 0u; i < GW_CRSF_CHANNEL_COUNT; ++i) {
    for (unsigned bit = 0u; bit < 11u; ++bit) {
      size_t packed_bit = i * 11u + bit;
      if ((channels[i] & (1u << bit)) != 0u) {
        frame[3u + packed_bit / 8u] |= (uint8_t)(1u << (packed_bit % 8u));
      }
    }
  }
  frame[25] = gw_crsf_crc8(frame + 2u, 23u);
}

static inline void test_send(gw_crsf_t * rx, const uint16_t * channels, uint32_t now_ms)
{
  uint8_t frame[TEST_CRSF_FRAME_LEN];
  test_frame(channels, frame);
  (void)gw_crsf_feed(rx, frame, sizeof(frame), now_ms);
}

#endif  // GATEWAY_TEST_CRSF_TEST_H
