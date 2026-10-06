#include "gateway/crsf.h"

#include <string.h>

#include "gateway/config.h"

#define CRSF_SYNC 0xC8u
#define CRSF_BROADCAST 0x00u
#define CRSF_CHANNELS 0x16u
#define CRSF_CHANNEL_PAYLOAD 22u

uint8_t gw_crsf_crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0u;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8u; ++bit) {
      crc = (uint8_t)((crc & 0x80u) ? (((unsigned)crc << 1) ^ 0xD5u) : ((unsigned)crc << 1));
    }
  }
  return crc;
}

gw_crsf_config_t gw_crsf_default_config(void)
{
  const gw_crsf_config_t config = {0u, 1u, 2u, 4u, 5u, 6u, 0u, GW_CRSF_CHANNEL_CENTER};
  return config;
}

bool gw_crsf_init(gw_crsf_t * receiver, const gw_crsf_config_t * config)
{
  memset(receiver, 0, sizeof(*receiver));
  if (
    config == NULL || config->failsafe_min > config->failsafe_max || config->failsafe_max > 2047u ||
    (config->failsafe_min == 0u && config->failsafe_max == 2047u)) {
    return false;
  }
  const uint8_t indices[] = {config->steering_channel, config->throttle_channel,
                             config->brake_channel,    config->auto_channel,
                             config->reset_channel,    config->failsafe_channel};
  for (size_t i = 0; i < sizeof(indices); ++i) {
    if (indices[i] >= GW_CRSF_CHANNEL_COUNT) {
      return false;
    }
    for (size_t j = 0; j < i; ++j) {
      if (indices[i] == indices[j]) {
        return false;
      }
    }
  }
  receiver->config = *config;
  receiver->configured = true;
  return true;
}

static bool timed_out(const gw_crsf_t * receiver, uint32_t now_ms)
{
  return !receiver->have_channels ||
         (uint32_t)(now_ms - receiver->last_channels_ms) >= GW_RC_TIMEOUT_MS;
}

static void accept_channels(gw_crsf_t * receiver, const uint8_t * payload, uint32_t now_ms)
{
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  uint32_t bits = 0u;
  unsigned available = 0u;
  size_t byte = 0u;
  for (size_t i = 0; i < GW_CRSF_CHANNEL_COUNT; ++i) {
    while (available < 11u) {
      bits |= (uint32_t)payload[byte++] << available;
      available += 8u;
    }
    channels[i] = (uint16_t)(bits & 0x7FFu);
    bits >>= 11;
    available -= 11u;
  }
  const gw_crsf_config_t * config = &receiver->config;
  uint16_t health = channels[config->failsafe_channel];
  receiver->failsafe = health >= config->failsafe_min && health <= config->failsafe_max;
  if (receiver->failsafe) {
    receiver->loss_pending = true;
    return;
  }
  memcpy(receiver->channels, channels, sizeof(channels));
  receiver->last_channels_ms = now_ms;
  receiver->have_channels = true;
  receiver->auto_switch = channels[config->auto_channel] > GW_CRSF_CHANNEL_CENTER;
  receiver->reset_switch = channels[config->reset_channel] > GW_CRSF_CHANNEL_CENTER;
}

static void discard(gw_crsf_t * receiver, size_t count)
{
  receiver->buffered -= count;
  memmove(receiver->buffer, receiver->buffer + count, receiver->buffered);
}

// Never look for a nested frame inside a plausible incomplete frame. Telemetry
// payloads are arbitrary bytes and can contain a whole channel frame. After
// CRC/length failure, or the assembly deadline, slide one byte and resynchronize.
static void drain(gw_crsf_t * receiver, uint32_t frame_ms, bool expired)
{
  while (receiver->buffered != 0u) {
    if (receiver->buffer[0] != CRSF_SYNC && receiver->buffer[0] != CRSF_BROADCAST) {
      discard(receiver, 1u);
      continue;
    }
    if (receiver->buffered < 2u) {
      if (expired) {
        discard(receiver, 1u);
        continue;
      }
      return;
    }
    size_t length = receiver->buffer[1];
    if (length < 2u || length > GW_CRSF_FRAME_MAX - 2u) {
      discard(receiver, 1u);
      continue;
    }
    size_t total = length + 2u;
    if (receiver->buffered < total) {
      if (expired) {
        discard(receiver, 1u);
        continue;
      }
      return;
    }
    if (gw_crsf_crc8(receiver->buffer + 2u, length - 1u) != receiver->buffer[total - 1u]) {
      discard(receiver, 1u);
      continue;
    }
    if (receiver->buffer[2] == CRSF_CHANNELS && length >= CRSF_CHANNEL_PAYLOAD + 2u) {
      accept_channels(receiver, receiver->buffer + 3u, frame_ms);
    }
    // Valid unknown frames and future channel extensions are consumed whole.
    discard(receiver, total);
  }
}

static void expire_partial(gw_crsf_t * receiver, uint32_t now_ms)
{
  if (
    receiver->buffered != 0u &&
    (uint32_t)(now_ms - receiver->frame_start_ms) >= GW_CRSF_FRAME_TIMEOUT_MS) {
    // A buffered complete frame recovered from noise keeps its arrival time.
    drain(receiver, receiver->last_byte_ms, true);
  }
}

size_t gw_crsf_feed(gw_crsf_t * receiver, const uint8_t * data, size_t len, uint32_t now_ms)
{
  if (len > GW_CRSF_READ_BUDGET) {
    len = GW_CRSF_READ_BUDGET;
  }
  if (receiver->have_channels && timed_out(receiver, now_ms)) {
    receiver->loss_pending = true;
  }
  expire_partial(receiver, now_ms);
  for (size_t i = 0; i < len; ++i) {
    if (!receiver->configured) {
      continue;
    }
    if (receiver->buffered == 0u) {
      receiver->frame_start_ms = now_ms;
    }
    receiver->buffer[receiver->buffered++] = data[i];
    receiver->last_byte_ms = now_ms;
    drain(receiver, now_ms, false);
  }
  return len;
}

static float normalized(uint16_t channel)
{
  if (channel <= GW_CRSF_CHANNEL_MIN) {
    return 0.0f;
  }
  if (channel >= GW_CRSF_CHANNEL_MAX) {
    return 1.0f;
  }
  return (float)(channel - GW_CRSF_CHANNEL_MIN) /
         (float)(GW_CRSF_CHANNEL_MAX - GW_CRSF_CHANNEL_MIN);
}

static float steering(uint16_t channel)
{
  if (channel < GW_CRSF_CHANNEL_CENTER) {
    float magnitude = (float)(GW_CRSF_CHANNEL_CENTER - channel) /
                      (float)(GW_CRSF_CHANNEL_CENTER - GW_CRSF_CHANNEL_MIN);
    return -(magnitude > 1.0f ? 1.0f : magnitude) * GW_STEER_ANGLE_MAX_RAD;
  }
  float magnitude = (float)(channel - GW_CRSF_CHANNEL_CENTER) /
                    (float)(GW_CRSF_CHANNEL_MAX - GW_CRSF_CHANNEL_CENTER);
  return (magnitude > 1.0f ? 1.0f : magnitude) * GW_STEER_ANGLE_MAX_RAD;
}

void gw_crsf_apply(gw_crsf_t * receiver, uint32_t now_ms, gw_inputs_t * inputs)
{
  expire_partial(receiver, now_ms);
  inputs->rc_link_ok = receiver->configured && !receiver->failsafe && !receiver->loss_pending &&
                       !timed_out(receiver, now_ms);
  inputs->tx_auto_switch = receiver->auto_switch;
  inputs->tx_reset = receiver->reset_switch;
  inputs->rc_steering = receiver->have_channels
                          ? steering(receiver->channels[receiver->config.steering_channel])
                          : 0.0f;
  inputs->rc_throttle =
    inputs->rc_link_ok ? normalized(receiver->channels[receiver->config.throttle_channel]) : 0.0f;
  inputs->rc_brake =
    inputs->rc_link_ok ? normalized(receiver->channels[receiver->config.brake_channel]) : 1.0f;
  receiver->loss_pending = false;
}
