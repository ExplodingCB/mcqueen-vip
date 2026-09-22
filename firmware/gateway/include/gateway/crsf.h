// Receiver-side CRSF input. Feed bytes from a nonblocking UART read, then call
// gw_crsf_apply() before every gw_step(), including ticks with no UART data.
// No allocation, bitfields, packed structs, board headers or blocking I/O.
#ifndef GATEWAY_CRSF_H
#define GATEWAY_CRSF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gateway/state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GW_CRSF_CHANNEL_COUNT 16u
#define GW_CRSF_FRAME_MAX 64u
#define GW_CRSF_READ_BUDGET 512u
#define GW_CRSF_CHANNEL_MIN 172u
#define GW_CRSF_CHANNEL_CENTER 992u
#define GW_CRSF_CHANNEL_MAX 1811u

// Board configuration, not Jetson-writable. Indices are zero-based and must
// be distinct. A dedicated failsafe channel is mandatory: its configured
// inclusive range means link loss, never a control or reset request. CRSF
// 0x16 has no failsafe flag; this is a receiver/transmitter setup contract.
typedef struct
{
  uint8_t steering_channel;
  uint8_t throttle_channel;
  uint8_t brake_channel;
  uint8_t auto_channel;
  uint8_t reset_channel;
  uint8_t failsafe_channel;
  uint16_t failsafe_min;
  uint16_t failsafe_max;
} gw_crsf_config_t;

typedef struct
{
  gw_crsf_config_t config;
  bool configured;
  uint8_t buffer[GW_CRSF_FRAME_MAX];
  size_t buffered;
  uint32_t frame_start_ms;
  uint32_t last_byte_ms;
  uint32_t last_channels_ms;  // only healthy, complete, CRC-valid channel frames
  bool have_channels;
  bool failsafe;
  bool loss_pending;  // retain a failure even if it recovers before the next tick
  bool auto_switch;   // last actual healthy switch position, never failsafe data
  bool reset_switch;
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
} gw_crsf_t;

// CH1 steering, CH2 throttle, CH3 brake, CH5 AUTO, CH6 reset, CH7 health.
// Health must be held high during operation and set low on configured failsafe.
gw_crsf_config_t gw_crsf_default_config(void);

// Invalid or NULL configuration leaves the parser fail-closed and returns false.
bool gw_crsf_init(gw_crsf_t * receiver, const gw_crsf_config_t * config);

// CRC-8 DVB-S2: polynomial 0xD5, init 0, xorout 0, no reflection.
// Covers type + payload, unlike the CAN command's SAE J1850 CRC.
uint8_t gw_crsf_crc8(const uint8_t * data, size_t len);

// Returns bytes consumed, at most GW_CRSF_READ_BUDGET per call. Pass the rest
// on later ticks; do not busy-wait for data. now_ms is the monotonic arrival
// time, with unsigned wraparound. Feed/apply must run on the same task.
size_t gw_crsf_feed(gw_crsf_t * receiver, const uint8_t * data, size_t len, uint32_t now_ms);

// Writes only rc_link_ok, rc_* and tx_* into the caller's current inputs.
// Sets throttle=0, brake=1 and preserves the last healthy AUTO position on
// loss. Failsafe values never become switch events. Call once per control tick.
void gw_crsf_apply(gw_crsf_t * receiver, uint32_t now_ms, gw_inputs_t * inputs);

#ifdef __cplusplus
}
#endif

#endif  // GATEWAY_CRSF_H
