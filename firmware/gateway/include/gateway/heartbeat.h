// Heartbeat counter tracking for the Jetson command stream.
//
// The Jetson increments a 16-bit counter every frame. After a resync (first
// frame after enable, or after a timeout) any value is accepted; from then on
// an advance of 1..GW_HEARTBEAT_MAX_SKIP is accepted so a single lost or
// corrupted frame does not end the session, and anything else (frozen,
// repeated, or replayed counters) is rejected. GW_HEARTBEAT_TIMEOUT_MS without
// an accepted frame marks the heartbeat unhealthy and forces a resync.
#ifndef GATEWAY_HEARTBEAT_H
#define GATEWAY_HEARTBEAT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool synced;
  uint16_t last_counter;
  uint32_t last_accept_ms;
  uint32_t continuous_since_ms;
} gw_heartbeat_t;

void gw_heartbeat_reset(gw_heartbeat_t * hb);

// Returns true and records the frame when the counter is acceptable.
bool gw_heartbeat_accept(gw_heartbeat_t * hb, uint16_t counter, uint32_t now_ms);

// True while an accepted frame arrived within the timeout. Calling this after
// the timeout also resets sync, so the next frame starts a new continuous run.
bool gw_heartbeat_healthy(gw_heartbeat_t * hb, uint32_t now_ms);

// Milliseconds of uninterrupted healthy heartbeat, 0 when unhealthy.
uint32_t gw_heartbeat_continuous_ms(gw_heartbeat_t * hb, uint32_t now_ms);

#endif  // GATEWAY_HEARTBEAT_H
