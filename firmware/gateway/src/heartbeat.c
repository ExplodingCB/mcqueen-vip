#include "gateway/heartbeat.h"

#include "gateway/config.h"

void gw_heartbeat_reset(gw_heartbeat_t * hb)
{
  hb->synced = false;
  hb->last_counter = 0u;
  hb->last_accept_ms = 0u;
  hb->continuous_since_ms = 0u;
}

bool gw_heartbeat_accept(gw_heartbeat_t * hb, uint16_t counter, uint32_t now_ms)
{
  if (!hb->synced) {
    hb->synced = true;
    hb->last_counter = counter;
    hb->last_accept_ms = now_ms;
    hb->continuous_since_ms = now_ms;
    return true;
  }
  uint16_t advance = (uint16_t)(counter - hb->last_counter);
  if (advance < 1u || advance > GW_HEARTBEAT_MAX_SKIP) {
    return false;
  }
  hb->last_counter = counter;
  hb->last_accept_ms = now_ms;
  return true;
}

bool gw_heartbeat_healthy(gw_heartbeat_t * hb, uint32_t now_ms)
{
  if (!hb->synced) {
    return false;
  }
  if ((uint32_t)(now_ms - hb->last_accept_ms) > GW_HEARTBEAT_TIMEOUT_MS) {
    gw_heartbeat_reset(hb);
    return false;
  }
  return true;
}

uint32_t gw_heartbeat_continuous_ms(gw_heartbeat_t * hb, uint32_t now_ms)
{
  if (!gw_heartbeat_healthy(hb, now_ms)) {
    return 0u;
  }
  return (uint32_t)(now_ms - hb->continuous_since_ms);
}
