#include "check.h"
#include "gateway/config.h"
#include "gateway/heartbeat.h"

static int first_frame_syncs(void)
{
  gw_heartbeat_t hb;
  gw_heartbeat_reset(&hb);
  CHECK(!gw_heartbeat_healthy(&hb, 0));
  CHECK(gw_heartbeat_accept(&hb, 1234, 10));
  CHECK(gw_heartbeat_healthy(&hb, 10));
  CHECK(gw_heartbeat_continuous_ms(&hb, 60) == 50u);
  return 0;
}

static int accepts_small_advances_only(void)
{
  gw_heartbeat_t hb;
  gw_heartbeat_reset(&hb);
  CHECK(gw_heartbeat_accept(&hb, 100, 0));
  CHECK(gw_heartbeat_accept(&hb, 101, 10));
  CHECK(gw_heartbeat_accept(&hb, 101 + GW_HEARTBEAT_MAX_SKIP, 20));
  CHECK(!gw_heartbeat_accept(&hb, 101 + GW_HEARTBEAT_MAX_SKIP, 30));          // frozen
  CHECK(!gw_heartbeat_accept(&hb, 100, 40));                                  // replayed
  CHECK(!gw_heartbeat_accept(&hb, 101 + 2 * GW_HEARTBEAT_MAX_SKIP + 1, 50));  // too far
  CHECK(hb.last_counter == 101 + GW_HEARTBEAT_MAX_SKIP);
  return 0;
}

static int wraps_at_16_bits(void)
{
  gw_heartbeat_t hb;
  gw_heartbeat_reset(&hb);
  CHECK(gw_heartbeat_accept(&hb, 65535, 0));
  CHECK(gw_heartbeat_accept(&hb, 0, 10));
  CHECK(gw_heartbeat_accept(&hb, 1, 20));
  return 0;
}

static int timeout_resyncs(void)
{
  gw_heartbeat_t hb;
  gw_heartbeat_reset(&hb);
  CHECK(gw_heartbeat_accept(&hb, 5, 1000));
  CHECK(gw_heartbeat_healthy(&hb, 1000 + GW_HEARTBEAT_TIMEOUT_MS));
  CHECK(!gw_heartbeat_healthy(&hb, 1000 + GW_HEARTBEAT_TIMEOUT_MS + 1));
  CHECK(gw_heartbeat_continuous_ms(&hb, 2000) == 0u);
  // After the timeout any counter is accepted again and the continuous run restarts.
  CHECK(gw_heartbeat_accept(&hb, 4000, 2000));
  CHECK(gw_heartbeat_continuous_ms(&hb, 2010) == 10u);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(first_frame_syncs);
  RUN(accepts_small_advances_only);
  RUN(wraps_at_16_bits);
  RUN(timeout_resyncs);
  return failures ? 1 : 0;
}
