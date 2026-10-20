// Raw receiver bytes through the parser, gateway state machine and actuators.
// Fault injection mapping: docs/04-safety.md section 7 items 8, 11 and 12.
#include <string.h>

#include "check.h"
#include "crsf_test.h"
#include "gateway/command.h"
#include "gateway/config.h"

typedef struct
{
  gw_t gw;
  gw_crsf_t rx;
  gw_inputs_t in;
  uint16_t channels[GW_CRSF_CHANNEL_COUNT];
  bool rc_sending;
  bool jetson_sending;
  uint16_t heartbeat;
  uint32_t now_ms;
} rig_t;

static void rig_init(rig_t * rig)
{
  memset(rig, 0, sizeof(*rig));
  gw_init(&rig->gw, 0u);
  gw_crsf_config_t config = gw_crsf_default_config();
  (void)gw_crsf_init(&rig->rx, &config);
  test_channels(rig->channels);
  rig->rc_sending = true;
  rig->jetson_sending = true;
  rig->in.self_test_ok = true;
  rig->in.rcs_auto_allowed = true;
}

static void tick(rig_t * rig)
{
  rig->now_ms += 10u;
  rig->in.now_ms = rig->now_ms;
  if (rig->rc_sending) {
    test_send(&rig->rx, rig->channels, rig->now_ms);
  }
  rig->in.command_received = rig->jetson_sending;
  if (rig->jetson_sending) {
    gw_command_t cmd = {0};
    cmd.heartbeat = ++rig->heartbeat;
    cmd.lat_enable = true;
    cmd.long_enable = true;
    cmd.throttle = 0.5f;
    gw_command_encode(&cmd, rig->in.command_frame);
  }
  gw_crsf_apply(&rig->rx, rig->now_ms, &rig->in);
  gw_step(&rig->gw, &rig->in);
}

static void run(rig_t * rig, uint32_t duration_ms)
{
  for (uint32_t elapsed = 0u; elapsed < duration_ms; elapsed += 10u) {
    tick(rig);
  }
}

static int ready(rig_t * rig, bool autonomous)
{
  rig_init(rig);
  run(rig, GW_HEARTBEAT_CONTINUOUS_MS + 100u);
  CHECK(rig->gw.mode == GW_MODE_RC);
  if (autonomous) {
    rig->channels[4] = GW_CRSF_CHANNEL_MAX;
    tick(rig);
    CHECK(rig->gw.mode == GW_MODE_AUTO);
  }
  return 0;
}

static int rc_passthrough_uses_gateway_limits(void)
{
  rig_t rig;
  CHECK(ready(&rig, false) == 0);
  rig.channels[0] = GW_CRSF_CHANNEL_MAX;
  rig.channels[1] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  CHECK_NEAR(rig.gw.out.steering, GW_STEER_RATE_MAX_RAD_S * 0.01f, 1e-6);
  CHECK_NEAR(rig.gw.out.throttle, GW_THROTTLE_RAMP_PER_S * 0.01f, 1e-6);
  run(&rig, 600u);
  CHECK_NEAR(rig.gw.out.steering, GW_STEER_ANGLE_MAX_RAD, 1e-6);
  CHECK_NEAR(rig.gw.out.throttle, GW_RC_THROTTLE_CAP, 1e-6);
  rig.channels[2] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  CHECK_NEAR(rig.gw.out.brake, 1.0f, 1e-6);
  CHECK_NEAR(rig.gw.out.throttle, 0.0f, 1e-6);
  CHECK(rig.gw.out.faults & GW_FAULT_BRAKE_THROTTLE_CONFLICT);
  return 0;
}

// Item 8: transmitter power-off, receiver stops channel frames, RC and AUTO.
static int item_8_timeout_in_rc_and_auto(void)
{
  for (unsigned autonomous = 0u; autonomous < 2u; ++autonomous) {
    rig_t rig;
    CHECK(ready(&rig, autonomous != 0u) == 0);
    rig.channels[1] = GW_CRSF_CHANNEL_MAX;
    run(&rig, 100u);
    CHECK(rig.gw.out.throttle > 0.0f);
    rig.rc_sending = false;
    run(&rig, GW_RC_TIMEOUT_MS - 10u);
    CHECK(rig.gw.mode == (autonomous ? GW_MODE_AUTO : GW_MODE_RC));
    tick(&rig);
    CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);
    CHECK(rig.gw.out.latched & GW_FAULT_RC_LINK_LOST);
    CHECK_NEAR(rig.gw.out.throttle, 0.0f, 0.0f);
    run(&rig, GW_URGENT_BRAKE_RAMP_MS);
    CHECK_NEAR(rig.gw.out.brake, 1.0f, 1e-6);
  }
  return 0;
}

// Item 8: receiver sends its configured failsafe channels instead of silence.
static int item_8_configured_failsafe_in_rc_and_auto(void)
{
  for (unsigned autonomous = 0u; autonomous < 2u; ++autonomous) {
    rig_t rig;
    CHECK(ready(&rig, autonomous != 0u) == 0);
    rig.channels[6] = GW_CRSF_CHANNEL_MIN;
    rig.channels[4] = GW_CRSF_CHANNEL_MIN;
    rig.channels[5] = GW_CRSF_CHANNEL_MAX;
    rig.channels[1] = GW_CRSF_CHANNEL_MAX;
    tick(&rig);
    CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);
    CHECK(rig.gw.out.latched & GW_FAULT_RC_LINK_LOST);
    CHECK_NEAR(rig.gw.out.throttle, 0.0f, 0.0f);
    CHECK(!rig.gw.auto_switch_armed);
    run(&rig, 1000u);
    CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);  // failsafe reset cannot release stop
  }
  return 0;
}

// Item 11: valid wire switch transition at and above the handover limit.
static int item_11_auto_refused_at_handover_speed(void)
{
  for (unsigned above = 0u; above < 2u; ++above) {
    rig_t rig;
    CHECK(ready(&rig, false) == 0);
    rig.in.speed = GW_HANDOVER_SPEED_M_S + (above ? 0.5f : 0.0f);
    rig.channels[4] = GW_CRSF_CHANNEL_MAX;
    tick(&rig);
    CHECK(rig.gw.mode == GW_MODE_RC);
    CHECK(rig.gw.out.faults & GW_FAULT_HANDOVER_REFUSED);
    rig.in.speed = 0.0f;
    run(&rig, 100u);
    CHECK(rig.gw.mode == GW_MODE_RC);
    rig.channels[4] = GW_CRSF_CHANNEL_MIN;
    tick(&rig);
    rig.channels[4] = GW_CRSF_CHANNEL_MAX;
    tick(&rig);
    CHECK(rig.gw.mode == GW_MODE_AUTO);
  }
  return 0;
}

// Item 12: a once-healthy Jetson heartbeat expires while RC input stays fresh.
static int item_12_auto_refused_with_stale_heartbeat(void)
{
  rig_t rig;
  CHECK(ready(&rig, false) == 0);
  rig.jetson_sending = false;
  run(&rig, GW_HEARTBEAT_TIMEOUT_MS);
  rig.channels[4] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  CHECK(rig.gw.mode == GW_MODE_RC);
  CHECK(rig.gw.out.faults & GW_FAULT_HANDOVER_REFUSED);
  rig.jetson_sending = true;
  run(&rig, GW_HEARTBEAT_CONTINUOUS_MS + 100u);
  CHECK(rig.gw.mode == GW_MODE_RC);
  rig.channels[4] = GW_CRSF_CHANNEL_MIN;
  tick(&rig);
  rig.channels[4] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  CHECK(rig.gw.mode == GW_MODE_AUTO);
  return 0;
}

static int auto_held_across_loss_requires_real_rc_cycle(void)
{
  for (unsigned silence = 0u; silence < 2u; ++silence) {
    rig_t rig;
    CHECK(ready(&rig, true) == 0);
    if (silence) {
      rig.rc_sending = false;
      run(&rig, GW_RC_TIMEOUT_MS);
    } else {
      // The physical switch stays AUTO, but the receiver emits an RC fallback.
      rig.channels[6] = GW_CRSF_CHANNEL_MIN;
      rig.channels[4] = GW_CRSF_CHANNEL_MIN;
      tick(&rig);
    }
    CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);
    run(&rig, GW_STANDSTILL_HOLD_MS + 20u);
    rig.rc_sending = true;
    rig.channels[6] = GW_CRSF_CHANNEL_MAX;
    rig.channels[4] = GW_CRSF_CHANNEL_MAX;
    rig.channels[5] = GW_CRSF_CHANNEL_MAX;
    // On timeout recovery, a loss pending tick is deliberately retained.
    tick(&rig);
    tick(&rig);
    CHECK(rig.gw.mode == GW_MODE_RC);
    run(&rig, 100u);
    CHECK(rig.gw.mode == GW_MODE_RC);
    rig.channels[4] = GW_CRSF_CHANNEL_MIN;
    tick(&rig);
    rig.channels[4] = GW_CRSF_CHANNEL_MAX;
    tick(&rig);
    CHECK(rig.gw.mode == GW_MODE_AUTO);
  }
  return 0;
}

static int simultaneous_loss_and_rc_switch_cannot_bypass_stop(void)
{
  rig_t rig;
  CHECK(ready(&rig, true) == 0);
  // Failsafe and recovery in the same UART batch. Real RC low is available,
  // but the retained failure must take precedence over the switch transition.
  rig.channels[6] = GW_CRSF_CHANNEL_MIN;
  test_send(&rig.rx, rig.channels, rig.now_ms + 10u);
  rig.channels[6] = GW_CRSF_CHANNEL_MAX;
  rig.channels[4] = GW_CRSF_CHANNEL_MIN;
  tick(&rig);
  CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);
  CHECK(!rig.gw.auto_switch_armed);
  return 0;
}

static int boot_without_receiver_and_reset_during_fault_stay_safe(void)
{
  rig_t rig;
  rig_init(&rig);
  rig.rc_sending = false;
  tick(&rig);
  CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);
  CHECK_NEAR(rig.gw.out.throttle, 0.0f, 0.0f);
  CHECK(!rig.gw.auto_switch_armed);
  CHECK(ready(&rig, true) == 0);
  rig.channels[5] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  rig.channels[6] = GW_CRSF_CHANNEL_MIN;
  rig.channels[5] = GW_CRSF_CHANNEL_MIN;  // synthetic low must not reset edge tracking
  tick(&rig);
  run(&rig, 600u);
  rig.channels[6] = GW_CRSF_CHANNEL_MAX;
  rig.channels[5] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  CHECK(rig.gw.mode == GW_MODE_URGENT_STOP);
  rig.channels[5] = GW_CRSF_CHANNEL_MIN;
  tick(&rig);
  rig.channels[5] = GW_CRSF_CHANNEL_MAX;
  tick(&rig);
  CHECK(rig.gw.mode == GW_MODE_RC);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(rc_passthrough_uses_gateway_limits);
  RUN(item_8_timeout_in_rc_and_auto);
  RUN(item_8_configured_failsafe_in_rc_and_auto);
  RUN(item_11_auto_refused_at_handover_speed);
  RUN(item_12_auto_refused_with_stale_heartbeat);
  RUN(auto_held_across_loss_requires_real_rc_cycle);
  RUN(simultaneous_loss_and_rc_switch_cannot_bypass_stop);
  RUN(boot_without_receiver_and_reset_during_fault_stay_safe);
  return failures ? 1 : 0;
}
