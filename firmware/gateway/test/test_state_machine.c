// Walks the fault-injection list from docs/04-safety.md section 7 on the host.
// A "jetson" helper sends a command frame every 10 ms; the gateway is stepped
// every millisecond.
#include <math.h>
#include <string.h>

#include "check.h"
#include "gateway/command.h"
#include "gateway/config.h"
#include "gateway/state_machine.h"

typedef struct
{
  gw_t gw;
  gw_inputs_t in;
  uint32_t now_ms;
  // Simulated Jetson.
  bool jetson_sending;
  bool jetson_freeze_counter;
  bool jetson_corrupt_every_10th;
  uint16_t jetson_counter;
  uint32_t jetson_frames_sent;
  gw_command_t jetson_cmd;
} rig_t;

static void rig_init(rig_t * r)
{
  memset(r, 0, sizeof(*r));
  r->now_ms = 1000;
  gw_init(&r->gw, r->now_ms);
  r->in.self_test_ok = true;
  r->in.rc_link_ok = true;
  r->in.rcs_auto_allowed = true;
  r->in.remote_estop = false;
  r->jetson_cmd.lat_enable = true;
  r->jetson_cmd.long_enable = true;
  r->jetson_counter = 100;
}

static void rig_tick(rig_t * r)
{
  r->now_ms += 1;
  r->in.now_ms = r->now_ms;
  r->in.command_received = false;
  if (r->jetson_sending && (r->now_ms % 10u) == 0u) {
    gw_command_t cmd = r->jetson_cmd;
    if (!r->jetson_freeze_counter) {
      r->jetson_counter++;
    }
    cmd.heartbeat = r->jetson_counter;
    gw_command_encode(&cmd, r->in.command_frame);
    r->jetson_frames_sent++;
    if (r->jetson_corrupt_every_10th && (r->jetson_frames_sent % 10u) == 0u) {
      r->in.command_frame[3] ^= 0x40u;  // flip a payload bit, keep the old CRC
    }
    r->in.command_received = true;
  }
  gw_step(&r->gw, &r->in);
}

static void rig_run(rig_t * r, uint32_t ms)
{
  for (uint32_t i = 0; i < ms; ++i) {
    rig_tick(r);
  }
}

static gw_mode_t mode(const rig_t * r) { return r->gw.out.mode; }

// Power on, pass the self test, start the heartbeat, wait, flip the switch.
static int bring_to_auto(rig_t * r)
{
  rig_init(r);
  rig_run(r, 5);
  CHECK(mode(r) == GW_MODE_RC);
  r->jetson_sending = true;
  rig_run(r, GW_HEARTBEAT_CONTINUOUS_MS + 100);
  r->in.tx_auto_switch = true;
  rig_run(r, 2);
  CHECK(mode(r) == GW_MODE_AUTO);
  return 0;
}

static int init_to_rc_and_fault(void)
{
  rig_t r;
  rig_init(&r);
  CHECK(mode(&r) == GW_MODE_INIT);
  CHECK(!r.gw.out.contactor_closed);
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  CHECK(r.gw.out.contactor_closed);
  CHECK(r.gw.out.steering_enabled);

  rig_init(&r);
  r.in.self_test_ok = false;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_FAULT);
  CHECK(r.gw.out.faults & GW_FAULT_SELF_TEST);
  CHECK(!r.gw.out.contactor_closed);
  CHECK_NEAR(r.gw.out.brake, 1.0f, 1e-6);
  r.in.self_test_ok = true;
  r.in.manual_reset = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_INIT);
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  return 0;
}

static int rc_passes_transmitter_through_limits(void)
{
  rig_t r;
  rig_init(&r);
  rig_run(&r, 1);
  r.in.rc_steering = 0.2f;
  r.in.rc_throttle = 1.0f;
  rig_run(&r, 1);
  // Rate and ramp limited on the first tick.
  CHECK_NEAR(r.gw.out.steering, GW_STEER_RATE_MAX_RAD_S * 0.001f, 1e-6);
  CHECK_NEAR(r.gw.out.throttle, GW_THROTTLE_RAMP_PER_S * 0.001f, 1e-6);
  rig_run(&r, 1000);
  CHECK_NEAR(r.gw.out.steering, 0.2f, 1e-5);
  CHECK_NEAR(r.gw.out.throttle, GW_RC_THROTTLE_CAP, 1e-5);
  r.in.rc_brake = 0.5f;
  rig_run(&r, 1);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  CHECK(r.gw.out.faults & GW_FAULT_BRAKE_THROTTLE_CONFLICT);
  return 0;
}

static int auto_entry_and_exit_by_switch(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.steering_angle = 0.1f;
  r.jetson_cmd.throttle = 0.3f;
  rig_run(&r, 500);
  CHECK_NEAR(r.gw.out.steering, 0.1f, 1e-5);
  CHECK_NEAR(r.gw.out.throttle, 0.3f, 1e-5);
  CHECK(r.gw.out.heartbeat_echo == r.jetson_counter);
  r.in.tx_auto_switch = false;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  // Back in RC the transmitter drives, not the Jetson.
  rig_run(&r, 200);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  return 0;
}

// Fault injection 11: flip to AUTO above the handover speed (must be refused).
static int auto_refused_above_handover_speed(void)
{
  rig_t r;
  rig_init(&r);
  rig_run(&r, 5);
  r.jetson_sending = true;
  rig_run(&r, GW_HEARTBEAT_CONTINUOUS_MS + 100);
  r.in.speed = GW_HANDOVER_SPEED_M_S + 0.5f;
  r.in.tx_auto_switch = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  CHECK(r.gw.out.faults & GW_FAULT_HANDOVER_REFUSED);
  // Slowing down with the switch still in AUTO must not enter AUTO by itself.
  r.in.speed = 0.0f;
  rig_run(&r, 500);
  CHECK(mode(&r) == GW_MODE_RC);
  // Re-arm by flipping back to RC, then AUTO.
  r.in.tx_auto_switch = false;
  rig_run(&r, 5);
  r.in.tx_auto_switch = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_AUTO);
  return 0;
}

// Fault injection 12: flip to AUTO with a stale heartbeat (must be refused).
static int auto_refused_with_stale_heartbeat(void)
{
  rig_t r;
  rig_init(&r);
  rig_run(&r, 5);
  r.in.tx_auto_switch = true;  // no Jetson at all
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  CHECK(r.gw.out.faults & GW_FAULT_HANDOVER_REFUSED);

  rig_init(&r);
  rig_run(&r, 5);
  r.jetson_sending = true;
  rig_run(&r, 300);  // healthy, but not yet continuous for one second
  r.in.tx_auto_switch = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  CHECK(r.gw.out.faults & GW_FAULT_HANDOVER_REFUSED);
  return 0;
}

// Fault injection 1 and 2: unplug the CAN cable or kill the Jetson in AUTO.
static int heartbeat_loss_stops_within_timeout(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.throttle = 0.5f;
  rig_run(&r, 400);
  r.jetson_sending = false;
  uint32_t t0 = r.now_ms;
  while (mode(&r) == GW_MODE_AUTO && r.now_ms - t0 < 200u) {
    rig_tick(&r);
  }
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.now_ms - t0 <= GW_HEARTBEAT_TIMEOUT_MS + 10u);
  CHECK(r.gw.out.latched & GW_FAULT_HEARTBEAT_TIMEOUT);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  return 0;
}

// Fault injection 3: freeze the heartbeat counter while continuing to send frames.
static int frozen_counter_stops_within_timeout(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_freeze_counter = true;
  uint32_t t0 = r.now_ms;
  while (mode(&r) == GW_MODE_AUTO && r.now_ms - t0 < 200u) {
    rig_tick(&r);
  }
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.now_ms - t0 <= GW_HEARTBEAT_TIMEOUT_MS + 10u);
  CHECK(r.gw.out.latched & GW_FAULT_HEARTBEAT_COUNTER);
  CHECK(r.gw.out.latched & GW_FAULT_HEARTBEAT_TIMEOUT);
  CHECK(r.gw.out.frames_rejected >= 4);
  return 0;
}

// Fault injection 4: steering beyond the angle limit is rejected, beyond the
// rate limit is slewed.
static int steering_limits(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.steering_angle = GW_STEER_ANGLE_MAX_RAD + 0.05f;
  uint32_t t0 = r.now_ms;
  while (mode(&r) == GW_MODE_AUTO && r.now_ms - t0 < 200u) {
    rig_tick(&r);
  }
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.gw.out.latched & GW_FAULT_STEER_ANGLE_LIMIT);
  CHECK(r.gw.out.latched & GW_FAULT_INVALID_FIELD);
  CHECK(fabsf(r.gw.out.steering) <= GW_STEER_ANGLE_MAX_RAD);

  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.steering_angle = 0.4f;  // a step: must be slewed at the rate limit
  rig_run(&r, 50);
  CHECK(mode(&r) == GW_MODE_AUTO);
  // The step reaches the gateway with the next frame, up to 10 ms later.
  CHECK(r.gw.out.steering <= GW_STEER_RATE_MAX_RAD_S * 0.050f + 1e-4f);
  CHECK(r.gw.out.steering >= GW_STEER_RATE_MAX_RAD_S * 0.040f - 1e-4f);
  CHECK(r.gw.out.latched & GW_FAULT_STEER_RATE_LIMIT);
  rig_run(&r, 200);
  CHECK_NEAR(r.gw.out.steering, 0.4f, 1e-5);
  return 0;
}

// Fault injection 5: out-of-range values in every field (the CAN equivalent of NaN).
static int garbage_fields_rejected(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.steering_angle = 3.0f;
  r.jetson_cmd.throttle = 5.0f;
  r.jetson_cmd.brake = -5.0f;
  uint32_t t0 = r.now_ms;
  while (mode(&r) == GW_MODE_AUTO && r.now_ms - t0 < 200u) {
    rig_tick(&r);
  }
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.gw.out.latched & GW_FAULT_INVALID_FIELD);
  return 0;
}

// Fault injection 6: throttle and full brake together.
static int throttle_with_brake_is_cut(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.throttle = 1.0f;
  r.jetson_cmd.brake = 1.0f;
  rig_run(&r, 100);
  CHECK(mode(&r) == GW_MODE_AUTO);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  CHECK_NEAR(r.gw.out.brake, 1.0f, 1e-6);
  CHECK(r.gw.out.faults & GW_FAULT_BRAKE_THROTTLE_CONFLICT);
  return 0;
}

// Fault injection 7: speed above the cap, then above cap plus margin.
static int speed_cap_and_margin(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.throttle = 1.0f;
  rig_run(&r, 600);
  CHECK_NEAR(r.gw.out.throttle, 1.0f, 1e-5);
  r.in.speed = GW_SPEED_CAP_M_S + 0.2f;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_AUTO);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  CHECK(r.gw.out.faults & GW_FAULT_SPEED_CAP);
  r.in.speed = GW_SPEED_CAP_M_S + GW_SPEED_CAP_MARGIN_M_S + 0.2f;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.gw.out.latched & GW_FAULT_SPEED_CAP_MARGIN);
  return 0;
}

static int power_cap_cuts_throttle(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.throttle = 1.0f;
  rig_run(&r, 600);
  r.in.motor_power = GW_POWER_CAP_W + 100.0f;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_AUTO);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  CHECK(r.gw.out.faults & GW_FAULT_POWER_CAP);
  return 0;
}

// Fault injection 8: transmitter off in RC and in AUTO.
static int rc_link_loss_stops(void)
{
  rig_t r;
  rig_init(&r);
  rig_run(&r, 5);
  r.in.rc_link_ok = false;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.gw.out.latched & GW_FAULT_RC_LINK_LOST);

  CHECK(bring_to_auto(&r) == 0);
  r.in.rc_link_ok = false;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  return 0;
}

// Fault injection 9: remote e-stop in AUTO. Contactor opens at once, brakes
// ramp, DRIVETRAIN_OFF at standstill, manual reset required.
static int remote_estop_sequence(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.in.speed = 8.0f;
  r.in.remote_estop = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(!r.gw.out.contactor_closed);
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  rig_run(&r, GW_URGENT_BRAKE_RAMP_MS);
  CHECK_NEAR(r.gw.out.brake, 1.0f, 1e-5);
  r.in.speed = 0.0f;
  rig_run(&r, GW_STANDSTILL_HOLD_MS + 2);
  CHECK(mode(&r) == GW_MODE_DRIVETRAIN_OFF);
  CHECK(!r.gw.out.steering_enabled);
  // A transmitter reset is not enough after an e-stop.
  r.in.remote_estop = false;
  r.in.tx_reset = true;
  rig_run(&r, 5);
  CHECK(mode(&r) == GW_MODE_DRIVETRAIN_OFF);
  r.in.manual_reset = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_INIT);
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  return 0;
}

static int jetson_request_stops(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.request_urgent_stop = true;
  rig_run(&r, 12);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(r.gw.out.latched & GW_FAULT_JETSON_REQUEST);
  return 0;
}

// Urgent stop shape: brake ramp over 300 ms, steering held above 3 m/s and
// released to the transmitter below, operator reset at standstill returns to RC.
static int urgent_stop_ramp_hold_and_reset(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.steering_angle = 0.2f;
  r.jetson_cmd.throttle = 0.5f;
  rig_run(&r, 500);
  r.in.speed = 6.0f;
  r.in.rc_steering = -0.2f;
  r.jetson_sending = false;
  rig_run(&r, GW_HEARTBEAT_TIMEOUT_MS + 2);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  uint32_t t_entry = r.gw.urgent_entry_ms;
  CHECK_NEAR(r.gw.out.throttle, 0.0f, 1e-6);
  CHECK(r.gw.out.contactor_closed);
  // Halfway through the ramp the brake is about half.
  while (r.now_ms < t_entry + GW_URGENT_BRAKE_RAMP_MS / 2u) {
    rig_tick(&r);
  }
  CHECK_NEAR(r.gw.out.brake, 0.5f, 0.02);
  CHECK_NEAR(r.gw.out.steering, 0.2f, 1e-5);  // held above 3 m/s
  while (r.now_ms < t_entry + GW_URGENT_BRAKE_RAMP_MS + 5u) {
    rig_tick(&r);
  }
  CHECK_NEAR(r.gw.out.brake, 1.0f, 1e-6);
  r.in.speed = 2.0f;
  rig_run(&r, 400);
  CHECK_NEAR(r.gw.out.steering, -0.2f, 1e-5);  // released to the transmitter
  r.in.speed = 0.0f;
  rig_run(&r, GW_STANDSTILL_HOLD_MS + 2);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  CHECK(!r.gw.out.contactor_closed);
  CHECK(r.gw.out.latched & GW_FAULT_HEARTBEAT_TIMEOUT);
  r.in.tx_reset = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  CHECK(r.gw.out.latched == 0u);
  CHECK(r.gw.out.contactor_closed);
  return 0;
}

static int urgent_stop_without_reset_opens_contactor(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_sending = false;
  rig_run(&r, GW_HEARTBEAT_TIMEOUT_MS + 2);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  rig_run(&r, GW_STANDSTILL_HOLD_MS + GW_URGENT_RESET_TIMEOUT_MS + 5);
  CHECK(mode(&r) == GW_MODE_DRIVETRAIN_OFF);
  CHECK(r.gw.out.latched & GW_FAULT_RESET_TIMEOUT);
  return 0;
}

// Fault injection 13: corrupt the CRC of every tenth frame. Rejected frames are
// counted and the session continues, because a single lost frame is not a fault.
static int corrupted_frames_are_dropped_not_fatal(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_corrupt_every_10th = true;
  uint32_t sent_before = r.jetson_frames_sent;
  uint8_t rejected_before = r.gw.out.frames_rejected;
  rig_run(&r, 2000);
  CHECK(mode(&r) == GW_MODE_AUTO);
  uint32_t sent = r.jetson_frames_sent - sent_before;
  CHECK((uint8_t)(r.gw.out.frames_rejected - rejected_before) == (uint8_t)(sent / 10u));
  CHECK(r.gw.out.latched & GW_FAULT_CRC);
  CHECK(!(r.gw.out.latched & GW_FAULT_HEARTBEAT_TIMEOUT));
  return 0;
}

// After any exit from AUTO the switch must be cycled through RC to re-enter.
static int no_automatic_reentry(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_sending = false;
  rig_run(&r, GW_HEARTBEAT_TIMEOUT_MS + 2);
  CHECK(mode(&r) == GW_MODE_URGENT_STOP);
  r.jetson_sending = true;
  rig_run(&r, GW_STANDSTILL_HOLD_MS + 2);
  r.in.tx_reset = true;
  rig_run(&r, 1);
  CHECK(mode(&r) == GW_MODE_RC);
  rig_run(&r, GW_HEARTBEAT_CONTINUOUS_MS + 500);  // switch still in AUTO position
  CHECK(mode(&r) == GW_MODE_RC);
  return 0;
}

// Single-axis bring-up: lat_enable false means the transmitter steers in AUTO.
static int lat_disabled_follows_transmitter(void)
{
  rig_t r;
  CHECK(bring_to_auto(&r) == 0);
  r.jetson_cmd.lat_enable = false;
  r.jetson_cmd.steering_angle = 0.3f;
  r.jetson_cmd.throttle = 0.2f;
  r.in.rc_steering = -0.1f;
  rig_run(&r, 300);
  CHECK(mode(&r) == GW_MODE_AUTO);
  CHECK_NEAR(r.gw.out.steering, -0.1f, 1e-5);
  CHECK_NEAR(r.gw.out.throttle, 0.2f, 1e-5);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(init_to_rc_and_fault);
  RUN(rc_passes_transmitter_through_limits);
  RUN(auto_entry_and_exit_by_switch);
  RUN(auto_refused_above_handover_speed);
  RUN(auto_refused_with_stale_heartbeat);
  RUN(heartbeat_loss_stops_within_timeout);
  RUN(frozen_counter_stops_within_timeout);
  RUN(steering_limits);
  RUN(garbage_fields_rejected);
  RUN(throttle_with_brake_is_cut);
  RUN(speed_cap_and_margin);
  RUN(power_cap_cuts_throttle);
  RUN(rc_link_loss_stops);
  RUN(remote_estop_sequence);
  RUN(jetson_request_stops);
  RUN(urgent_stop_ramp_hold_and_reset);
  RUN(urgent_stop_without_reset_opens_contactor);
  RUN(corrupted_frames_are_dropped_not_fatal);
  RUN(no_automatic_reentry);
  RUN(lat_disabled_follows_transmitter);
  return failures ? 1 : 0;
}
