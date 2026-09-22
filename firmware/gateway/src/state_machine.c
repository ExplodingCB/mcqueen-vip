#include "gateway/state_machine.h"

#include <math.h>
#include <string.h>

#include "gateway/config.h"
#include "gateway/limits.h"

const char * gw_mode_name(gw_mode_t mode)
{
  switch (mode) {
    case GW_MODE_INIT:
      return "INIT";
    case GW_MODE_RC:
      return "RC";
    case GW_MODE_AUTO:
      return "AUTO";
    case GW_MODE_URGENT_STOP:
      return "URGENT_STOP";
    case GW_MODE_DRIVETRAIN_OFF:
      return "DRIVETRAIN_OFF";
    case GW_MODE_FAULT:
      return "FAULT";
    default:
      return "?";
  }
}

void gw_init(gw_t * gw, uint32_t now_ms)
{
  memset(gw, 0, sizeof(*gw));
  gw->mode = GW_MODE_INIT;
  gw_heartbeat_reset(&gw->heartbeat);
  gw->last_tick_ms = now_ms;
  gw->out.mode = GW_MODE_INIT;
  gw->out.brake = 1.0f;
  gw->out.steering_enabled = false;
  gw->out.contactor_closed = false;
}

static void enter_urgent_stop(gw_t * gw, const gw_inputs_t * in, gw_faults_t faults)
{
  gw->mode = GW_MODE_URGENT_STOP;
  gw->urgent_entry_ms = in->now_ms;
  gw->urgent_brake_start = gw->out.brake;
  gw->urgent_from_estop = (faults & GW_FAULT_REMOTE_ESTOP) != 0u;
  gw->standstill_reached_ms = 0u;
  gw->auto_switch_armed = false;
}

// Command frame intake: CRC, range, heartbeat. Rejected frames never touch
// gw->cmd, so a misbehaving Jetson can only ever starve the heartbeat.
static void intake_command(gw_t * gw, const gw_inputs_t * in, gw_faults_t * faults)
{
  if (!in->command_received) {
    return;
  }
  gw_command_t cmd;
  if (!gw_command_unpack(in->command_frame, &cmd)) {
    *faults |= GW_FAULT_CRC;
    gw->out.frames_rejected++;
    return;
  }
  if (!gw_command_fields_valid(&cmd)) {
    *faults |= GW_FAULT_INVALID_FIELD;
    if (!(fabsf(cmd.steering_angle) <= GW_STEER_ANGLE_MAX_RAD)) {
      *faults |= GW_FAULT_STEER_ANGLE_LIMIT;
    }
    gw->out.frames_rejected++;
    return;
  }
  if (!gw_heartbeat_accept(&gw->heartbeat, cmd.heartbeat, in->now_ms)) {
    *faults |= GW_FAULT_HEARTBEAT_COUNTER;
    gw->out.frames_rejected++;
    return;
  }
  gw->cmd = cmd;
  gw->cmd_valid = true;
  gw->out.heartbeat_echo = cmd.heartbeat;
}

static void track_standstill(gw_t * gw, const gw_inputs_t * in)
{
  if (in->speed < GW_STANDSTILL_SPEED_M_S) {
    if (!gw->standstill && gw->standstill_since_ms == 0u) {
      gw->standstill_since_ms = in->now_ms == 0u ? 1u : in->now_ms;
    }
    if (
      !gw->standstill &&
      (uint32_t)(in->now_ms - gw->standstill_since_ms) >= GW_STANDSTILL_HOLD_MS) {
      gw->standstill = true;
      gw->standstill_reached_ms = in->now_ms;
    }
  } else {
    gw->standstill = false;
    gw->standstill_since_ms = 0u;
  }
}

static bool auto_entry_allowed(gw_t * gw, const gw_inputs_t * in, gw_faults_t faults)
{
  if (!in->rcs_auto_allowed) {
    return false;
  }
  if ((faults & GW_FAULTS_BLOCKING_AUTO) != 0u) {
    return false;
  }
  if (!gw->cmd_valid) {
    return false;
  }
  if (gw_heartbeat_continuous_ms(&gw->heartbeat, in->now_ms) < GW_HEARTBEAT_CONTINUOUS_MS) {
    return false;
  }
  if (in->speed >= GW_HANDOVER_SPEED_M_S) {
    return false;
  }
  return true;
}

static void drive_outputs(
  gw_t * gw, const gw_inputs_t * in, float dt_s, float steer_target, float throttle_target,
  float brake_target, float throttle_cap, gw_faults_t * faults)
{
  gw_outputs_t * out = &gw->out;
  float angle = gw_limit_steer_angle(steer_target, faults);
  out->steering = gw_limit_steer_rate(angle, out->steering, dt_s, faults);
  out->steering_enabled = true;
  float throttle = gw_limit_throttle_ramp(throttle_target, out->throttle, dt_s, throttle_cap);
  throttle = gw_limit_brake_throttle(throttle, brake_target, faults);
  throttle = gw_limit_speed(throttle, in->speed, faults);
  throttle = gw_limit_power(throttle, in->motor_power, faults);
  out->throttle = throttle;
  out->brake = brake_target < 0.0f ? 0.0f : (brake_target > 1.0f ? 1.0f : brake_target);
  out->contactor_closed = true;
}

void gw_step(gw_t * gw, const gw_inputs_t * in)
{
  float dt_s = (float)(uint32_t)(in->now_ms - gw->last_tick_ms) * 0.001f;
  gw->last_tick_ms = in->now_ms;
  gw_faults_t faults = 0u;

  intake_command(gw, in, &faults);
  bool hb_ok = gw_heartbeat_healthy(&gw->heartbeat, in->now_ms);
  if (!hb_ok) {
    gw->cmd_valid = false;
  }

  // Input-derived faults, independent of mode.
  if (!in->rc_link_ok) {
    faults |= GW_FAULT_RC_LINK_LOST;
  }
  if (in->remote_estop) {
    faults |= GW_FAULT_REMOTE_ESTOP;
  }
  if (gw->cmd_valid && gw->cmd.request_urgent_stop) {
    faults |= GW_FAULT_JETSON_REQUEST;
  }
  if (gw->mode == GW_MODE_AUTO && !hb_ok) {
    faults |= GW_FAULT_HEARTBEAT_TIMEOUT;
  }
  // Speed and power flags are needed before the transitions; the limiters set
  // the same bits again when applied to the real throttle below.
  (void)gw_limit_speed(1.0f, in->speed, &faults);
  (void)gw_limit_power(1.0f, in->motor_power, &faults);

  bool tx_reset_edge = in->rc_link_ok && in->tx_reset && !gw->prev_tx_reset;
  bool manual_reset_edge = in->manual_reset && !gw->prev_manual_reset;
  // Preserve the last trustworthy reset level across receiver loss.
  if (in->rc_link_ok) {
    gw->prev_tx_reset = in->tx_reset;
  }
  gw->prev_manual_reset = in->manual_reset;
  if (!in->rc_link_ok) {
    gw->auto_switch_armed = false;
  } else if (!in->tx_auto_switch) {
    gw->auto_switch_armed = true;
  }
  track_standstill(gw, in);

  // Transitions.
  switch (gw->mode) {
    case GW_MODE_INIT:
      if (in->self_test_ok) {
        if ((faults & GW_FAULTS_STOPPING) != 0u) {
          enter_urgent_stop(gw, in, faults);
        } else {
          gw->mode = GW_MODE_RC;
        }
      } else {
        faults |= GW_FAULT_SELF_TEST;
        gw->mode = GW_MODE_FAULT;
      }
      break;
    case GW_MODE_RC:
      if ((faults & GW_FAULTS_STOPPING) != 0u) {
        enter_urgent_stop(gw, in, faults);
      } else if (in->tx_auto_switch && gw->auto_switch_armed) {
        gw->auto_switch_armed = false;
        if (auto_entry_allowed(gw, in, faults)) {
          gw->mode = GW_MODE_AUTO;
        } else {
          faults |= GW_FAULT_HANDOVER_REFUSED;
        }
      }
      break;
    case GW_MODE_AUTO:
      if ((faults & GW_FAULTS_STOPPING) != 0u) {
        enter_urgent_stop(gw, in, faults);
      } else if (!in->tx_auto_switch) {
        gw->mode = GW_MODE_RC;
      }
      break;
    case GW_MODE_URGENT_STOP:
      if (in->remote_estop) {
        gw->urgent_from_estop = true;
      }
      if (gw->standstill) {
        if (gw->urgent_from_estop) {
          gw->mode = GW_MODE_DRIVETRAIN_OFF;
        } else if (tx_reset_edge && (faults & GW_FAULTS_STOPPING) == 0u) {
          gw->mode = GW_MODE_RC;
          gw->out.latched = 0u;
        } else if (
          (uint32_t)(in->now_ms - gw->standstill_reached_ms) >= GW_URGENT_RESET_TIMEOUT_MS) {
          faults |= GW_FAULT_RESET_TIMEOUT;
          gw->mode = GW_MODE_DRIVETRAIN_OFF;
        }
      }
      break;
    case GW_MODE_DRIVETRAIN_OFF:
    case GW_MODE_FAULT:
      if (manual_reset_edge) {
        gw->mode = GW_MODE_INIT;
        gw->out.latched = 0u;
      }
      break;
    default:
      gw->mode = GW_MODE_FAULT;
      break;
  }

  // Outputs for the mode we ended up in.
  gw_outputs_t * out = &gw->out;
  switch (gw->mode) {
    case GW_MODE_RC:
      drive_outputs(
        gw, in, dt_s, in->rc_steering, in->rc_throttle, in->rc_brake, GW_RC_THROTTLE_CAP, &faults);
      break;
    case GW_MODE_AUTO: {
      // With lat_enable or long_enable false the transmitter drives that axis,
      // which is how single-axis bring-up tests run.
      float steer = gw->cmd.lat_enable ? gw->cmd.steering_angle : in->rc_steering;
      float throttle = gw->cmd.long_enable ? gw->cmd.throttle : in->rc_throttle;
      float brake = gw->cmd.long_enable ? gw->cmd.brake : in->rc_brake;
      drive_outputs(gw, in, dt_s, steer, throttle, brake, 1.0f, &faults);
      break;
    }
    case GW_MODE_URGENT_STOP: {
      out->throttle = 0.0f;
      float elapsed = (float)(uint32_t)(in->now_ms - gw->urgent_entry_ms);
      float ramp = elapsed / (float)GW_URGENT_BRAKE_RAMP_MS;
      if (ramp > 1.0f) {
        ramp = 1.0f;
      }
      out->brake = gw->urgent_brake_start + (1.0f - gw->urgent_brake_start) * ramp;
      out->steering_enabled = true;
      if (in->speed <= GW_STEER_HOLD_SPEED_M_S) {
        float angle = gw_limit_steer_angle(in->rc_steering, &faults);
        out->steering = gw_limit_steer_rate(angle, out->steering, dt_s, &faults);
      }
      out->contactor_closed = !(gw->urgent_from_estop || gw->standstill);
      break;
    }
    case GW_MODE_INIT:
      out->throttle = 0.0f;
      out->brake = 1.0f;
      out->steering_enabled = false;
      out->contactor_closed = false;
      break;
    case GW_MODE_DRIVETRAIN_OFF:
    case GW_MODE_FAULT:
    default:
      out->throttle = 0.0f;
      out->brake = 1.0f;
      out->steering_enabled = false;
      out->contactor_closed = false;
      break;
  }

  out->mode = gw->mode;
  out->faults = faults;
  out->latched |= faults;
}
