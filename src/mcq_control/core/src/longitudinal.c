#include "mcq/longitudinal.h"

#include "mcq/limits.h"

void mcq_long_init(mcq_long_t * lc, const mcq_long_params_t * params)
{
  lc->params = *params;
  mcq_pid_init(&lc->pid, &params->pid);
  lc->state = MCQ_LONG_OFF;
  lc->accel = 0.0f;
}

float mcq_long_update(
  mcq_long_t * lc, bool enabled, bool stop_requested, float v, float v_target, float a_target)
{
  const mcq_long_params_t * p = &lc->params;

  if (!enabled) {
    lc->state = MCQ_LONG_OFF;
  } else if (lc->state == MCQ_LONG_OFF) {
    lc->state = MCQ_LONG_PID;
    mcq_pid_reset(&lc->pid);
  }

  if (lc->state != MCQ_LONG_OFF) {
    bool want_stop = stop_requested && v < p->stopping_speed;
    if (lc->state == MCQ_LONG_PID && want_stop) {
      lc->state = MCQ_LONG_STOPPING;
    } else if (lc->state == MCQ_LONG_STOPPING && !stop_requested) {
      lc->state = MCQ_LONG_PID;
      mcq_pid_reset(&lc->pid);
    }
  }

  float accel;
  switch (lc->state) {
    case MCQ_LONG_OFF:
      mcq_pid_reset(&lc->pid);
      accel = 0.0f;
      break;
    case MCQ_LONG_STOPPING:
      // Hold a constant deceleration to a standstill; integrators stay reset.
      mcq_pid_reset(&lc->pid);
      accel = p->stopping_decel;
      break;
    case MCQ_LONG_PID:
    default: {
      float target_v = stop_requested ? 0.0f : v_target;
      float ff = stop_requested ? p->stopping_decel : a_target;
      accel = mcq_pid_update(&lc->pid, target_v - v, v, ff, false);
      break;
    }
  }
  lc->accel = mcq_clampf(accel, p->decel_max, p->accel_max);
  return lc->accel;
}

void mcq_actuator_map(const mcq_actuator_map_t * m, float accel, float * throttle, float * brake)
{
  // With both pedals released the kart decelerates by rolling_decel, so the
  // pedals must supply accel + rolling_decel. Inside the deadband we coast.
  float net = accel + m->rolling_decel;
  *throttle = 0.0f;
  *brake = 0.0f;
  if (net > m->deadband) {
    *throttle = mcq_clampf(net * m->throttle_per_accel, 0.0f, 1.0f);
  } else if (net < -m->deadband) {
    *brake = mcq_clampf(-net * m->brake_per_decel, 0.0f, 1.0f);
  }
}
