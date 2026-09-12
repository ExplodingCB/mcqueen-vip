#include "mcq/pid.h"

#include <math.h>

mcq_gain_schedule_t mcq_gain_constant(float gain)
{
  mcq_gain_schedule_t s = {0};
  s.n = 1;
  s.x[0] = 0.0f;
  s.y[0] = gain;
  return s;
}

float mcq_gain_lookup(const mcq_gain_schedule_t * schedule, float x)
{
  if (schedule->n <= 0) {
    return 0.0f;
  }
  if (schedule->n == 1 || x <= schedule->x[0]) {
    return schedule->y[0];
  }
  int last = schedule->n - 1;
  if (x >= schedule->x[last]) {
    return schedule->y[last];
  }
  for (int i = 0; i < last; ++i) {
    if (x <= schedule->x[i + 1]) {
      float span = schedule->x[i + 1] - schedule->x[i];
      float t = span > 0.0f ? (x - schedule->x[i]) / span : 0.0f;
      return schedule->y[i] + t * (schedule->y[i + 1] - schedule->y[i]);
    }
  }
  return schedule->y[last];
}

void mcq_pid_init(mcq_pid_t * pid, const mcq_pid_params_t * params)
{
  pid->params = *params;
  mcq_pid_reset(pid);
}

void mcq_pid_reset(mcq_pid_t * pid)
{
  pid->p = 0.0f;
  pid->i = 0.0f;
  pid->f = 0.0f;
  pid->control = 0.0f;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float mcq_pid_update(
  mcq_pid_t * pid, float error, float speed, float feedforward, bool freeze_integrator)
{
  const mcq_pid_params_t * p = &pid->params;
  float k_p = mcq_gain_lookup(&p->k_p, speed);
  float k_i = mcq_gain_lookup(&p->k_i, speed);

  pid->p = error * k_p;
  pid->f = feedforward * p->k_f;

  if (!freeze_integrator) {
    float i = pid->i + error * k_i * p->i_rate;
    // Anti-windup: keep the new integrator only if the total output stays
    // inside the limits, or if the change pulls the output back inside.
    float control = pid->p + i + pid->f;
    bool inside = control >= p->neg_limit && control <= p->pos_limit;
    bool unwinding =
      (control > p->pos_limit && error < 0.0f) || (control < p->neg_limit && error > 0.0f);
    if (inside || unwinding) {
      pid->i = i;
    }
    if (inside && p->i_unwind > 0.0f) {
      pid->i *= (1.0f - p->i_unwind);
    }
  }

  pid->control = clampf(pid->p + pid->i + pid->f, p->neg_limit, p->pos_limit);
  return pid->control;
}
