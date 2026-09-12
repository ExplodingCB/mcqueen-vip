#include "mcq/limits.h"

#include <math.h>

float mcq_clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float mcq_rate_limit(float target, float prev, float rate_max, float dt)
{
  float max_step = rate_max * dt;
  return mcq_clampf(target, prev - max_step, prev + max_step);
}

float mcq_limit_curvature(
  const mcq_lateral_limits_t * lim, float curvature, float prev_curvature, float speed, float dt)
{
  float v = fmaxf(speed, lim->v_min);
  float v2 = v * v;
  float kappa_max = lim->a_lat_max / v2;
  float dkappa_max = lim->jerk_lat_max / v2 * dt;
  float out = mcq_clampf(curvature, -kappa_max, kappa_max);
  out = mcq_clampf(out, prev_curvature - dkappa_max, prev_curvature + dkappa_max);
  return out;
}
