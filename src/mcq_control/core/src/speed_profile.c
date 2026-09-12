#include "mcq/speed_profile.h"

#include <math.h>
#include <stddef.h>

void mcq_speed_profile(
  const mcq_speed_profile_params_t * p, const float * s, const float * kappa, const float * v_ref,
  int n, float v0, float v_end, float * v_out)
{
  if (n <= 0) {
    return;
  }
  for (int i = 0; i < n; ++i) {
    float k = fabsf(kappa[i]);
    float v = p->v_cap;
    if (k > 1e-6f) {
      float v_curv = sqrtf(p->a_lat_max / k);
      if (v_curv < v) {
        v = v_curv;
      }
    }
    if (v_ref != NULL && v_ref[i] < v) {
      v = v_ref[i];
    }
    if (v < p->v_min) {
      v = p->v_min;
    }
    v_out[i] = v;
  }
  // Forward pass: cannot accelerate faster than a_long_max from v0.
  if (v_out[0] > v0) {
    v_out[0] = fmaxf(v0, p->v_min);
  }
  for (int i = 1; i < n; ++i) {
    float ds = fmaxf(s[i] - s[i - 1], 0.0f);
    float reachable = sqrtf(v_out[i - 1] * v_out[i - 1] + 2.0f * p->a_long_max * ds);
    if (v_out[i] > reachable) {
      v_out[i] = reachable;
    }
  }
  // Backward pass: must be able to brake to each later limit.
  if (v_out[n - 1] > v_end) {
    v_out[n - 1] = v_end;
  }
  for (int i = n - 2; i >= 0; --i) {
    float ds = fmaxf(s[i + 1] - s[i], 0.0f);
    float reachable = sqrtf(v_out[i + 1] * v_out[i + 1] + 2.0f * p->a_brake_max * ds);
    if (v_out[i] > reachable) {
      v_out[i] = reachable;
    }
  }
}
