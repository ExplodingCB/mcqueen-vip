// Speed profile over a path (docs/02-architecture.md 7.2 step 4): the
// curvature-limited speed, capped, then a forward pass with the acceleration
// limit and a backward pass with the braking limit so the profile is reachable.
#ifndef MCQ_SPEED_PROFILE_H
#define MCQ_SPEED_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float a_lat_max;    // m/s^2
  float a_long_max;   // m/s^2, acceleration
  float a_brake_max;  // m/s^2, positive number
  float v_cap;        // m/s
  float v_min;        // m/s, floor to keep the profile moving through tight spots
} mcq_speed_profile_params_t;

// s[i] is arc length (ascending), kappa[i] curvature, v_ref[i] an optional
// reference speed (raceline profile, may be NULL). v_out[i] receives the
// profile. v0 is the speed at s[0] (the current speed) and bounds the forward
// pass from the start; v_end bounds the backward pass at the last point.
void mcq_speed_profile(
  const mcq_speed_profile_params_t * p, const float * s, const float * kappa, const float * v_ref,
  int n, float v0, float v_end, float * v_out);

#ifdef __cplusplus
}
#endif

#endif  // MCQ_SPEED_PROFILE_H
