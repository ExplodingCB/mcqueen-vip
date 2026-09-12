// Software-side command shaping (docs/02-architecture.md 8.3). Looser than the
// gateway's firmware limits so the gateway never has to act in normal driving.
// Curvature limits follow the shape of opendbc/car/lateral.py: lateral
// acceleration and lateral jerk bounds expressed in curvature at the current
// speed, plus a plain rate limiter on the steering angle.
#ifndef MCQ_LIMITS_H
#define MCQ_LIMITS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float a_lat_max;       // m/s^2
  float jerk_lat_max;    // m/s^3
  float steer_max;       // rad
  float steer_rate_max;  // rad/s
  float v_min;           // m/s below which curvature limits use v_min (avoids division by ~0)
} mcq_lateral_limits_t;

// Limits desired curvature by lateral acceleration and lateral jerk given the
// previous commanded curvature and the control period.
float mcq_limit_curvature(
  const mcq_lateral_limits_t * lim, float curvature, float prev_curvature, float speed, float dt);

// Plain slew-rate limiter with symmetric bounds.
float mcq_rate_limit(float target, float prev, float rate_max, float dt);

float mcq_clampf(float v, float lo, float hi);

#ifdef __cplusplus
}
#endif

#endif  // MCQ_LIMITS_H
