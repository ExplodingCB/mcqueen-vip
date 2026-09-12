#include "mcq/bicycle.h"

#include <math.h>

#define MCQ_PI 3.14159265358979323846f

float mcq_wrap_angle(float a)
{
  while (a > MCQ_PI) {
    a -= 2.0f * MCQ_PI;
  }
  while (a < -MCQ_PI) {
    a += 2.0f * MCQ_PI;
  }
  return a;
}

float mcq_curvature_from_steer(const mcq_bicycle_params_t * p, float steer, float speed)
{
  // delta = L * kappa + K_us * v^2 * kappa  =>  kappa = delta / (L + K_us v^2)
  return tanf(steer) / (p->wheelbase + p->understeer * speed * speed);
}

float mcq_steer_from_curvature(const mcq_bicycle_params_t * p, float curvature, float speed)
{
  float steer = atanf(curvature * (p->wheelbase + p->understeer * speed * speed));
  if (steer > p->steer_max) {
    steer = p->steer_max;
  } else if (steer < -p->steer_max) {
    steer = -p->steer_max;
  }
  return steer;
}

void mcq_kinematic_step(
  const mcq_bicycle_params_t * p, mcq_kinematic_state_t * s, float steer, float accel, float dt)
{
  float yaw_rate = s->v * tanf(steer) / p->wheelbase;
  s->x += s->v * cosf(s->yaw) * dt;
  s->y += s->v * sinf(s->yaw) * dt;
  s->yaw = mcq_wrap_angle(s->yaw + yaw_rate * dt);
  s->v += accel * dt;
  if (s->v < 0.0f) {
    s->v = 0.0f;
  }
}
