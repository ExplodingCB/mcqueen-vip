#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "check.h"
#include "mcq/bicycle.h"

static const mcq_bicycle_params_t BIKE = {
  .wheelbase = 1.05f, .understeer = 0.002f, .steer_max = 0.45f};

static int steer_curvature_roundtrip(void)
{
  for (float v = 0.0f; v < 15.0f; v += 2.5f) {
    for (float k = -0.2f; k <= 0.2f; k += 0.05f) {
      float steer = mcq_steer_from_curvature(&BIKE, k, v);
      CHECK_NEAR(mcq_curvature_from_steer(&BIKE, steer, v), k, 1e-5);
    }
  }
  CHECK_NEAR(mcq_steer_from_curvature(&BIKE, 10.0f, 0.0f), BIKE.steer_max, 1e-6);
  CHECK_NEAR(mcq_steer_from_curvature(&BIKE, -10.0f, 0.0f), -BIKE.steer_max, 1e-6);
  return 0;
}

static int understeer_needs_more_steer_at_speed(void)
{
  float slow = mcq_steer_from_curvature(&BIKE, 0.1f, 1.0f);
  float fast = mcq_steer_from_curvature(&BIKE, 0.1f, 10.0f);
  CHECK(fast > slow);
  return 0;
}

static int kinematic_circle(void)
{
  mcq_bicycle_params_t bike = BIKE;
  bike.understeer = 0.0f;
  mcq_kinematic_state_t s = {0.0f, 0.0f, 0.0f, 5.0f};
  float steer = 0.1f;
  float radius = bike.wheelbase / tanf(steer);
  float dt = 0.001f;
  // Half a circle takes pi * R / v seconds.
  int steps = (int)((float)M_PI * radius / 5.0f / dt);
  for (int i = 0; i < steps; ++i) {
    mcq_kinematic_step(&bike, &s, steer, 0.0f, dt);
  }
  CHECK_NEAR(s.x, 0.0f, 0.05);
  CHECK_NEAR(s.y, 2.0f * radius, 0.05);
  CHECK_NEAR(fabsf(s.yaw), (float)M_PI, 0.01);
  return 0;
}

static int speed_never_negative(void)
{
  mcq_kinematic_state_t s = {0.0f, 0.0f, 0.0f, 1.0f};
  for (int i = 0; i < 100; ++i) {
    mcq_kinematic_step(&BIKE, &s, 0.0f, -5.0f, 0.1f);
  }
  CHECK_NEAR(s.v, 0.0f, 1e-9);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(steer_curvature_roundtrip);
  RUN(understeer_needs_more_steer_at_speed);
  RUN(kinematic_circle);
  RUN(speed_never_negative);
  return failures ? 1 : 0;
}
