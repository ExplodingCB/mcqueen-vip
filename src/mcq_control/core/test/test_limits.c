#include "check.h"
#include "mcq/limits.h"

static const mcq_lateral_limits_t LIM = {
  .a_lat_max = 4.0f,
  .jerk_lat_max = 5.0f,
  .steer_max = 0.45f,
  .steer_rate_max = 3.0f,
  .v_min = 1.0f};

static int curvature_bounded_by_lateral_accel(void)
{
  float v = 10.0f;
  float k_max = LIM.a_lat_max / (v * v);
  float k = mcq_limit_curvature(&LIM, 1.0f, k_max, v, 0.01f);
  CHECK_NEAR(k, k_max, 1e-6);
  k = mcq_limit_curvature(&LIM, -1.0f, -k_max, v, 0.01f);
  CHECK_NEAR(k, -k_max, 1e-6);
  // A previous command outside the bound is walked back at the jerk limit, not stepped.
  k = mcq_limit_curvature(&LIM, 1.0f, 1.0f, v, 0.01f);
  CHECK(k < 1.0f && k > k_max);
  return 0;
}

static int curvature_bounded_by_lateral_jerk(void)
{
  float v = 10.0f;
  float dt = 0.01f;
  float k = mcq_limit_curvature(&LIM, 0.03f, 0.0f, v, dt);
  CHECK_NEAR(k, LIM.jerk_lat_max / (v * v) * dt, 1e-7);
  return 0;
}

static int low_speed_uses_floor(void)
{
  float k = mcq_limit_curvature(&LIM, 0.5f, 0.5f, 0.0f, 0.01f);
  CHECK_NEAR(k, 0.5f, 1e-6);  // 4 m/s^2 at 1 m/s allows kappa 4
  return 0;
}

static int rate_limiter(void)
{
  CHECK_NEAR(mcq_rate_limit(1.0f, 0.0f, 3.0f, 0.01f), 0.03f, 1e-6);
  CHECK_NEAR(mcq_rate_limit(-1.0f, 0.0f, 3.0f, 0.01f), -0.03f, 1e-6);
  CHECK_NEAR(mcq_rate_limit(0.01f, 0.0f, 3.0f, 0.01f), 0.01f, 1e-6);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(curvature_bounded_by_lateral_accel);
  RUN(curvature_bounded_by_lateral_jerk);
  RUN(low_speed_uses_floor);
  RUN(rate_limiter);
  return failures ? 1 : 0;
}
