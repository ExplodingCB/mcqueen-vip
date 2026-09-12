#include "check.h"
#include "gateway/config.h"
#include "gateway/limits.h"

static int steer_angle_clamps_and_flags(void)
{
  gw_faults_t f = 0;
  CHECK_NEAR(gw_limit_steer_angle(0.1f, &f), 0.1f, 1e-6);
  CHECK(f == 0);
  CHECK_NEAR(gw_limit_steer_angle(1.0f, &f), GW_STEER_ANGLE_MAX_RAD, 1e-6);
  CHECK(f & GW_FAULT_STEER_ANGLE_LIMIT);
  f = 0;
  CHECK_NEAR(gw_limit_steer_angle(-1.0f, &f), -GW_STEER_ANGLE_MAX_RAD, 1e-6);
  CHECK(f & GW_FAULT_STEER_ANGLE_LIMIT);
  return 0;
}

static int steer_rate_limits_step(void)
{
  gw_faults_t f = 0;
  float dt = 0.01f;
  float max_step = GW_STEER_RATE_MAX_RAD_S * dt;
  CHECK_NEAR(gw_limit_steer_rate(0.4f, 0.0f, dt, &f), max_step, 1e-6);
  CHECK(f & GW_FAULT_STEER_RATE_LIMIT);
  f = 0;
  CHECK_NEAR(gw_limit_steer_rate(-0.4f, 0.0f, dt, &f), -max_step, 1e-6);
  f = 0;
  CHECK_NEAR(gw_limit_steer_rate(0.01f, 0.0f, dt, &f), 0.01f, 1e-6);
  CHECK(f == 0);
  return 0;
}

static int throttle_ramps_up_and_drops_instantly(void)
{
  float dt = 0.01f;
  CHECK_NEAR(gw_limit_throttle_ramp(1.0f, 0.0f, dt, 1.0f), GW_THROTTLE_RAMP_PER_S * dt, 1e-6);
  CHECK_NEAR(gw_limit_throttle_ramp(0.0f, 1.0f, dt, 1.0f), 0.0f, 1e-6);
  CHECK_NEAR(gw_limit_throttle_ramp(1.0f, 0.99f, dt, 0.5f), 0.5f, 1e-6);  // cap
  CHECK_NEAR(gw_limit_throttle_ramp(-1.0f, 0.0f, dt, 1.0f), 0.0f, 1e-6);
  return 0;
}

static int brake_overrides_throttle(void)
{
  gw_faults_t f = 0;
  CHECK_NEAR(gw_limit_brake_throttle(0.8f, 0.05f, &f), 0.8f, 1e-6);
  CHECK(f == 0);
  CHECK_NEAR(gw_limit_brake_throttle(0.8f, 1.0f, &f), 0.0f, 1e-6);
  CHECK(f & GW_FAULT_BRAKE_THROTTLE_CONFLICT);
  return 0;
}

static int speed_cap_cuts_then_stops(void)
{
  gw_faults_t f = 0;
  CHECK_NEAR(gw_limit_speed(0.7f, GW_SPEED_CAP_M_S - 0.1f, &f), 0.7f, 1e-6);
  CHECK(f == 0);
  CHECK_NEAR(gw_limit_speed(0.7f, GW_SPEED_CAP_M_S + 0.1f, &f), 0.0f, 1e-6);
  CHECK(f == GW_FAULT_SPEED_CAP);
  f = 0;
  CHECK_NEAR(
    gw_limit_speed(0.7f, GW_SPEED_CAP_M_S + GW_SPEED_CAP_MARGIN_M_S + 0.1f, &f), 0.0f, 1e-6);
  CHECK(f & GW_FAULT_SPEED_CAP_MARGIN);
  return 0;
}

static int power_cap_cuts_throttle(void)
{
  gw_faults_t f = 0;
  CHECK_NEAR(gw_limit_power(0.9f, GW_POWER_CAP_W - 1.0f, &f), 0.9f, 1e-6);
  CHECK(f == 0);
  CHECK_NEAR(gw_limit_power(0.9f, GW_POWER_CAP_W + 1.0f, &f), 0.0f, 1e-6);
  CHECK(f & GW_FAULT_POWER_CAP);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(steer_angle_clamps_and_flags);
  RUN(steer_rate_limits_step);
  RUN(throttle_ramps_up_and_drops_instantly);
  RUN(brake_overrides_throttle);
  RUN(speed_cap_cuts_then_stops);
  RUN(power_cap_cuts_throttle);
  return failures ? 1 : 0;
}
