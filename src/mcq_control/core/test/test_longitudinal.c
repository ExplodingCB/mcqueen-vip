#include "check.h"
#include "mcq/longitudinal.h"

static mcq_long_params_t params(void)
{
  mcq_long_params_t p = {0};
  p.pid.k_p = mcq_gain_constant(1.0f);
  p.pid.k_i = mcq_gain_constant(0.2f);
  p.pid.k_f = 1.0f;
  p.pid.pos_limit = 3.0f;
  p.pid.neg_limit = -5.0f;
  p.pid.i_rate = 0.01f;
  p.stopping_speed = 0.5f;
  p.stopping_decel = -1.5f;
  p.accel_max = 3.0f;
  p.decel_max = -5.0f;
  return p;
}

static int off_outputs_zero(void)
{
  mcq_long_t lc;
  mcq_long_params_t p = params();
  mcq_long_init(&lc, &p);
  CHECK_NEAR(mcq_long_update(&lc, false, false, 5.0f, 10.0f, 1.0f), 0.0f, 1e-6);
  CHECK(lc.state == MCQ_LONG_OFF);
  return 0;
}

static int pid_tracks_speed_with_feedforward(void)
{
  mcq_long_t lc;
  mcq_long_params_t p = params();
  mcq_long_init(&lc, &p);
  float a = mcq_long_update(&lc, true, false, 4.0f, 5.0f, 0.5f);
  CHECK(lc.state == MCQ_LONG_PID);
  CHECK_NEAR(a, 1.0f * 1.0f + 0.2f * 0.01f + 0.5f, 1e-5);
  a = mcq_long_update(&lc, true, false, 6.0f, 5.0f, 0.0f);
  CHECK(a < 0.0f);
  return 0;
}

static int stop_request_enters_stopping_at_low_speed(void)
{
  mcq_long_t lc;
  mcq_long_params_t p = params();
  mcq_long_init(&lc, &p);
  float a = mcq_long_update(&lc, true, true, 5.0f, 5.0f, 0.0f);
  CHECK(lc.state == MCQ_LONG_PID);  // still fast: PID toward zero speed
  CHECK(a < -1.0f);
  a = mcq_long_update(&lc, true, true, 0.3f, 5.0f, 0.0f);
  CHECK(lc.state == MCQ_LONG_STOPPING);
  CHECK_NEAR(a, p.stopping_decel, 1e-6);
  a = mcq_long_update(&lc, true, false, 0.3f, 2.0f, 0.0f);
  CHECK(lc.state == MCQ_LONG_PID);
  CHECK(a > 0.0f);
  return 0;
}

static int output_clamped_to_limits(void)
{
  mcq_long_t lc;
  mcq_long_params_t p = params();
  mcq_long_init(&lc, &p);
  CHECK_NEAR(mcq_long_update(&lc, true, false, 0.0f, 50.0f, 0.0f), p.accel_max, 1e-6);
  CHECK_NEAR(mcq_long_update(&lc, true, false, 50.0f, 0.0f, 0.0f), p.decel_max, 1e-6);
  return 0;
}

static int actuator_map(void)
{
  mcq_actuator_map_t m = {
    .throttle_per_accel = 0.25f, .brake_per_decel = 0.2f, .deadband = 0.05f, .rolling_decel = 0.3f};
  float t, b;
  mcq_actuator_map(&m, 1.0f, &t, &b);
  CHECK_NEAR(t, 1.3f * 0.25f, 1e-6);
  CHECK_NEAR(b, 0.0f, 1e-6);
  mcq_actuator_map(&m, -2.0f, &t, &b);
  CHECK_NEAR(t, 0.0f, 1e-6);
  CHECK_NEAR(b, 1.7f * 0.2f, 1e-6);
  mcq_actuator_map(&m, -0.3f, &t, &b);  // coasting
  CHECK_NEAR(t, 0.0f, 1e-6);
  CHECK_NEAR(b, 0.0f, 1e-6);
  mcq_actuator_map(&m, 100.0f, &t, &b);
  CHECK_NEAR(t, 1.0f, 1e-6);
  mcq_actuator_map(&m, -100.0f, &t, &b);
  CHECK_NEAR(b, 1.0f, 1e-6);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(off_outputs_zero);
  RUN(pid_tracks_speed_with_feedforward);
  RUN(stop_request_enters_stopping_at_low_speed);
  RUN(output_clamped_to_limits);
  RUN(actuator_map);
  return failures ? 1 : 0;
}
