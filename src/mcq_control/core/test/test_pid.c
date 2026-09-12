#include "check.h"
#include "mcq/pid.h"

static mcq_pid_params_t params(void)
{
  mcq_pid_params_t p = {0};
  p.k_p = mcq_gain_constant(1.0f);
  p.k_i = mcq_gain_constant(0.5f);
  p.k_f = 1.0f;
  p.pos_limit = 1.0f;
  p.neg_limit = -1.0f;
  p.i_rate = 0.01f;
  return p;
}

static int schedule_interpolates_and_clamps(void)
{
  mcq_gain_schedule_t s = {0};
  s.n = 3;
  s.x[0] = 0.0f;
  s.x[1] = 10.0f;
  s.x[2] = 20.0f;
  s.y[0] = 1.0f;
  s.y[1] = 2.0f;
  s.y[2] = 4.0f;
  CHECK_NEAR(mcq_gain_lookup(&s, -5.0f), 1.0f, 1e-6);
  CHECK_NEAR(mcq_gain_lookup(&s, 5.0f), 1.5f, 1e-6);
  CHECK_NEAR(mcq_gain_lookup(&s, 15.0f), 3.0f, 1e-6);
  CHECK_NEAR(mcq_gain_lookup(&s, 50.0f), 4.0f, 1e-6);
  return 0;
}

static int proportional_and_feedforward(void)
{
  mcq_pid_t pid;
  mcq_pid_params_t p = params();
  p.k_i = mcq_gain_constant(0.0f);
  mcq_pid_init(&pid, &p);
  CHECK_NEAR(mcq_pid_update(&pid, 0.2f, 0.0f, 0.3f, false), 0.5f, 1e-6);
  CHECK_NEAR(mcq_pid_update(&pid, 5.0f, 0.0f, 0.0f, false), 1.0f, 1e-6);  // clamped
  return 0;
}

static int integrator_accumulates(void)
{
  mcq_pid_t pid;
  mcq_pid_params_t p = params();
  p.k_p = mcq_gain_constant(0.0f);
  mcq_pid_init(&pid, &p);
  for (int i = 0; i < 100; ++i) {
    mcq_pid_update(&pid, 1.0f, 0.0f, 0.0f, false);
  }
  CHECK_NEAR(pid.i, 0.5f, 1e-4);  // 100 * 1.0 * 0.5 * 0.01
  return 0;
}

static int integrator_freezes_when_saturated(void)
{
  mcq_pid_t pid;
  mcq_pid_params_t p = params();
  mcq_pid_init(&pid, &p);
  for (int i = 0; i < 1000; ++i) {
    mcq_pid_update(&pid, 5.0f, 0.0f, 0.0f, false);  // P alone saturates
  }
  CHECK(pid.i < 0.01f);
  CHECK_NEAR(pid.control, 1.0f, 1e-6);
  // Error reverses: the integrator may move back toward the inside.
  mcq_pid_update(&pid, -0.1f, 0.0f, 0.0f, false);
  CHECK(pid.i < 0.0f);
  return 0;
}

static int freeze_flag_holds_integrator(void)
{
  mcq_pid_t pid;
  mcq_pid_params_t p = params();
  mcq_pid_init(&pid, &p);
  for (int i = 0; i < 10; ++i) {
    mcq_pid_update(&pid, 0.1f, 0.0f, 0.0f, true);
  }
  CHECK_NEAR(pid.i, 0.0f, 1e-9);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(schedule_interpolates_and_clamps);
  RUN(proportional_and_feedforward);
  RUN(integrator_accumulates);
  RUN(integrator_freezes_when_saturated);
  RUN(freeze_flag_holds_integrator);
  return failures ? 1 : 0;
}
