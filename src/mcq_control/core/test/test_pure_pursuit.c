#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "check.h"
#include "mcq/bicycle.h"
#include "mcq/pure_pursuit.h"

static const mcq_bicycle_params_t BIKE = {
  .wheelbase = 1.05f, .understeer = 0.0f, .steer_max = 0.45f};
static const mcq_pure_pursuit_params_t PP = {.k_v = 0.5f, .l_min = 1.5f, .l_max = 6.0f};

static int straight_path_offset_left_steers_right(void)
{
  float x[50], y[50];
  for (int i = 0; i < 50; ++i) {
    x[i] = (float)i;
    y[i] = 0.0f;
  }
  mcq_path_t path = {x, y, 50, false};
  mcq_pure_pursuit_result_t r;
  CHECK(mcq_pure_pursuit(&PP, &BIKE, &path, 10.0f, 1.0f, 0.0f, 4.0f, &r));
  CHECK(r.steer < 0.0f);
  CHECK_NEAR(r.lateral_error, -1.0f, 1e-5);  // path is to the right
  CHECK_NEAR(r.lookahead, 2.0f, 1e-6);
  CHECK(mcq_pure_pursuit(&PP, &BIKE, &path, 10.0f, -1.0f, 0.0f, 4.0f, &r));
  CHECK(r.steer > 0.0f);
  CHECK_NEAR(r.lateral_error, 1.0f, 1e-5);
  CHECK(mcq_pure_pursuit(&PP, &BIKE, &path, 10.0f, 0.0f, 0.0f, 4.0f, &r));
  CHECK_NEAR(r.steer, 0.0f, 1e-6);
  return 0;
}

static int lookahead_is_clamped(void)
{
  float x[10], y[10];
  for (int i = 0; i < 10; ++i) {
    x[i] = (float)i * 2.0f;
    y[i] = 0.0f;
  }
  mcq_path_t path = {x, y, 10, false};
  mcq_pure_pursuit_result_t r;
  CHECK(mcq_pure_pursuit(&PP, &BIKE, &path, 0.0f, 0.0f, 0.0f, 0.0f, &r));
  CHECK_NEAR(r.lookahead, PP.l_min, 1e-6);
  CHECK(mcq_pure_pursuit(&PP, &BIKE, &path, 0.0f, 0.0f, 0.0f, 100.0f, &r));
  CHECK_NEAR(r.lookahead, PP.l_max, 1e-6);
  CHECK(r.target_index == 3);  // first point at least 6 m ahead
  return 0;
}

static int rejects_short_path(void)
{
  float x[1] = {0.0f}, y[1] = {0.0f};
  mcq_path_t path = {x, y, 1, false};
  mcq_pure_pursuit_result_t r;
  CHECK(!mcq_pure_pursuit(&PP, &BIKE, &path, 0.0f, 0.0f, 0.0f, 1.0f, &r));
  return 0;
}

// Closed loop: a kinematic bicycle tracks a 15 m circle at 5 m/s starting 1 m
// off the line. Steady-state lateral error must be small and the loop stable.
static int tracks_circle_closed_loop(void)
{
  enum { N = 200 };
  float x[N], y[N];
  float radius = 15.0f;
  for (int i = 0; i < N; ++i) {
    float a = 2.0f * (float)M_PI * (float)i / (float)N;
    x[i] = radius * cosf(a);
    y[i] = radius * sinf(a);
  }
  mcq_path_t path = {x, y, N, true};
  mcq_kinematic_state_t s = {radius + 1.0f, 0.0f, (float)M_PI / 2.0f, 5.0f};
  float dt = 0.01f;
  float worst_late = 0.0f;
  float steer = 0.0f;
  for (int step = 0; step < 4000; ++step) {  // 40 s, about two laps
    mcq_pure_pursuit_result_t r;
    CHECK(mcq_pure_pursuit(&PP, &BIKE, &path, s.x, s.y, s.yaw, s.v, &r));
    steer = r.steer;
    mcq_kinematic_step(&BIKE, &s, steer, 0.0f, dt);
    if (step > 2000) {
      float e = fabsf(sqrtf(s.x * s.x + s.y * s.y) - radius);
      if (e > worst_late) {
        worst_late = e;
      }
    }
  }
  CHECK(worst_late < 0.15f);
  // Pure pursuit on a circle converges to a constant steer near the geometric value.
  CHECK_NEAR(steer, atanf(BIKE.wheelbase / radius), 0.02);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(straight_path_offset_left_steers_right);
  RUN(lookahead_is_clamped);
  RUN(rejects_short_path);
  RUN(tracks_circle_closed_loop);
  return failures ? 1 : 0;
}
