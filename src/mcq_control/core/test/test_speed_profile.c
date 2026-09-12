#include <math.h>

#include "check.h"
#include "mcq/speed_profile.h"

static const mcq_speed_profile_params_t P = {
  .a_lat_max = 3.0f, .a_long_max = 2.0f, .a_brake_max = 3.0f, .v_cap = 10.0f, .v_min = 1.0f};

static int respects_curvature_and_cap(void)
{
  enum { N = 3 };
  float s[N] = {0.0f, 100.0f, 200.0f};
  float kappa[N] = {0.0f, 0.1f, 0.0f};
  float v[N];
  mcq_speed_profile(&P, s, kappa, NULL, N, 10.0f, 10.0f, v);
  CHECK_NEAR(v[0], 10.0f, 1e-6);
  CHECK_NEAR(v[1], sqrtf(3.0f / 0.1f), 1e-5);
  CHECK_NEAR(v[2], 10.0f, 1e-6);
  return 0;
}

static int forward_and_backward_passes_are_reachable(void)
{
  enum { N = 60 };
  float s[N], kappa[N], v[N];
  for (int i = 0; i < N; ++i) {
    s[i] = (float)i * 1.0f;
    kappa[i] = (i >= 30 && i < 40) ? 0.2f : 0.0f;  // a tight corner mid-path
  }
  mcq_speed_profile(&P, s, kappa, NULL, N, 2.0f, 0.0f, v);
  CHECK_NEAR(v[0], 2.0f, 1e-6);  // starts from the current speed
  for (int i = 1; i < N; ++i) {
    float ds = s[i] - s[i - 1];
    CHECK(v[i] * v[i] <= v[i - 1] * v[i - 1] + 2.0f * P.a_long_max * ds + 1e-3f);
    CHECK(v[i - 1] * v[i - 1] <= v[i] * v[i] + 2.0f * P.a_brake_max * ds + 1e-3f);
    CHECK(v[i] <= P.v_cap + 1e-6f);
  }
  CHECK_NEAR(v[N - 1], 0.0f, 1e-6);  // v_end
  for (int i = 30; i < 40; ++i) {
    CHECK(v[i] <= sqrtf(P.a_lat_max / 0.2f) + 1e-4f);
  }
  return 0;
}

static int reference_profile_is_an_upper_bound(void)
{
  enum { N = 4 };
  float s[N] = {0.0f, 10.0f, 20.0f, 30.0f};
  float kappa[N] = {0.0f, 0.0f, 0.0f, 0.0f};
  float ref[N] = {5.0f, 5.0f, 5.0f, 5.0f};
  float v[N];
  mcq_speed_profile(&P, s, kappa, ref, N, 5.0f, 5.0f, v);
  for (int i = 0; i < N; ++i) {
    CHECK_NEAR(v[i], 5.0f, 1e-6);
  }
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(respects_curvature_and_cap);
  RUN(forward_and_backward_passes_are_reachable);
  RUN(reference_profile_is_an_upper_bound);
  return failures ? 1 : 0;
}
