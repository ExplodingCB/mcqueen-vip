// The estimator, driven by hand-built sensor streams.
//
// Noise-free on purpose: these tests pin the structure (does the prediction
// integrate correctly, does a late fix land in the right place, is the gyro bias
// observable, does an outage widen the covariance). The statistical question,
// whether the thing reaches 0.10 m RMS against the simulator's own sensor noise,
// is answered by src/mcq_sim/test/test_estimator.py, which drives this same
// library through ctypes.
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "check.h"
#include "mcq/ekf.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IMU_DT 0.005  // 200 Hz
#define GNSS_DT 0.05  // 20 Hz
#define LATENCY 0.08  // s, what the receiver and USB cost us

static mcq_ekf_params_t params(void) { return mcq_ekf_default_params(); }

static int test_uninitialized_does_nothing(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  CHECK(!mcq_ekf_predict(&ekf, 0.1, 0.0, 0.0, 0.0));
  CHECK(!mcq_ekf_wheel_speed(&ekf, 0.1, 5.0, 0.05));
  // The first position fix is what initializes it.
  CHECK(mcq_ekf_gnss_position(&ekf, 0.1, 3.0, 4.0, 0.02));
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.x, 3.0, 1e-9);
  CHECK_NEAR(out.y, 4.0, 1e-9);
  // Heading is unknown until it moves, and the covariance has to say so.
  CHECK(out.yaw_sigma > 1.0);
  return 0;
}

static int test_out_of_order_predict_is_refused(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 1.0, 0.0, 0.0, 0.0, 0.1, 0.1);
  CHECK(mcq_ekf_predict(&ekf, 1.005, 0.0, 0.0, 0.0));
  CHECK(!mcq_ekf_predict(&ekf, 1.004, 0.0, 0.0, 0.0));
  CHECK(!mcq_ekf_predict(&ekf, 1.005, 0.0, 0.0, 0.0));
  return 0;
}

static int test_prediction_integrates_acceleration(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.02, 0.01);
  // 1 m/s^2 forward for two seconds: 2 m/s and 2 m.
  for (int k = 1; k <= 400; ++k) {
    CHECK(mcq_ekf_predict(&ekf, (double)k * IMU_DT, 1.0, 0.0, 0.0));
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.v, 2.0, 1e-9);
  CHECK_NEAR(out.x, 2.0, 1e-6);
  CHECK_NEAR(out.y, 0.0, 1e-9);
  CHECK_NEAR(out.a_long, 1.0, 1e-9);
  // Dead reckoning with no fixes: the filter has to become less sure.
  CHECK(out.pos_sigma > 0.02);
  return 0;
}

static int test_prediction_integrates_yaw_rate(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.02, 0.01);
  const double rate = 0.2;
  for (int k = 1; k <= 400; ++k) {  // 2 s
    CHECK(mcq_ekf_predict(&ekf, (double)k * IMU_DT, 0.0, 0.0, rate));
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.yaw, 0.4, 1e-9);
  CHECK_NEAR(out.yaw_rate, rate, 1e-9);
  return 0;
}

static int test_yaw_wraps_rather_than_winding_up(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 3.0, 0.02, 0.01);
  for (int k = 1; k <= 200; ++k) {  // 1 s at 1 rad/s, past pi
    CHECK(mcq_ekf_predict(&ekf, (double)k * IMU_DT, 0.0, 0.0, 1.0));
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK(out.yaw > -M_PI && out.yaw <= M_PI);
  CHECK_NEAR(out.yaw, 4.0 - 2.0 * M_PI, 1e-9);
  return 0;
}

static int test_a_good_fix_pulls_a_bad_position(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 5.0, 0.1);  // 5 m of doubt
  CHECK(mcq_ekf_gnss_position(&ekf, 0.0, 10.0, 0.0, 0.02));
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  // A 2 cm fix against 5 m of prior: the fix wins almost entirely.
  CHECK_NEAR(out.x, 10.0, 0.01);
  CHECK(out.pos_sigma < 0.03);
  return 0;
}

static int test_a_wild_fix_is_rejected(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.02, 0.01);
  // 50 m away with a 2 cm claimed sigma: that is not a fix, it is a fault.
  CHECK(!mcq_ekf_gnss_position(&ekf, 0.0, 50.0, 0.0, 0.02));
  CHECK(ekf.rejections == 1);
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.x, 0.0, 1e-9);
  return 0;
}

static int test_course_converges_yaw(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  // Believes it faces east; actually driving north at 5 m/s.
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.1, M_PI);
  const double v = 5.0;
  double t = 0.0;
  for (int fix = 0; fix < 40; ++fix) {  // 2 s of fixes
    for (int k = 0; k < 10; ++k) {      // IMU between fixes
      t += IMU_DT;
      mcq_ekf_predict(&ekf, t, 0.0, 0.0, 0.0);
    }
    mcq_ekf_gnss_velocity(&ekf, t, 0.0, v, 0.03);
    mcq_ekf_gnss_position(&ekf, t, 0.0, v * t, 0.02);
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.yaw, M_PI / 2.0, 0.05);
  CHECK_NEAR(out.v, v, 0.2);
  return 0;
}

static int test_course_is_ignored_at_a_standstill(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.1, 0.05);
  double t = 0.0;
  for (int fix = 0; fix < 40; ++fix) {
    for (int k = 0; k < 10; ++k) {
      t += IMU_DT;
      mcq_ekf_predict(&ekf, t, 0.0, 0.0, 0.0);
    }
    // Parked, so the reported direction of travel is pure noise. Feeding it in
    // as a heading would spin the estimate.
    const double angle = (fix % 2) ? 2.0 : -2.0;
    mcq_ekf_gnss_velocity(&ekf, t, 0.05 * cos(angle), 0.05 * sin(angle), 0.03);
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.yaw, 0.0, 0.05);
  return 0;
}

static int test_gyro_bias_is_estimated(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05);
  // Driving dead straight east, but the gyro insists the kart is turning.
  const double bias = 0.02;
  const double v = 6.0;
  double t = 0.0;
  for (int fix = 0; fix < 400; ++fix) {  // 20 s
    for (int k = 0; k < 10; ++k) {
      t += IMU_DT;
      mcq_ekf_predict(&ekf, t, 0.0, 0.0, bias);
    }
    mcq_ekf_gnss_position(&ekf, t, v * t, 0.0, 0.02);
    mcq_ekf_gnss_velocity(&ekf, t, v, 0.0, 0.03);
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.gyro_bias, bias, 0.005);
  CHECK_NEAR(out.yaw, 0.0, 0.02);
  CHECK_NEAR(out.yaw_rate, 0.0, 0.005);
  return 0;
}

// The reason the history buffer exists, and the failure it prevents.
//
// Two filters see the same fixes; one is told when each fix was measured, the
// other assumes it is current. Both also get a wheel speed that is 1 percent
// fast, which is an ordinary tyre-pressure error and the sort of thing only GNSS
// can catch.
//
// The naive filter does not end up half a metre behind, which is what one might
// expect. It ends up rejecting every single fix: a constant 0.48 m innovation
// against a 2 cm sigma is 24 sigma, the gate throws it out as an outlier, and
// the filter runs on dead reckoning while reporting a healthy covariance. That
// is the dangerous failure, because it looks perfect in a noise-free test and
// drifts away on the kart with nothing in the log saying GNSS was discarded.
static void drive_with_late_fixes(
  mcq_ekf_t * ekf, bool honest, double wheel_scale, double * expected_x)
{
  const double v = 6.0;
  mcq_ekf_set_pose(ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.02);
  double t = 0.0;
  int fixes = 0;
  while (t < 10.0) {
    t += IMU_DT;
    mcq_ekf_predict(ekf, t, 0.0, 0.0, 0.0);
    mcq_ekf_wheel_speed(ekf, t, v * wheel_scale, 0.05);
    const double due = (double)(fixes + 1) * GNSS_DT + LATENCY;
    if (t + 1e-9 >= due) {
      const double t_meas = (double)(fixes + 1) * GNSS_DT;
      const double measured = v * t_meas;
      mcq_ekf_gnss_position(ekf, honest ? t_meas : t, measured, 0.0, 0.02);
      fixes++;
    }
  }
  *expected_x = v * t;
}

static int test_a_late_fix_belongs_in_the_past(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  double expected = 0.0;
  drive_with_late_fixes(&ekf, true, 1.01, &expected);
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  // Every fix was believed, and the wheel scale error is corrected away.
  CHECK(ekf.rejections == 0);
  CHECK(ekf.rewinds > 150);
  CHECK(fabs(out.x - expected) < 0.05);
  return 0;
}

static int test_ignoring_latency_makes_the_gate_eat_every_fix(void)
{
  mcq_ekf_t naive;
  mcq_ekf_t honest;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&naive, &p);
  mcq_ekf_init(&honest, &p);
  double expected_naive = 0.0;
  double expected_honest = 0.0;
  drive_with_late_fixes(&naive, false, 1.01, &expected_naive);
  drive_with_late_fixes(&honest, true, 1.01, &expected_honest);

  mcq_ekf_output_t bad;
  mcq_ekf_output_t good;
  mcq_ekf_output(&naive, &bad);
  mcq_ekf_output(&honest, &good);

  // Not one fix survived the gate, and it never rewound because it was never
  // told anything happened in the past.
  CHECK(naive.rejections > 150);
  CHECK(naive.rewinds == 0);
  // So the wheel error integrates unchecked: 1 percent of 6 m/s for 10 s.
  CHECK(fabs(bad.x - expected_naive) > 0.4);
  CHECK(fabs(good.x - expected_honest) < 0.05);
  return 0;
}

static int test_a_fix_older_than_the_history_is_counted(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.02);
  double t = 0.0;
  for (int k = 0; k < 200; ++k) {  // 1 s, more than the 64-step buffer holds
    t += IMU_DT;
    mcq_ekf_predict(&ekf, t, 0.0, 0.0, 0.0);
  }
  // A fix from half a second ago, well past the end of the buffer.
  mcq_ekf_gnss_position(&ekf, t - 0.5, 0.0, 0.0, 0.02);
  CHECK(ekf.late_drops == 1);
  return 0;
}

static int test_an_outage_widens_the_covariance(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.02);
  double t = 0.0;
  for (int fix = 0; fix < 40; ++fix) {  // 2 s with fixes
    for (int k = 0; k < 10; ++k) {
      t += IMU_DT;
      mcq_ekf_predict(&ekf, t, 0.0, 0.0, 0.0);
    }
    mcq_ekf_gnss_position(&ekf, t, 0.0, 0.0, 0.02);
    mcq_ekf_gnss_velocity(&ekf, t, 0.0, 0.0, 0.03);
  }
  mcq_ekf_output_t settled;
  mcq_ekf_output(&ekf, &settled);
  CHECK(settled.pos_sigma < 0.05);

  for (int k = 0; k < 2000; ++k) {  // 10 s of nothing, the Phase 1 budget
    t += IMU_DT;
    mcq_ekf_predict(&ekf, t, 0.0, 0.0, 0.0);
  }
  mcq_ekf_output_t adrift;
  mcq_ekf_output(&ekf, &adrift);
  // The filter must know it is lost. A geofence covariance gate is what stops
  // the kart (docs/02-architecture.md section 9), and it can only fire if the
  // covariance actually grows.
  CHECK(adrift.pos_sigma > 10.0 * settled.pos_sigma);
  CHECK(adrift.pos_sigma > 0.5);
  return 0;
}

static int test_wheel_speed_corrects_velocity(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.01);
  double t = 0.0;
  for (int k = 0; k < 200; ++k) {  // 1 s of "we are doing 5 m/s"
    t += IMU_DT;
    mcq_ekf_predict(&ekf, t, 0.0, 0.0, 0.0);
    mcq_ekf_wheel_speed(&ekf, t, 5.0, 0.05);
  }
  mcq_ekf_output_t out;
  mcq_ekf_output(&ekf, &out);
  CHECK_NEAR(out.v, 5.0, 0.1);
  CHECK_NEAR(out.v_lat, 0.0, 0.05);
  return 0;
}

static int test_covariance_stays_symmetric_and_positive(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.02);
  double t = 0.0;
  const double v = 6.0;
  const double rate = 0.3;
  double yaw = 0.0;
  for (int fix = 0; fix < 600; ++fix) {  // 30 s of driving in circles
    for (int k = 0; k < 10; ++k) {
      t += IMU_DT;
      yaw += rate * IMU_DT;
      mcq_ekf_predict(&ekf, t, 0.0, v * rate, rate);
    }
    mcq_ekf_gnss_velocity(&ekf, t - LATENCY, v * cos(yaw), v * sin(yaw), 0.03);
    mcq_ekf_wheel_speed(&ekf, t, v, 0.05);
  }
  for (int i = 0; i < MCQ_EKF_N; ++i) {
    CHECK(ekf.p[i * MCQ_EKF_N + i] > 0.0);
    CHECK(ekf.p[i * MCQ_EKF_N + i] < 1e6);
    for (int j = 0; j < MCQ_EKF_N; ++j) {
      CHECK_NEAR(ekf.p[i * MCQ_EKF_N + j], ekf.p[j * MCQ_EKF_N + i], 1e-15);
    }
  }
  return 0;
}

static int test_yaw_rate_consistency_check(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 0.0, 0.0, 0.0, 0.05, 0.01);
  // Drive at 5 m/s with the gyro reading what a 0.1 rad steering angle implies.
  const double wheelbase = 1.05;
  const double steer = 0.1;
  const double rate = 5.0 * tan(steer) / wheelbase;
  double t = 0.0;
  for (int k = 0; k < 200; ++k) {
    t += IMU_DT;
    mcq_ekf_predict(&ekf, t, 0.0, 0.0, rate);
    mcq_ekf_wheel_speed(&ekf, t, 5.0, 0.05);
  }
  CHECK(mcq_ekf_yaw_rate_consistent(&ekf, steer, wheelbase, 0.05));
  // Same motion, but the steering angle says it should be going straight.
  CHECK(!mcq_ekf_yaw_rate_consistent(&ekf, 0.0, wheelbase, 0.05));
  return 0;
}

static int test_covariance6_layout(void)
{
  mcq_ekf_t ekf;
  mcq_ekf_params_t p = params();
  mcq_ekf_init(&ekf, &p);
  mcq_ekf_set_pose(&ekf, 0.0, 1.0, 2.0, 0.0, 0.3, 0.2);
  double cov[36];
  mcq_ekf_covariance6(&ekf, cov);
  CHECK_NEAR(cov[0], 0.09, 1e-12);          // x
  CHECK_NEAR(cov[1 * 6 + 1], 0.09, 1e-12);  // y
  CHECK_NEAR(cov[5 * 6 + 5], 0.04, 1e-12);  // yaw
  // Unestimated rows say "no idea" rather than "exactly zero".
  CHECK(cov[2 * 6 + 2] > 1e3);
  CHECK(cov[3 * 6 + 3] > 1e3);
  CHECK(cov[4 * 6 + 4] > 1e3);
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(test_uninitialized_does_nothing);
  RUN(test_out_of_order_predict_is_refused);
  RUN(test_prediction_integrates_acceleration);
  RUN(test_prediction_integrates_yaw_rate);
  RUN(test_yaw_wraps_rather_than_winding_up);
  RUN(test_a_good_fix_pulls_a_bad_position);
  RUN(test_a_wild_fix_is_rejected);
  RUN(test_course_converges_yaw);
  RUN(test_course_is_ignored_at_a_standstill);
  RUN(test_gyro_bias_is_estimated);
  RUN(test_a_late_fix_belongs_in_the_past);
  RUN(test_ignoring_latency_makes_the_gate_eat_every_fix);
  RUN(test_a_fix_older_than_the_history_is_counted);
  RUN(test_an_outage_widens_the_covariance);
  RUN(test_wheel_speed_corrects_velocity);
  RUN(test_covariance_stays_symmetric_and_positive);
  RUN(test_yaw_rate_consistency_check);
  RUN(test_covariance6_layout);
  printf("%s: %d failure(s)\n", __FILE__, failures);
  return failures ? 1 : 0;
}
