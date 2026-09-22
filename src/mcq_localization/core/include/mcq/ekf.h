// Error-state EKF for the kart's pose (docs/02-architecture.md section 9).
//
// State, eight elements, in the map frame (east-north-up, REP 103):
//
//   0 px, 1 py       position, m
//   2 vx, 3 vy       velocity, m/s
//   4 yaw            heading, rad
//   5 gyro z bias    rad/s
//   6 ax bias        body forward accelerometer bias, m/s^2
//   7 ay bias        body left accelerometer bias, m/s^2
//
// The IMU drives the prediction at its own rate; GNSS position, GNSS velocity
// and course, and wheel speed correct it. The biases are in the state because
// without them a filter cannot dead-reckon through a GNSS outage, which Phase 1
// has to survive (10 s at first, docs/02 section 9).
//
// Doubles, not the floats of mcq_control/core: this runs on the Jetson rather
// than the microcontroller, and a covariance in single precision loses symmetry
// and positive-definiteness far too easily.
//
// Delayed measurements. A fix from a ZED-F9P over USB reaches the Jetson some
// tens of milliseconds after it was valid, and at 6 m/s 80 ms is half a metre,
// five times the Phase 1 pose budget. So every sample carries the time it was
// measured, and this filter keeps a short history of states, covariances and
// IMU inputs: a fix rewinds to the nearest stored step, updates there, and the
// stored IMU inputs replay forward to now.
//
// Ignoring the latency does not merely bias the estimate. A constant 0.48 m
// innovation against a 2 cm sigma is 24 sigma, so the gate below throws out
// every fix as an outlier and the filter dead-reckons while still reporting a
// healthy covariance. `test_ignoring_latency_makes_the_gate_eat_every_fix` pins
// that: with a rolling radius 1 percent off, the filter that ignores latency
// drifts 0.6 m in ten seconds and keeps going, while the one that replays holds
// 1 cm.
#ifndef MCQ_EKF_H
#define MCQ_EKF_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCQ_EKF_N 8             // state elements
#define MCQ_EKF_HISTORY 64      // predict steps kept for replay
#define MCQ_EKF_MAX_MEAS_DIM 3  // largest update applied at once

typedef struct
{
  // Process noise. The first two drive velocity and heading through the IMU;
  // the walks let the biases move (docs/03-hardware.md section 5).
  double accel_sigma;      // m/s^2, white noise per axis
  double gyro_sigma;       // rad/s, white noise per axis
  double accel_bias_walk;  // m/s^2 per root second
  double gyro_bias_walk;   // rad/s per root second

  // Course over ground is the direction of travel, which is the heading only
  // when the kart is not sliding. Above course_min_speed it is used as a yaw
  // measurement, with course_slip_sigma added to cover the slip angle.
  double course_min_speed;   // m/s
  double course_slip_sigma;  // rad

  // A wheel-speed update is only as good as the rolling radius, so it carries
  // its own floor on sigma no matter what the sensor claims.
  double wheel_sigma_min;  // m/s

  // Predicting for longer than this without an IMU sample is treated as a gap
  // rather than one enormous step.
  double max_predict_dt;  // s
} mcq_ekf_params_t;

// Sensible starting point; every field is a tunable and belongs in YAML.
mcq_ekf_params_t mcq_ekf_default_params(void);

typedef struct
{
  double t;
  double x[MCQ_EKF_N];
  double p[MCQ_EKF_N * MCQ_EKF_N];
  double ax, ay, gz;  // the IMU sample that advanced the state to t
  double dt;
} mcq_ekf_step_t;

typedef struct
{
  mcq_ekf_params_t params;
  double x[MCQ_EKF_N];
  double p[MCQ_EKF_N * MCQ_EKF_N];
  double t;          // time of the current state
  bool initialized;  // a position has been set or a first fix applied

  // Ring buffer of predict steps, oldest first, for replaying a late fix.
  mcq_ekf_step_t history[MCQ_EKF_HISTORY];
  int history_head;   // index of the oldest entry
  int history_count;  // entries in use

  // Last IMU sample applied, so the outputs can report yaw rate and
  // accelerations with the estimated biases removed.
  double last_ax, last_ay, last_gz;

  // Counters, for telemetry and for tests that care whether a path was taken.
  unsigned long predictions;
  unsigned long updates;
  unsigned long rewinds;         // fixes that replayed through the history
  unsigned long late_drops;      // fixes older than the whole history
  unsigned long rejections;      // updates refused by the gate below
  double last_innovation_ratio;  // last Mahalanobis ratio, for debugging
} mcq_ekf_t;

typedef struct
{
  double x, y, yaw;
  double vx, vy;         // map frame
  double v;              // body longitudinal speed, m/s
  double v_lat;          // body lateral speed, m/s, positive left
  double yaw_rate;       // rad/s, last gyro minus the estimated bias
  double a_long, a_lat;  // m/s^2, last accelerometer minus the estimated bias
  double pos_sigma;      // m, larger of the two horizontal standard deviations
  double yaw_sigma;      // rad
  double gyro_bias, ax_bias, ay_bias;
} mcq_ekf_output_t;

void mcq_ekf_init(mcq_ekf_t * ekf, const mcq_ekf_params_t * params);

// Sets the pose and its uncertainty, and marks the filter initialized. Called
// with the first fix, or with a survey point at a known start line.
void mcq_ekf_set_pose(
  mcq_ekf_t * ekf, double t, double x, double y, double yaw, double pos_sigma, double yaw_sigma);

// Advances the state to the sample time with one IMU sample, body frame,
// gravity already removed from az (which this planar filter does not use).
// Returns false when the sample is older than the current state.
bool mcq_ekf_predict(mcq_ekf_t * ekf, double t, double ax, double ay, double gz);

// GNSS position at its measurement time. Returns false when it was rejected.
bool mcq_ekf_gnss_position(mcq_ekf_t * ekf, double t, double x, double y, double sigma);

// GNSS velocity in the map frame at its measurement time. Above
// course_min_speed this also applies the direction of travel as a yaw
// measurement, which is how yaw converges without a dual-antenna receiver.
bool mcq_ekf_gnss_velocity(mcq_ekf_t * ekf, double t, double vx, double vy, double sigma);

// Wheel speed, unsigned, from the unpowered front wheels.
bool mcq_ekf_wheel_speed(mcq_ekf_t * ekf, double t, double speed, double sigma);

// Is the measured yaw rate consistent with the steering angle through the
// bicycle model? Not an update: a disagreement means a sensor or the model is
// wrong, and the caller raises it rather than filtering it away
// (docs/02-architecture.md section 9).
bool mcq_ekf_yaw_rate_consistent(
  const mcq_ekf_t * ekf, double steering_angle, double wheelbase, double tolerance);

void mcq_ekf_output(const mcq_ekf_t * ekf, mcq_ekf_output_t * out);

// Row-major 6x6 covariance for mcq_msgs/EgoState: x y z roll pitch yaw. The
// unestimated rows carry a large number rather than zero, because zero reads as
// certainty to anything downstream.
void mcq_ekf_covariance6(const mcq_ekf_t * ekf, double * out36);

#ifdef __cplusplus
}
#endif

#endif  // MCQ_EKF_H
