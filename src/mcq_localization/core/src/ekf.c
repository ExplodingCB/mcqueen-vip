#include "mcq/ekf.h"

#include <math.h>
#include <string.h>

#include "mcq/linalg.h"

#define N MCQ_EKF_N
// math.h does not promise MCQ_PI under strict C11, as mcq_control/core found too.
#define MCQ_PI 3.14159265358979323846

#define IDX_PX 0
#define IDX_PY 1
#define IDX_VX 2
#define IDX_VY 3
#define IDX_YAW 4
#define IDX_BGZ 5
#define IDX_BAX 6
#define IDX_BAY 7

// Five sigma per measurement dimension. Loose on purpose: the job is to throw
// out a wild outlier, not to argue with a fix that is merely worse than hoped.
// A tight gate on a filter whose covariance is still settling rejects the very
// measurements that would have settled it.
#define GATE_CHI2_PER_DIM 25.0

static double wrap_angle(double a)
{
  while (a > MCQ_PI) {
    a -= 2.0 * MCQ_PI;
  }
  while (a < -MCQ_PI) {
    a += 2.0 * MCQ_PI;
  }
  return a;
}

mcq_ekf_params_t mcq_ekf_default_params(void)
{
  mcq_ekf_params_t p;
  p.accel_sigma = 0.15;
  p.gyro_sigma = 0.01;
  p.accel_bias_walk = 0.02;
  p.gyro_bias_walk = 0.001;
  p.course_min_speed = 1.0;
  p.course_slip_sigma = 0.05;
  p.wheel_sigma_min = 0.05;
  p.max_predict_dt = 0.05;
  return p;
}

void mcq_ekf_init(mcq_ekf_t * ekf, const mcq_ekf_params_t * params)
{
  memset(ekf, 0, sizeof(*ekf));
  ekf->params = params ? *params : mcq_ekf_default_params();
  // Unknown everything: metres of position, a full circle of heading.
  for (int i = 0; i < N; ++i) {
    ekf->p[i * N + i] = 1e4;
  }
  ekf->p[IDX_YAW * N + IDX_YAW] = MCQ_PI * MCQ_PI;
  ekf->p[IDX_BGZ * N + IDX_BGZ] = 0.01;
  ekf->p[IDX_BAX * N + IDX_BAX] = 1.0;
  ekf->p[IDX_BAY * N + IDX_BAY] = 1.0;
}

void mcq_ekf_set_pose(
  mcq_ekf_t * ekf, double t, double x, double y, double yaw, double pos_sigma, double yaw_sigma)
{
  ekf->x[IDX_PX] = x;
  ekf->x[IDX_PY] = y;
  ekf->x[IDX_YAW] = wrap_angle(yaw);
  ekf->p[IDX_PX * N + IDX_PX] = pos_sigma * pos_sigma;
  ekf->p[IDX_PY * N + IDX_PY] = pos_sigma * pos_sigma;
  ekf->p[IDX_YAW * N + IDX_YAW] = yaw_sigma * yaw_sigma;
  ekf->t = t;
  ekf->initialized = true;
  ekf->history_head = 0;
  ekf->history_count = 0;
}

// ----------------------------------------------------------------- prediction

// Advances x and P by one IMU sample. No history bookkeeping: the caller owns
// that, because replay calls this on stored inputs.
static void predict_step(mcq_ekf_t * ekf, double ax, double ay, double gz, double dt)
{
  const mcq_ekf_params_t * pr = &ekf->params;
  double * x = ekf->x;

  const double yaw = x[IDX_YAW];
  const double c = cos(yaw);
  const double s = sin(yaw);
  const double abx = ax - x[IDX_BAX];
  const double aby = ay - x[IDX_BAY];
  const double amx = c * abx - s * aby;  // map frame acceleration
  const double amy = s * abx + c * aby;
  const double yaw_rate = gz - x[IDX_BGZ];

  // State.
  x[IDX_PX] += x[IDX_VX] * dt + 0.5 * amx * dt * dt;
  x[IDX_PY] += x[IDX_VY] * dt + 0.5 * amy * dt * dt;
  x[IDX_VX] += amx * dt;
  x[IDX_VY] += amy * dt;
  x[IDX_YAW] = wrap_angle(yaw + yaw_rate * dt);
  // Biases are random walks: unchanged in the mean, spread in Q.

  // Jacobian.
  double f[N * N];
  mcq_mat_identity(f, N);
  const double half_dt2 = 0.5 * dt * dt;
  f[IDX_PX * N + IDX_VX] = dt;
  f[IDX_PY * N + IDX_VY] = dt;
  f[IDX_PX * N + IDX_YAW] = -amy * half_dt2;
  f[IDX_PY * N + IDX_YAW] = amx * half_dt2;
  f[IDX_PX * N + IDX_BAX] = -c * half_dt2;
  f[IDX_PX * N + IDX_BAY] = s * half_dt2;
  f[IDX_PY * N + IDX_BAX] = -s * half_dt2;
  f[IDX_PY * N + IDX_BAY] = -c * half_dt2;
  f[IDX_VX * N + IDX_YAW] = -amy * dt;
  f[IDX_VY * N + IDX_YAW] = amx * dt;
  f[IDX_VX * N + IDX_BAX] = -c * dt;
  f[IDX_VX * N + IDX_BAY] = s * dt;
  f[IDX_VY * N + IDX_BAX] = -s * dt;
  f[IDX_VY * N + IDX_BAY] = -c * dt;
  f[IDX_YAW * N + IDX_BGZ] = -dt;

  // P = F P F^T + Q.
  double fp[N * N];
  double p_new[N * N];
  mcq_mat_mul(f, ekf->p, fp, N);
  mcq_mat_mul_transpose(fp, f, p_new, N);

  const double q_vel = (pr->accel_sigma * dt) * (pr->accel_sigma * dt);
  const double q_pos = (pr->accel_sigma * half_dt2) * (pr->accel_sigma * half_dt2);
  const double q_yaw = (pr->gyro_sigma * dt) * (pr->gyro_sigma * dt);
  p_new[IDX_PX * N + IDX_PX] += q_pos;
  p_new[IDX_PY * N + IDX_PY] += q_pos;
  p_new[IDX_VX * N + IDX_VX] += q_vel;
  p_new[IDX_VY * N + IDX_VY] += q_vel;
  p_new[IDX_YAW * N + IDX_YAW] += q_yaw;
  p_new[IDX_BGZ * N + IDX_BGZ] += pr->gyro_bias_walk * pr->gyro_bias_walk * dt;
  p_new[IDX_BAX * N + IDX_BAX] += pr->accel_bias_walk * pr->accel_bias_walk * dt;
  p_new[IDX_BAY * N + IDX_BAY] += pr->accel_bias_walk * pr->accel_bias_walk * dt;

  memcpy(ekf->p, p_new, sizeof(p_new));
  mcq_mat_symmetrize(ekf->p, N);

  ekf->last_ax = ax;
  ekf->last_ay = ay;
  ekf->last_gz = gz;
}

// -------------------------------------------------------------------- history

static int history_index(const mcq_ekf_t * ekf, int i)
{
  return (ekf->history_head + i) % MCQ_EKF_HISTORY;
}

// Records the pre-step state so a late measurement can rewind to it.
static void history_push(mcq_ekf_t * ekf, double t, double ax, double ay, double gz, double dt)
{
  int slot;
  if (ekf->history_count < MCQ_EKF_HISTORY) {
    slot = history_index(ekf, ekf->history_count);
    ekf->history_count++;
  } else {
    slot = ekf->history_head;  // overwrite the oldest
    ekf->history_head = (ekf->history_head + 1) % MCQ_EKF_HISTORY;
  }
  mcq_ekf_step_t * e = &ekf->history[slot];
  e->t = t;
  e->dt = dt;
  e->ax = ax;
  e->ay = ay;
  e->gz = gz;
  memcpy(e->x, ekf->x, sizeof(e->x));
  memcpy(e->p, ekf->p, sizeof(e->p));
}

bool mcq_ekf_predict(mcq_ekf_t * ekf, double t, double ax, double ay, double gz)
{
  if (!ekf->initialized) {
    return false;
  }
  double dt = t - ekf->t;
  if (dt <= 0.0) {
    return false;  // out of order; the caller decides what that means
  }
  if (dt > ekf->params.max_predict_dt) {
    dt = ekf->params.max_predict_dt;  // a gap, not one enormous step
  }
  history_push(ekf, t, ax, ay, gz, dt);
  predict_step(ekf, ax, ay, gz, dt);
  ekf->t = t;
  ekf->predictions++;
  return true;
}

// --------------------------------------------------------------------- update

// Joseph-form update: P = (I - KH) P (I - KH)^T + K R K^T. Slower than the
// short form and worth it, because it keeps P symmetric and positive definite
// over a session length rather than only over a test.
static bool apply_update(
  mcq_ekf_t * ekf, const double * h, const double * y, const double * r, int m)
{
  double ph_t[N * MCQ_EKF_MAX_MEAS_DIM];  // P H^T, n x m
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < m; ++j) {
      double sum = 0.0;
      for (int k = 0; k < N; ++k) {
        sum += ekf->p[i * N + k] * h[j * N + k];
      }
      ph_t[i * m + j] = sum;
    }
  }

  double s[MCQ_EKF_MAX_MEAS_DIM * MCQ_EKF_MAX_MEAS_DIM];
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      double sum = r[i * m + j];
      for (int k = 0; k < N; ++k) {
        sum += h[i * N + k] * ph_t[k * m + j];
      }
      s[i * m + j] = sum;
    }
  }

  double s_inv[MCQ_EKF_MAX_MEAS_DIM * MCQ_EKF_MAX_MEAS_DIM];
  if (!mcq_mat_inverse(s, s_inv, m)) {
    return false;
  }

  // Mahalanobis distance: is this measurement believable at all?
  double d2 = 0.0;
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      d2 += y[i] * s_inv[i * m + j] * y[j];
    }
  }
  ekf->last_innovation_ratio = d2 / (double)m;
  if (d2 > GATE_CHI2_PER_DIM * (double)m) {
    ekf->rejections++;
    return false;
  }

  double k[N * MCQ_EKF_MAX_MEAS_DIM];  // K = P H^T S^-1, n x m
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < m; ++j) {
      double sum = 0.0;
      for (int q = 0; q < m; ++q) {
        sum += ph_t[i * m + q] * s_inv[q * m + j];
      }
      k[i * m + j] = sum;
    }
  }

  for (int i = 0; i < N; ++i) {
    double delta = 0.0;
    for (int j = 0; j < m; ++j) {
      delta += k[i * m + j] * y[j];
    }
    ekf->x[i] += delta;
  }
  ekf->x[IDX_YAW] = wrap_angle(ekf->x[IDX_YAW]);

  double a[N * N];  // I - K H
  mcq_mat_identity(a, N);
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double sum = 0.0;
      for (int q = 0; q < m; ++q) {
        sum += k[i * m + q] * h[q * N + j];
      }
      a[i * N + j] -= sum;
    }
  }

  double ap[N * N];
  double p_new[N * N];
  mcq_mat_mul(a, ekf->p, ap, N);
  mcq_mat_mul_transpose(ap, a, p_new, N);
  for (int i = 0; i < N; ++i) {  // + K R K^T
    for (int j = 0; j < N; ++j) {
      double sum = 0.0;
      for (int q = 0; q < m; ++q) {
        for (int w = 0; w < m; ++w) {
          sum += k[i * m + q] * r[q * m + w] * k[j * m + w];
        }
      }
      p_new[i * N + j] += sum;
    }
  }
  memcpy(ekf->p, p_new, sizeof(p_new));
  mcq_mat_symmetrize(ekf->p, N);
  ekf->updates++;
  return true;
}

// Rewinds to the stored step nearest the measurement time and returns the index
// of the first history entry that has to replay, or -1 to update in place.
static int rewind_to(mcq_ekf_t * ekf, double t)
{
  if (ekf->history_count == 0 || t >= ekf->t) {
    return -1;  // nothing stored, or the measurement is current
  }
  const int newest = ekf->history_count - 1;
  const mcq_ekf_step_t * oldest = &ekf->history[history_index(ekf, 0)];
  if (t < oldest->t - oldest->dt) {
    // Older than everything kept. Applying it at the oldest boundary is stale,
    // but dropping every such fix would leave the filter with no position
    // updates at all if the latency ever exceeded the buffer.
    ekf->late_drops++;
    memcpy(ekf->x, oldest->x, sizeof(ekf->x));
    memcpy(ekf->p, oldest->p, sizeof(ekf->p));
    return 0;
  }
  for (int i = newest; i >= 0; --i) {
    const mcq_ekf_step_t * e = &ekf->history[history_index(ekf, i)];
    const double before = e->t - e->dt;
    if (before <= t) {
      // Snap to whichever end of this step is closer, so the worst error from
      // not splitting the step is half an IMU period.
      if ((t - before) > (e->t - t)) {
        if (i == newest) {
          return -1;  // closer to now than to the start of the last step
        }
        const mcq_ekf_step_t * next = &ekf->history[history_index(ekf, i + 1)];
        memcpy(ekf->x, next->x, sizeof(ekf->x));
        memcpy(ekf->p, next->p, sizeof(ekf->p));
        return i + 1;
      }
      memcpy(ekf->x, e->x, sizeof(ekf->x));
      memcpy(ekf->p, e->p, sizeof(ekf->p));
      return i;
    }
  }
  return -1;
}

// Replays the stored IMU inputs from index onward, refreshing their snapshots
// so a later measurement rewinds onto the corrected trajectory.
static void replay_from(mcq_ekf_t * ekf, int index)
{
  if (index < 0) {
    return;
  }
  ekf->rewinds++;
  for (int i = index; i < ekf->history_count; ++i) {
    mcq_ekf_step_t * e = &ekf->history[history_index(ekf, i)];
    memcpy(e->x, ekf->x, sizeof(e->x));
    memcpy(e->p, ekf->p, sizeof(e->p));
    predict_step(ekf, e->ax, e->ay, e->gz, e->dt);
  }
  const mcq_ekf_step_t * newest = &ekf->history[history_index(ekf, ekf->history_count - 1)];
  ekf->t = newest->t;
}

bool mcq_ekf_gnss_position(mcq_ekf_t * ekf, double t, double x, double y, double sigma)
{
  if (!ekf->initialized) {
    mcq_ekf_set_pose(ekf, t, x, y, 0.0, sigma, MCQ_PI);
    return true;
  }
  const int replay = rewind_to(ekf, t);
  double h[2 * N];
  memset(h, 0, sizeof(h));
  h[0 * N + IDX_PX] = 1.0;
  h[1 * N + IDX_PY] = 1.0;
  const double y_res[2] = {x - ekf->x[IDX_PX], y - ekf->x[IDX_PY]};
  const double var = sigma * sigma;
  const double r[4] = {var, 0.0, 0.0, var};
  const bool ok = apply_update(ekf, h, y_res, r, 2);
  replay_from(ekf, replay);
  return ok;
}

bool mcq_ekf_gnss_velocity(mcq_ekf_t * ekf, double t, double vx, double vy, double sigma)
{
  if (!ekf->initialized) {
    return false;
  }
  const int replay = rewind_to(ekf, t);
  double h[2 * N];
  memset(h, 0, sizeof(h));
  h[0 * N + IDX_VX] = 1.0;
  h[1 * N + IDX_VY] = 1.0;
  const double y_res[2] = {vx - ekf->x[IDX_VX], vy - ekf->x[IDX_VY]};
  const double var = sigma * sigma;
  const double r[4] = {var, 0.0, 0.0, var};
  bool ok = apply_update(ekf, h, y_res, r, 2);

  // Direction of travel as a heading measurement. This is what converges yaw
  // on a single-antenna receiver, and it is only meaningful once the kart is
  // moving: at a standstill the direction is noise.
  const double speed = sqrt(vx * vx + vy * vy);
  if (speed >= ekf->params.course_min_speed) {
    const double course = atan2(vy, vx);
    double h_yaw[N];
    memset(h_yaw, 0, sizeof(h_yaw));
    h_yaw[IDX_YAW] = 1.0;
    const double innovation = wrap_angle(course - ekf->x[IDX_YAW]);
    // The angle is uncertain by the velocity noise across the speed, plus the
    // slip angle, which is a difference between heading and course rather than
    // an error in either.
    const double sigma_course = sigma / speed;
    const double slip = ekf->params.course_slip_sigma;
    const double r_yaw[1] = {sigma_course * sigma_course + slip * slip};
    ok = apply_update(ekf, h_yaw, &innovation, r_yaw, 1) && ok;
  }
  replay_from(ekf, replay);
  return ok;
}

bool mcq_ekf_wheel_speed(mcq_ekf_t * ekf, double t, double speed, double sigma)
{
  if (!ekf->initialized) {
    return false;
  }
  const int replay = rewind_to(ekf, t);
  // The wheels measure speed along the body x axis, not the magnitude of the
  // velocity vector, and that stays well defined at a standstill.
  const double c = cos(ekf->x[IDX_YAW]);
  const double s = sin(ekf->x[IDX_YAW]);
  double h[N];
  memset(h, 0, sizeof(h));
  h[IDX_VX] = c;
  h[IDX_VY] = s;
  const double forward = c * ekf->x[IDX_VX] + s * ekf->x[IDX_VY];
  const double innovation = speed - forward;
  const double used = sigma > ekf->params.wheel_sigma_min ? sigma : ekf->params.wheel_sigma_min;
  const double r[1] = {used * used};
  const bool ok = apply_update(ekf, h, &innovation, r, 1);
  replay_from(ekf, replay);
  return ok;
}

bool mcq_ekf_yaw_rate_consistent(
  const mcq_ekf_t * ekf, double steering_angle, double wheelbase, double tolerance)
{
  mcq_ekf_output_t out;
  mcq_ekf_output(ekf, &out);
  if (wheelbase <= 0.0) {
    return false;
  }
  const double expected = out.v * tan(steering_angle) / wheelbase;
  return fabs(expected - out.yaw_rate) <= tolerance;
}

void mcq_ekf_output(const mcq_ekf_t * ekf, mcq_ekf_output_t * out)
{
  const double * x = ekf->x;
  const double c = cos(x[IDX_YAW]);
  const double s = sin(x[IDX_YAW]);
  out->x = x[IDX_PX];
  out->y = x[IDX_PY];
  out->yaw = x[IDX_YAW];
  out->vx = x[IDX_VX];
  out->vy = x[IDX_VY];
  out->v = c * x[IDX_VX] + s * x[IDX_VY];
  out->v_lat = -s * x[IDX_VX] + c * x[IDX_VY];
  out->yaw_rate = ekf->last_gz - x[IDX_BGZ];
  out->a_long = ekf->last_ax - x[IDX_BAX];
  out->a_lat = ekf->last_ay - x[IDX_BAY];
  const double var_x = ekf->p[IDX_PX * N + IDX_PX];
  const double var_y = ekf->p[IDX_PY * N + IDX_PY];
  out->pos_sigma = sqrt(var_x > var_y ? var_x : var_y);
  out->yaw_sigma = sqrt(ekf->p[IDX_YAW * N + IDX_YAW]);
  out->gyro_bias = x[IDX_BGZ];
  out->ax_bias = x[IDX_BAX];
  out->ay_bias = x[IDX_BAY];
}

void mcq_ekf_covariance6(const mcq_ekf_t * ekf, double * out36)
{
  memset(out36, 0, 36 * sizeof(double));
  const int map[3] = {IDX_PX, IDX_PY, IDX_YAW};  // x, y, yaw
  const int slot[3] = {0, 1, 5};                 // rows in x y z roll pitch yaw
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out36[slot[i] * 6 + slot[j]] = ekf->p[map[i] * N + map[j]];
    }
  }
  // z, roll and pitch are not estimated. Large, not zero: zero is a claim of
  // perfect knowledge, and something downstream will believe it.
  out36[2 * 6 + 2] = 1e6;
  out36[3 * 6 + 3] = 1e6;
  out36[4 * 6 + 4] = 1e6;
}
