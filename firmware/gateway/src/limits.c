#include "gateway/limits.h"

#include <math.h>

#include "gateway/config.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float gw_limit_steer_angle(float angle, gw_faults_t * faults)
{
  if (fabsf(angle) > GW_STEER_ANGLE_MAX_RAD) {
    *faults |= GW_FAULT_STEER_ANGLE_LIMIT;
    return clampf(angle, -GW_STEER_ANGLE_MAX_RAD, GW_STEER_ANGLE_MAX_RAD);
  }
  return angle;
}

float gw_limit_steer_rate(float target, float prev, float dt_s, gw_faults_t * faults)
{
  float max_step = GW_STEER_RATE_MAX_RAD_S * dt_s;
  float delta = target - prev;
  if (fabsf(delta) > max_step) {
    *faults |= GW_FAULT_STEER_RATE_LIMIT;
    return prev + (delta > 0.0f ? max_step : -max_step);
  }
  return target;
}

float gw_limit_throttle_ramp(float target, float prev, float dt_s, float cap)
{
  float max_rise = GW_THROTTLE_RAMP_PER_S * dt_s;
  float out = target;
  if (out > prev + max_rise) {
    out = prev + max_rise;
  }
  return clampf(out, 0.0f, cap);
}

float gw_limit_brake_throttle(float throttle, float brake, gw_faults_t * faults)
{
  if (brake > GW_BRAKE_THROTTLE_CUTOFF && throttle > 0.0f) {
    *faults |= GW_FAULT_BRAKE_THROTTLE_CONFLICT;
    return 0.0f;
  }
  return throttle;
}

float gw_limit_speed(float throttle, float speed, gw_faults_t * faults)
{
  if (speed > GW_SPEED_CAP_M_S + GW_SPEED_CAP_MARGIN_M_S) {
    *faults |= GW_FAULT_SPEED_CAP_MARGIN | GW_FAULT_SPEED_CAP;
    return 0.0f;
  }
  if (speed > GW_SPEED_CAP_M_S) {
    *faults |= GW_FAULT_SPEED_CAP;
    return 0.0f;
  }
  return throttle;
}

float gw_limit_power(float throttle, float motor_power, gw_faults_t * faults)
{
  if (motor_power > GW_POWER_CAP_W) {
    *faults |= GW_FAULT_POWER_CAP;
    return 0.0f;
  }
  return throttle;
}
