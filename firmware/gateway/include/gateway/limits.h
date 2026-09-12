// Per-tick limiters. Pure functions so they can be unit tested on the host and
// reused by the Jetson side (mcq_control applies the same shapes with looser
// values so this layer never has to act in normal driving).
#ifndef GATEWAY_LIMITS_H
#define GATEWAY_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#include "gateway/faults.h"

#ifdef __cplusplus
extern "C" {
#endif

// Clamps |angle| to the mechanical limit. Sets GW_FAULT_STEER_ANGLE_LIMIT when
// it had to.
float gw_limit_steer_angle(float angle, gw_faults_t * faults);

// Limits the change from prev to target to the rate limit over dt_s. Sets
// GW_FAULT_STEER_RATE_LIMIT when it had to.
float gw_limit_steer_rate(float target, float prev, float dt_s, gw_faults_t * faults);

// Throttle may rise by at most GW_THROTTLE_RAMP_PER_S * dt_s per tick; it may
// fall instantly. Result is clamped to 0..cap.
float gw_limit_throttle_ramp(float target, float prev, float dt_s, float cap);

// Throttle is forced to zero when the brake exceeds the cutoff. Sets
// GW_FAULT_BRAKE_THROTTLE_CONFLICT when it acted.
float gw_limit_brake_throttle(float throttle, float brake, gw_faults_t * faults);

// Speed cap: above the cap the throttle is cut (GW_FAULT_SPEED_CAP); above cap
// plus margin GW_FAULT_SPEED_CAP_MARGIN is set, which the state machine treats
// as an urgent stop trigger. Returns the throttle to apply.
float gw_limit_speed(float throttle, float speed, gw_faults_t * faults);

// Drivetrain power above the cap cuts throttle (GW_FAULT_POWER_CAP).
float gw_limit_power(float throttle, float motor_power, gw_faults_t * faults);

#ifdef __cplusplus
}
#endif

#endif  // GATEWAY_LIMITS_H
