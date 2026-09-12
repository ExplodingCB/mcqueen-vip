// Longitudinal controller (docs/02-architecture.md 8.2), ported from the shape
// of openpilot's longcontrol.py: an off / stopping / pid state machine and a
// PID on speed error with the planned acceleration as feedforward, producing a
// signed acceleration that the actuator map turns into throttle and brake.
#ifndef MCQ_LONGITUDINAL_H
#define MCQ_LONGITUDINAL_H

#include <stdbool.h>

#include "mcq/pid.h"

typedef enum {
  MCQ_LONG_OFF = 0,
  MCQ_LONG_STOPPING = 1,
  MCQ_LONG_PID = 2,
} mcq_long_state_t;

typedef struct
{
  mcq_pid_params_t pid;  // on speed error, output is acceleration
  float stopping_speed;  // m/s below which a stop request enters STOPPING
  float stopping_decel;  // m/s^2 applied in STOPPING (negative)
  float accel_max;       // m/s^2
  float decel_max;       // m/s^2, negative
} mcq_long_params_t;

typedef struct
{
  float throttle_per_accel;  // throttle per m/s^2 of requested acceleration
  float brake_per_decel;     // brake per m/s^2 of requested deceleration
  float deadband;            // m/s^2 around zero where neither pedal is used
  float rolling_decel;       // m/s^2 the kart loses with both pedals released (positive number)
} mcq_actuator_map_t;

typedef struct
{
  mcq_long_params_t params;
  mcq_pid_t pid;
  mcq_long_state_t state;
  float accel;  // last output
} mcq_long_t;

void mcq_long_init(mcq_long_t * lc, const mcq_long_params_t * params);

// enabled: longitudinal control active. stop_requested: target is a full stop.
// v_target and a_target come from the trajectory at the current time.
float mcq_long_update(
  mcq_long_t * lc, bool enabled, bool stop_requested, float v, float v_target, float a_target);

// Maps a signed acceleration to throttle and brake in 0..1 with a deadband.
void mcq_actuator_map(const mcq_actuator_map_t * m, float accel, float * throttle, float * brake);

#endif  // MCQ_LONGITUDINAL_H
