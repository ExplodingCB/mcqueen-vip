// PID with speed-scheduled gains, feedforward, output limits and anti-windup.
//
// Follows the controller shape used by openpilot's controls (pid.py): gains are
// piecewise-linear in a scheduling variable (speed), the integrator is frozen
// while the output is saturated in the direction of the error, and the
// integrator moves at its own rate so the loop can run faster than the tune.
// No allocation, no dependencies beyond libm.
#ifndef MCQ_PID_H
#define MCQ_PID_H

#include <stdbool.h>

#define MCQ_PID_MAX_BREAKPOINTS 8

typedef struct
{
  int n;                             // number of breakpoints (1 = constant gain)
  float x[MCQ_PID_MAX_BREAKPOINTS];  // scheduling variable, ascending
  float y[MCQ_PID_MAX_BREAKPOINTS];  // gain at each breakpoint
} mcq_gain_schedule_t;

typedef struct
{
  mcq_gain_schedule_t k_p;
  mcq_gain_schedule_t k_i;
  float k_f;        // feedforward gain
  float pos_limit;  // output upper bound
  float neg_limit;  // output lower bound
  float i_rate;     // seconds per update of the integrator (1 / control rate)
  float i_unwind;   // integrator decay per update when the output is not saturated (0..1)
} mcq_pid_params_t;

typedef struct
{
  mcq_pid_params_t params;
  float p;
  float i;
  float f;
  float control;
} mcq_pid_t;

// A schedule with one breakpoint is a constant gain.
mcq_gain_schedule_t mcq_gain_constant(float gain);
float mcq_gain_lookup(const mcq_gain_schedule_t * schedule, float x);

void mcq_pid_init(mcq_pid_t * pid, const mcq_pid_params_t * params);
void mcq_pid_reset(mcq_pid_t * pid);

// error = setpoint - measurement, feedforward is added through k_f.
// speed schedules the gains. freeze_integrator holds the integrator (used
// while the actuator is not yet under control, matching openpilot).
float mcq_pid_update(
  mcq_pid_t * pid, float error, float speed, float feedforward, bool freeze_integrator);

#endif  // MCQ_PID_H
