// Bicycle model relations shared by the controllers, the planner and the
// simulator. Ported in spirit from opendbc's vehicle_model.py: the steady-state
// relation between front-wheel angle and path curvature includes an understeer
// term so the same steering command means the same curvature at every speed.
#ifndef MCQ_BICYCLE_H
#define MCQ_BICYCLE_H

typedef struct
{
  float wheelbase;   // m
  float understeer;  // s^2/m: extra rad of steer per m/s^2 of lateral accel
  float steer_max;   // rad at the front wheels
} mcq_bicycle_params_t;

// Curvature (1/m) of the steady-state path for a front-wheel angle at speed.
float mcq_curvature_from_steer(const mcq_bicycle_params_t * p, float steer, float speed);

// Front-wheel angle for a desired curvature at speed, clamped to steer_max.
float mcq_steer_from_curvature(const mcq_bicycle_params_t * p, float curvature, float speed);

// Kinematic bicycle state: rear-axle position, heading, speed.
typedef struct
{
  float x;
  float y;
  float yaw;
  float v;
} mcq_kinematic_state_t;

// Advances the rear-axle kinematic bicycle by dt with a front-wheel angle and
// a longitudinal acceleration. Speed is not allowed below zero.
void mcq_kinematic_step(
  const mcq_bicycle_params_t * p, mcq_kinematic_state_t * s, float steer, float accel, float dt);

float mcq_wrap_angle(float a);

#endif  // MCQ_BICYCLE_H
