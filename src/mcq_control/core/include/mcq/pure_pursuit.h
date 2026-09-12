// Pure pursuit lateral controller for bring-up (docs/02-architecture.md 8.1).
//
// lookahead L = clamp(k_v * v, L_min, L_max); the target is the first path
// point at least L ahead of the closest point; kappa = 2 sin(alpha) / L where
// alpha is the angle between the heading and the line to the target; the
// steering angle comes from the bicycle model.
#ifndef MCQ_PURE_PURSUIT_H
#define MCQ_PURE_PURSUIT_H

#include <stdbool.h>

#include "mcq/bicycle.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float k_v;    // s, lookahead per unit speed
  float l_min;  // m
  float l_max;  // m
} mcq_pure_pursuit_params_t;

typedef struct
{
  const float * x;  // path points, map frame
  const float * y;
  int n;
  bool closed;  // path wraps from the last point to the first
} mcq_path_t;

typedef struct
{
  float steer;          // rad at the front wheels
  float curvature;      // 1/m commanded
  float lookahead;      // m used
  int nearest_index;    // index of the closest path point
  int target_index;     // index of the lookahead point
  float lateral_error;  // m, signed, positive when the path is to the left
} mcq_pure_pursuit_result_t;

// Returns false when the path has fewer than two points.
bool mcq_pure_pursuit(
  const mcq_pure_pursuit_params_t * pp, const mcq_bicycle_params_t * bike, const mcq_path_t * path,
  float x, float y, float yaw, float speed, mcq_pure_pursuit_result_t * out);

#ifdef __cplusplus
}
#endif

#endif  // MCQ_PURE_PURSUIT_H
