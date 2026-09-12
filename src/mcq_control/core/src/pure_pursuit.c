#include "mcq/pure_pursuit.h"

#include <math.h>

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int next_index(const mcq_path_t * path, int i)
{
  int j = i + 1;
  if (j >= path->n) {
    return path->closed ? 0 : path->n - 1;
  }
  return j;
}

bool mcq_pure_pursuit(
  const mcq_pure_pursuit_params_t * pp, const mcq_bicycle_params_t * bike, const mcq_path_t * path,
  float x, float y, float yaw, float speed, mcq_pure_pursuit_result_t * out)
{
  if (path->n < 2) {
    return false;
  }
  int nearest = 0;
  float best = INFINITY;
  for (int i = 0; i < path->n; ++i) {
    float dx = path->x[i] - x;
    float dy = path->y[i] - y;
    float d2 = dx * dx + dy * dy;
    if (d2 < best) {
      best = d2;
      nearest = i;
    }
  }

  float lookahead = clampf(pp->k_v * speed, pp->l_min, pp->l_max);

  // Walk forward along the path until a point is at least lookahead away from
  // the vehicle. On an open path the last point is used when the end comes first.
  int target = nearest;
  int steps = 0;
  while (steps < path->n) {
    float dx = path->x[target] - x;
    float dy = path->y[target] - y;
    if (sqrtf(dx * dx + dy * dy) >= lookahead) {
      break;
    }
    int nxt = next_index(path, target);
    if (nxt == target) {
      break;
    }
    target = nxt;
    ++steps;
  }

  float dx = path->x[target] - x;
  float dy = path->y[target] - y;
  float dist = sqrtf(dx * dx + dy * dy);
  float alpha = mcq_wrap_angle(atan2f(dy, dx) - yaw);
  float curvature = dist > 1e-3f ? 2.0f * sinf(alpha) / dist : 0.0f;

  // Signed lateral error to the nearest segment for telemetry and tests.
  int nxt = next_index(path, nearest);
  float tx = path->x[nxt] - path->x[nearest];
  float ty = path->y[nxt] - path->y[nearest];
  float tlen = sqrtf(tx * tx + ty * ty);
  float lateral_error = 0.0f;
  if (tlen > 1e-6f) {
    float ex = x - path->x[nearest];
    float ey = y - path->y[nearest];
    lateral_error = (tx * ey - ty * ex) / tlen;  // positive: vehicle left of path
    lateral_error = -lateral_error;              // report path relative to vehicle
  }

  out->steer = mcq_steer_from_curvature(bike, curvature, speed);
  out->curvature = curvature;
  out->lookahead = lookahead;
  out->nearest_index = nearest;
  out->target_index = target;
  out->lateral_error = lateral_error;
  return true;
}
