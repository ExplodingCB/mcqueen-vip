#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "mcq_track/track.hpp"

void check(bool condition)
{
  if (!condition) throw std::runtime_error("track check failed");
}
int main()
{
  try {
    mcq_track::Track open({{0, 0, 1, 2}, {5, 0, 1, 2}, {10, 0, 1, 2}}, false);
    check(open.inside(5, 2.49, 0.5));
    check(!open.inside(5, 2.51, 0.5));
    check(open.inside(-0.49, 0, 0.5));
    check(!open.inside(-0.51, 0, 0.5));
    check(!open.inside(11, 0));  // Frenet d alone would miss an open-track end.
    check(std::abs(open.distance_to_edge(5, 0) - 1) < 1e-12);
    check(open.cartesian(-1).x == 0);
    check(open.cartesian(20).x == 10);
    check(open.frenet(12, 1).first == 10);
    mcq_track::Track closed(
      {{0, 0, 0.3, 0.3}, {10, 0, 0.3, 0.3}, {10, 10, 0.3, 0.3}, {0, 10, 0.3, 0.3}});
    check(!closed.inside(5, 5));
    check(closed.inside(5, 0, 0.5));
    std::array<double, 36> covariance{};
    check(!mcq_track::covariance_violation(covariance, 0.25));
    covariance[0] = covariance[7] = .2;
    covariance[1] = covariance[6] = .1;
    check(mcq_track::covariance_violation(covariance, .25));  // eig .3, diagonal .2
    covariance[1] = covariance[6] = 0;
    check(!mcq_track::covariance_violation(covariance, .25));
    covariance[0] = -1;
    check(mcq_track::covariance_violation(covariance, .25));
    covariance[0] = std::numeric_limits<double>::quiet_NaN();
    check(mcq_track::covariance_violation(covariance, .25));
    covariance[0] = .1;
    covariance[1] = .01;
    check(mcq_track::covariance_violation(covariance, .25));
    bool rejected = false;
    try {
      mcq_track::Track invalid({{0, 0, 1, 1}, {1, 0, -1, 1}, {2, 0, 1, 1}});
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected);
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
