#include "mcq_track/track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace mcq_track
{
namespace
{
void finite(double value)
{
  if (!std::isfinite(value)) throw std::invalid_argument("track geometry must be finite");
}
std::vector<double> tridiagonal(
  const std::vector<double> & lower, std::vector<double> diagonal,
  const std::vector<double> & upper, std::vector<double> rhs)
{
  for (std::size_t i = 1; i < diagonal.size(); ++i) {
    const double f = lower[i] / diagonal[i - 1];
    diagonal[i] -= f * upper[i - 1];
    rhs[i] -= f * rhs[i - 1];
  }
  rhs.back() /= diagonal.back();
  for (std::size_t i = rhs.size() - 1; i > 0; --i)
    rhs[i - 1] = (rhs[i - 1] - upper[i - 1] * rhs[i]) / diagonal[i - 1];
  return rhs;
}
double segment_distance(Point p, Point a, Point b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double norm = dx * dx + dy * dy;
  const double t =
    norm > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / norm, 0.0, 1.0) : 0;
  return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
}  // namespace

Track::Track(std::vector<Sample> samples, bool closed, std::string id)
: closed_(closed), id_(std::move(id)), length_(0), root_(-1)
{
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto & p = samples[i];
    finite(p.x);
    finite(p.y);
    finite(p.left);
    finite(p.right);
    if (p.left <= 0 || p.right <= 0) throw std::invalid_argument("track widths must be positive");
    // Match the Python reference: deduplicate against the original preceding row.
    if (i > 0 && std::hypot(p.x - samples[i - 1].x, p.y - samples[i - 1].y) <= 1e-6) continue;
    if (
      closed && i > 0 && i == samples.size() - 1 &&
      std::hypot(p.x - samples.front().x, p.y - samples.front().y) <= 1e-6)
      continue;
    samples_.push_back(p);
  }
  if (samples_.size() < 3)
    throw std::invalid_argument("a track needs at least three distinct points");
  s_.push_back(0);
  for (std::size_t i = 1; i < samples_.size(); ++i)
    s_.push_back(
      s_.back() + std::hypot(samples_[i].x - samples_[i - 1].x, samples_[i].y - samples_[i - 1].y));
  length_ = s_.back();
  if (closed_)
    length_ +=
      std::hypot(samples_.front().x - samples_.back().x, samples_.front().y - samples_.back().y);
  if (!std::isfinite(length_) || length_ <= 0)
    throw std::invalid_argument("track arc length must be finite and positive");
  for (std::size_t i = 1; i < s_.size(); ++i)
    if (s_[i] <= s_[i - 1]) throw std::invalid_argument("track arc length must increase");
  sx_ = spline(true);
  sy_ = spline(false);
  for (const auto * curve : {&sx_, &sy_})
    for (const auto & coefficients : curve->coefficients)
      for (double coefficient : coefficients) finite(coefficient);
  std::vector<std::size_t> ids(samples_.size());
  std::iota(ids.begin(), ids.end(), 0);
  tree_.reserve(ids.size());
  root_ = build_tree(ids, 0, ids.size(), 0);
  std::vector<Point> left, right;
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const auto & p = samples_[i];
    const double psi = heading_at(s_[i]);
    left.push_back({p.x - p.left * std::sin(psi), p.y + p.left * std::cos(psi)});
    right.push_back({p.x + p.right * std::sin(psi), p.y - p.right * std::cos(psi)});
    finite(left.back().x);
    finite(left.back().y);
    finite(right.back().x);
    finite(right.back().y);
  }
  if (closed_) {
    rings_.push_back(left);
    rings_.push_back(right);
  } else {
    left.insert(left.end(), right.rbegin(), right.rend());
    rings_.push_back(left);
  }
}

Track Track::from_csv(const std::string & path, bool closed, const std::string & id)
{
  std::ifstream file(path);
  if (!file) throw std::invalid_argument("cannot open track CSV: " + path);
  std::vector<Sample> samples;
  std::string line;
  while (std::getline(file, line)) {
    line = line.substr(0, line.find('#'));
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    std::stringstream row(line);
    std::vector<double> values;
    std::string field;
    while (std::getline(row, field, ',')) {
      std::size_t used = 0;
      const double value = std::stod(field, &used);
      if (field.find_first_not_of(" \t\r", used) != std::string::npos)
        throw std::invalid_argument("invalid CSV value");
      values.push_back(value);
    }
    if (values.size() != 4 || line.back() == ',')
      throw std::invalid_argument("track CSV requires four columns");
    samples.push_back({values[0], values[1], values[2], values[3]});
  }
  return Track(std::move(samples), closed, id);
}

Track::Spline Track::spline(bool x) const
{
  const std::size_t n = samples_.size();
  const std::size_t segments = closed_ ? n : n - 1;
  std::vector<double> h(segments), slope(segments), lower(n), diagonal(n), upper(n), rhs(n);
  const auto value = [&](std::size_t i) { return x ? samples_[i % n].x : samples_[i % n].y; };
  for (std::size_t i = 0; i < segments; ++i) {
    h[i] = (i + 1 == n ? length_ : s_[i + 1]) - s_[i];
    slope[i] = (value(i + 1) - value(i)) / h[i];
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!closed_ && (i == 0 || i == n - 1)) {
      diagonal[i] = 1;
      continue;
    }
    const std::size_t prev = (i + n - 1) % n;
    lower[i] = h[prev];
    upper[i] = h[i];
    diagonal[i] = 2 * (h[prev] + h[i]);
    rhs[i] = 6 * (slope[i] - slope[prev]);
  }
  std::vector<double> second;
  if (closed_) {
    // Sherman-Morrison reduction of the cyclic tridiagonal periodic system.
    const double corner = h.back(), gamma = -diagonal.front();
    diagonal.front() -= gamma;
    diagonal.back() -= corner * corner / gamma;
    second = tridiagonal(lower, diagonal, upper, rhs);
    std::vector<double> u(n, 0);
    u.front() = gamma;
    u.back() = corner;
    const auto correction = tridiagonal(lower, diagonal, upper, u);
    const double factor = (second.front() + corner * second.back() / gamma) /
                          (1 + correction.front() + corner * correction.back() / gamma);
    for (std::size_t i = 0; i < n; ++i) second[i] -= factor * correction[i];
  } else
    second = tridiagonal(lower, diagonal, upper, rhs);
  Spline result;
  for (std::size_t i = 0; i < segments; ++i) {
    const double next = second[(i + 1) % n];
    result.coefficients.push_back(
      {value(i), slope[i] - h[i] * (2 * second[i] + next) / 6, second[i] / 2,
       (next - second[i]) / (6 * h[i])});
  }
  return result;
}

double Track::wrap(double s) const
{
  finite(s);
  if (!closed_) return std::clamp(s, 0.0, length_);
  const double wrapped = std::fmod(s, length_);
  return wrapped < 0 ? wrapped + length_ : wrapped;
}
std::size_t Track::segment(double s) const
{
  const auto i = std::upper_bound(s_.begin(), s_.end(), s) - s_.begin();
  return std::min(static_cast<std::size_t>(i - 1), sx_.coefficients.size() - 1);
}
std::array<double, 3> Track::evaluate(const Spline & spline, double s) const
{
  const auto i = segment(s);
  const auto & c = spline.coefficients[i];
  const double t = s - s_[i];
  return {
    ((c[3] * t + c[2]) * t + c[1]) * t + c[0], (3 * c[3] * t + 2 * c[2]) * t + c[1],
    6 * c[3] * t + 2 * c[2]};
}
Cartesian Track::cartesian(double s, double d) const
{
  finite(d);
  s = wrap(s);
  const auto x = evaluate(sx_, s), y = evaluate(sy_, s);
  const double psi = std::atan2(y[1], x[1]);
  return {x[0] - d * std::sin(psi), y[0] + d * std::cos(psi), psi};
}
double Track::heading_at(double s) const { return cartesian(s).heading; }
double Track::curvature_at(double s) const
{
  s = wrap(s);
  const auto x = evaluate(sx_, s), y = evaluate(sy_, s);
  return (x[1] * y[2] - y[1] * x[2]) / std::max(std::pow(std::hypot(x[1], y[1]), 3), 1e-9);
}
std::pair<double, double> Track::width_at(double s) const
{
  s = wrap(s);
  const auto i = segment(s), next = (i + 1) % samples_.size();
  const double end = next == 0 ? length_ : s_[next];
  const double t = (s - s_[i]) / (end - s_[i]);
  return {
    samples_[i].left + t * (samples_[next].left - samples_[i].left),
    samples_[i].right + t * (samples_[next].right - samples_[i].right)};
}
int Track::build_tree(std::vector<std::size_t> & ids, std::size_t begin, std::size_t end, int axis)
{
  if (begin == end) return -1;
  const auto middle = begin + (end - begin) / 2;
  std::nth_element(
    ids.begin() + begin, ids.begin() + middle, ids.begin() + end,
    [&](std::size_t a, std::size_t b) {
      const double av = axis == 0 ? samples_[a].x : samples_[a].y;
      const double bv = axis == 0 ? samples_[b].x : samples_[b].y;
      return av == bv ? a < b : av < bv;
    });
  const int node = static_cast<int>(tree_.size());
  tree_.push_back({ids[middle], -1, -1, axis});
  const int left = build_tree(ids, begin, middle, 1 - axis);
  const int right = build_tree(ids, middle + 1, end, 1 - axis);
  tree_[node].left = left;
  tree_[node].right = right;
  return node;
}
void Track::nearest(int node, Point p, std::size_t & best, double & distance) const
{
  if (node < 0) return;
  const auto & item = tree_[node];
  const auto & q = samples_[item.point];
  const double dist = (p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y);
  if (dist < distance || (dist == distance && item.point < best)) {
    distance = dist;
    best = item.point;
  }
  const double delta = item.axis == 0 ? p.x - q.x : p.y - q.y;
  nearest(delta < 0 ? item.left : item.right, p, best, distance);
  if (delta * delta <= distance) nearest(delta < 0 ? item.right : item.left, p, best, distance);
}
std::pair<double, double> Track::frenet(double x, double y) const
{
  finite(x);
  finite(y);
  std::size_t index = 0;
  double dist = std::numeric_limits<double>::infinity();
  nearest(root_, {x, y}, index, dist);
  const auto n = samples_.size();
  const auto prev = index == 0 ? (closed_ ? n - 1 : 0) : index - 1;
  const auto next = index + 1 == n ? (closed_ ? 0 : index) : index + 1;
  std::pair<double, double> result{0, 0};
  double best = std::numeric_limits<double>::infinity();
  for (const auto & pair : {std::pair{prev, index}, std::pair{index, next}}) {
    const auto & a = samples_[pair.first];
    const auto & b = samples_[pair.second];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length <= 1e-9) continue;
    const double tx = (b.x - a.x) / length, ty = (b.y - a.y) / length;
    const double px = x - a.x, py = y - a.y, t = std::clamp(px * tx + py * ty, 0.0, length);
    const double distance = std::hypot(x - (a.x + t * tx), y - (a.y + t * ty));
    if (distance < best) {
      best = distance;
      result = {wrap(s_[pair.first] + t), tx * py - ty * px};
    }
  }
  return result;
}
double Track::distance_to_edge(double x, double y) const
{
  finite(x);
  finite(y);
  bool contained = false;
  double distance = std::numeric_limits<double>::infinity();
  for (const auto & ring : rings_) {
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
      const auto a = ring[j], b = ring[i];
      distance = std::min(distance, segment_distance({x, y}, a, b));
      if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)
        contained = !contained;
    }
  }
  return contained ? distance : -distance;
}
bool Track::inside(double x, double y, double margin) const
{
  finite(margin);
  if (margin < 0) throw std::invalid_argument("geofence inflation must be nonnegative");
  return distance_to_edge(x, y) >= -margin;
}
bool covariance_violation(const std::array<double, 36> & covariance, double max_variance)
{
  if (!std::isfinite(max_variance) || max_variance <= 0)
    throw std::invalid_argument("variance threshold must be positive");
  const double xx = covariance[0], xy = covariance[1], yx = covariance[6], yy = covariance[7];
  if (
    !std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yx) || !std::isfinite(yy) ||
    xx < 0 || yy < 0 || std::abs(xy - yx) > 1e-9 || xy * xy > xx * yy + 1e-12)
    return true;
  const double largest = 0.5 * (xx + yy + std::hypot(xx - yy, 2 * xy));
  return !std::isfinite(largest) || largest > max_variance;
}
}  // namespace mcq_track
