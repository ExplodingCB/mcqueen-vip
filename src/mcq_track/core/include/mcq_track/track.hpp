#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace mcq_track
{
struct Point
{
  double x;
  double y;
};
struct Sample
{
  double x;
  double y;
  double right;
  double left;
};
struct Cartesian
{
  double x;
  double y;
  double heading;
};

// Immutable after construction, safe to share between estimator and track server.
// No ROS, YAML or Python dependencies. CSV and message adapters use this same core.
class Track
{
public:
  Track(std::vector<Sample> samples, bool closed = true, std::string id = "unnamed");
  static Track from_csv(
    const std::string & path, bool closed = true, const std::string & id = "unnamed");
  std::pair<double, double> frenet(double x, double y) const;
  Cartesian cartesian(double s, double d = 0.0) const;
  double heading_at(double s) const;
  double curvature_at(double s) const;
  std::pair<double, double> width_at(double s) const;  // left, right
  double distance_to_edge(double x, double y) const;   // signed polygon distance, before inflation
  bool inside(double x, double y, double margin = 0.0) const;
  const std::vector<Sample> & samples() const { return samples_; }
  const std::vector<double> & arc_lengths() const { return s_; }
  bool closed() const { return closed_; }
  double length() const { return length_; }
  const std::string & id() const { return id_; }

private:
  struct Node
  {
    std::size_t point;
    int left;
    int right;
    int axis;
  };
  struct Spline
  {
    std::vector<std::array<double, 4>> coefficients;
  };
  Spline spline(bool x) const;
  std::array<double, 3> evaluate(const Spline & spline, double s) const;
  double wrap(double s) const;
  std::size_t segment(double s) const;
  int build_tree(std::vector<std::size_t> & ids, std::size_t begin, std::size_t end, int axis);
  void nearest(int node, Point p, std::size_t & best, double & distance) const;
  std::vector<Sample> samples_;
  std::vector<double> s_;
  bool closed_;
  std::string id_;
  double length_;
  Spline sx_, sy_;
  std::vector<Node> tree_;
  int root_;
  std::vector<std::vector<Point>> rings_;
};

// Planar covariance maximum eigenvalue. Reject malformed, asymmetric, non-PSD
// or non-finite covariance as well as an untrusted finite estimate.
bool covariance_violation(const std::array<double, 36> & covariance, double max_variance);
}  // namespace mcq_track
