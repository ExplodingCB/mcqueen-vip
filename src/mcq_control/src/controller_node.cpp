// Composable controller node (docs/02-architecture.md section 8).
//
// Subscribes Trajectory, EgoState and VehicleState, runs the C control core at
// 100 Hz and publishes VehicleCommand with the heartbeat counter. Every check
// from docs/04-safety.md section 5 that belongs to the controller (trajectory
// age, ego age) turns into request_urgent_stop; the gateway does the rest.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "mcq/bicycle.h"
#include "mcq/limits.h"
#include "mcq/longitudinal.h"
#include "mcq/pure_pursuit.h"
#include "mcq_msgs/msg/ego_state.hpp"
#include "mcq_msgs/msg/geofence_state.hpp"
#include "mcq_msgs/msg/trajectory.hpp"
#include "mcq_msgs/msg/vehicle_command.hpp"
#include "mcq_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "replay_step.hpp"

namespace mcq_control
{

using mcq_msgs::msg::EgoState;
using mcq_msgs::msg::GeofenceState;
using mcq_msgs::msg::Trajectory;
using mcq_msgs::msg::VehicleCommand;
using mcq_msgs::msg::VehicleState;

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("controller", options), replay_(*this)
  {
    const double rate_hz = declare_parameter<double>("rate_hz", 100.0);
    dt_ = static_cast<float>(1.0 / rate_hz);
    trajectory_max_age_ = declare_parameter<double>("trajectory_max_age", 0.2);
    ego_max_age_ = declare_parameter<double>("ego_max_age", 0.2);
    geofence_max_age_ = declare_parameter<double>("geofence_max_age", 0.2);

    bike_.wheelbase = param("wheelbase", 1.05);
    bike_.understeer = param("understeer", 0.002);
    bike_.steer_max = param("steer_max", 0.45);

    pp_.k_v = param("lateral.k_v", 0.5);
    pp_.l_min = param("lateral.l_min", 1.5);
    pp_.l_max = param("lateral.l_max", 6.0);
    lim_.a_lat_max = param("lateral.a_lat_max", 4.0);
    lim_.jerk_lat_max = param("lateral.jerk_lat_max", 6.0);
    lim_.steer_max = bike_.steer_max;
    lim_.steer_rate_max = param("lateral.steer_rate_max", 2.5);
    lim_.v_min = 1.0f;

    mcq_long_params_t lp{};
    lp.pid.k_p = mcq_gain_constant(param("longitudinal.k_p", 0.8));
    lp.pid.k_i = mcq_gain_constant(param("longitudinal.k_i", 0.2));
    lp.pid.k_f = param("longitudinal.k_f", 1.0);
    lp.pid.pos_limit = param("longitudinal.accel_max", 2.5);
    lp.pid.neg_limit = param("longitudinal.decel_max", -4.0);
    lp.pid.i_rate = dt_;
    lp.pid.i_unwind = 0.0f;
    lp.stopping_speed = param("longitudinal.stopping_speed", 0.5);
    lp.stopping_decel = param("longitudinal.stopping_decel", -1.5);
    lp.accel_max = lp.pid.pos_limit;
    lp.decel_max = lp.pid.neg_limit;
    mcq_long_init(&long_, &lp);

    map_.throttle_per_accel = param("longitudinal.throttle_per_accel", 0.33);
    map_.brake_per_decel = param("longitudinal.brake_per_decel", 0.17);
    map_.deadband = param("longitudinal.deadband", 0.05);
    map_.rolling_decel = param("rolling_decel", 0.3);

    // Best-effort, keep-last: a slow subscriber (the recorder, the pit link)
    // must never back-pressure the control loop. Freshness is checked by age.
    const auto qos = replay_.qos();
    pub_ = create_publisher<VehicleCommand>("vehicle_command", qos);
    sub_traj_ = create_subscription<Trajectory>("trajectory", qos, [this](const Trajectory::SharedPtr msg) {
      if (!traj_) {
        RCLCPP_INFO(
          get_logger(), "first trajectory received, %.0f ms old",
          (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3);
      }
      traj_ = msg;
      replay_.received("trajectory", msg->header);
    });
    sub_ego_ = create_subscription<EgoState>("ego_state", qos, [this](const EgoState::SharedPtr msg) {
      if (!ego_) {
        RCLCPP_INFO(
          get_logger(), "first ego state received, %.0f ms old",
          (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3);
      }
      ego_ = msg;
      replay_.received("ego_state", msg->header);
    });
    sub_geofence_ = create_subscription<GeofenceState>(
      "geofence_state", qos, [this](const GeofenceState::SharedPtr msg) {
        geofence_ = msg;
        replay_.received("geofence_state", msg->header);
      });
    sub_vehicle_ = create_subscription<VehicleState>(
      "vehicle_state", qos, [this](const VehicleState::SharedPtr msg) {
        vehicle_ = msg;
        replay_.received("vehicle_state", msg->header);
      });
    if (replay_.enabled()) {
      replay_.start([this](const rclcpp::Time & stamp) { tick(stamp); });
    } else {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz), [this]() { tick(this->now()); });
    }
    RCLCPP_INFO(get_logger(), "controller running at %.0f Hz", rate_hz);
  }

private:
  float param(const std::string & name, double default_value)
  {
    return static_cast<float>(declare_parameter<double>(name, default_value));
  }

  static double yaw_of(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // Target speed and acceleration at a time offset into the trajectory.
  void target_at(const Trajectory & traj, double tau, float & v_t, float & a_t) const
  {
    const auto & pts = traj.points;
    if (tau <= pts.front().t) {
      v_t = pts.front().v;
      a_t = pts.front().a;
      return;
    }
    for (size_t i = 1; i < pts.size(); ++i) {
      if (tau <= pts[i].t) {
        const double span = pts[i].t - pts[i - 1].t;
        const double f = span > 1e-9 ? (tau - pts[i - 1].t) / span : 0.0;
        v_t = static_cast<float>(pts[i - 1].v + f * (pts[i].v - pts[i - 1].v));
        a_t = static_cast<float>(pts[i - 1].a + f * (pts[i].a - pts[i - 1].a));
        return;
      }
    }
    v_t = pts.back().v;
    a_t = pts.back().a;
  }

  void tick(const rclcpp::Time & now)
  {
    VehicleCommand cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = "base_link";
    cmd.heartbeat = heartbeat_++;

    const bool auto_mode = vehicle_ && vehicle_->mode == VehicleState::MODE_AUTO;
    if (!traj_ || !ego_ || traj_->points.size() < 2) {
      // Hold with no axis authority in RC. The real gateway also honors Jetson
      // stop requests in RC, so only AUTO missing inputs request urgent stop.
      cmd.lat_enable = false;
      cmd.long_enable = false;
      cmd.request_urgent_stop = auto_mode;
      pub_->publish(cmd);
      return;
    }

    const double traj_age = (now - rclcpp::Time(traj_->header.stamp)).seconds();
    const double ego_age = (now - rclcpp::Time(ego_->header.stamp)).seconds();
    const bool stale = traj_age > trajectory_max_age_ || ego_age > ego_max_age_;
    const double geofence_age = geofence_ ? (now - rclcpp::Time(geofence_->header.stamp)).seconds() : 1e9;
    const bool geofence_stop = !geofence_ || geofence_->violation ||
      geofence_age > geofence_max_age_ || geofence_age < -.05;
    // The Jetson-side checks apply while the Jetson is driving. In RC the
    // gateway ignores these commands anyway, and a stop request would brake a
    // human's drive out of the pit for a startup hiccup.
    const bool request_stop = (stale || geofence_stop) && auto_mode;
    if (request_stop && !was_stale_) {
      RCLCPP_WARN(
        get_logger(), "unsafe inputs (trajectory %.0f ms, ego %.0f ms, geofence stop %d): requesting urgent stop",
        traj_age * 1e3, ego_age * 1e3, static_cast<int>(geofence_stop));
    }
    was_stale_ = request_stop;
    const bool stop = traj_->stop_requested || stale || geofence_stop;

    const float x = static_cast<float>(ego_->pose.position.x);
    const float y = static_cast<float>(ego_->pose.position.y);
    const float yaw = static_cast<float>(yaw_of(ego_->pose));
    const float v = ego_->v;

    const size_t n = traj_->points.size();
    px_.resize(n);
    py_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      px_[i] = traj_->points[i].x;
      py_[i] = traj_->points[i].y;
    }
    mcq_path_t path;
    path.x = px_.data();
    path.y = py_.data();
    path.n = static_cast<int>(n);
    path.closed = false;

    mcq_pure_pursuit_result_t res;
    if (!mcq_pure_pursuit(&pp_, &bike_, &path, x, y, yaw, v, &res)) {
      cmd.lat_enable = false;
      cmd.long_enable = false;
      cmd.request_urgent_stop = true;
      pub_->publish(cmd);
      return;
    }
    const float kappa = mcq_limit_curvature(&lim_, res.curvature, prev_kappa_, v, dt_);
    prev_kappa_ = kappa;
    float steer = mcq_steer_from_curvature(&bike_, kappa, v);
    steer = mcq_rate_limit(steer, prev_steer_, lim_.steer_rate_max, dt_);
    prev_steer_ = steer;

    float v_t = 0.0f;
    float a_t = 0.0f;
    target_at(*traj_, traj_age, v_t, a_t);
    const float accel = mcq_long_update(&long_, true, stop, v, v_t, a_t);
    float throttle = 0.0f;
    float brake = 0.0f;
    mcq_actuator_map(&map_, accel, &throttle, &brake);

    cmd.steering_angle = steer;
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.lat_enable = !geofence_stop;
    cmd.long_enable = !geofence_stop;
    cmd.request_urgent_stop = request_stop;
    pub_->publish(cmd);
  }

  mcq_replay::ReplayStep replay_;
  float dt_{0.01f};
  double trajectory_max_age_{0.2};
  double ego_max_age_{0.1};
  double geofence_max_age_{0.2};
  mcq_bicycle_params_t bike_{};
  mcq_pure_pursuit_params_t pp_{};
  mcq_lateral_limits_t lim_{};
  mcq_long_t long_{};
  mcq_actuator_map_t map_{};
  float prev_kappa_{0.0f};
  float prev_steer_{0.0f};
  uint16_t heartbeat_{0};
  bool was_stale_{false};
  std::vector<float> px_;
  std::vector<float> py_;

  Trajectory::SharedPtr traj_;
  EgoState::SharedPtr ego_;
  GeofenceState::SharedPtr geofence_;
  VehicleState::SharedPtr vehicle_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<EgoState>::SharedPtr sub_ego_;
  rclcpp::Subscription<GeofenceState>::SharedPtr sub_geofence_;
  rclcpp::Subscription<VehicleState>::SharedPtr sub_vehicle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mcq_control

RCLCPP_COMPONENTS_REGISTER_NODE(mcq_control::ControllerNode)
