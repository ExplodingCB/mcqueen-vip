#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mcq_msgs/msg/ego_state.hpp"
#include "mcq_msgs/msg/geofence_state.hpp"
#include "mcq_msgs/msg/track_model.hpp"
#include "mcq_msgs/srv/load_track.hpp"
#include "mcq_track/track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "yaml-cpp/yaml.h"

namespace mcq_track
{
using mcq_msgs::msg::EgoState;
using mcq_msgs::msg::GeofenceState;
using mcq_msgs::msg::TrackModel;
using mcq_msgs::srv::LoadTrack;

class TrackServer : public rclcpp::Node
{
public:
  explicit TrackServer(const rclcpp::NodeOptions & options) : Node("track_server", options)
  {
    margin_ = declare_parameter<double>("geofence_margin", .5);
    variance_max_ = declare_parameter<double>("position_variance_max", .25);
    ego_max_age_ = declare_parameter<double>("ego_max_age", .2);
    if (
      !std::isfinite(margin_) || margin_ < 0 || !std::isfinite(variance_max_) ||
      variance_max_ <= 0 || !std::isfinite(ego_max_age_) || ego_max_age_ <= 0)
      throw std::invalid_argument("invalid geofence parameters");
    const std::string directory = declare_parameter<std::string>("track_dir", "");
    if (directory.empty()) throw std::invalid_argument("track_dir is required");
    pub_model_ =
      create_publisher<TrackModel>("track_model", rclcpp::QoS(1).reliable().transient_local());
    pub_geofence_ = create_publisher<GeofenceState>("geofence_state", rclcpp::SensorDataQoS());
    load(directory);
    sub_ego_ = create_subscription<EgoState>(
      "ego_state", rclcpp::SensorDataQoS(), [this](EgoState::SharedPtr msg) {
        ego_ = msg;
        publish_geofence();
      });
    service_ = create_service<LoadTrack>(
      "mcq/load_track",
      [this](const LoadTrack::Request::SharedPtr request, LoadTrack::Response::SharedPtr response) {
        try {
          load(request->path);
          response->success = true;
          response->message = "track loaded";
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "track reload rejected: %s", error.what());
        }
        response->track_id = track_->id();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      publish_model();
      publish_geofence();
    });
  }

private:
  void load(const std::string & directory)
  {
    const auto path = std::filesystem::path(directory);
    const auto metadata = YAML::LoadFile((path / "track.yaml").string());
    if (!metadata.IsMap()) throw std::invalid_argument("track.yaml must contain a mapping");
    const bool closed = metadata["closed"] ? metadata["closed"].as<bool>() : true;
    const auto id =
      metadata["track_id"] ? metadata["track_id"].as<std::string>() : path.filename().string();
    auto candidate =
      std::make_unique<Track>(Track::from_csv((path / "track.csv").string(), closed, id));
    TrackModel model;
    model.header.frame_id = "map";
    model.closed = closed;
    model.track_id = id;
    for (const auto & sample : candidate->samples()) {
      if (
        sample.left > std::numeric_limits<float>::max() ||
        sample.right > std::numeric_limits<float>::max())
        throw std::invalid_argument("track widths exceed TrackModel representation");
      geometry_msgs::msg::Point point;
      point.x = sample.x;
      point.y = sample.y;
      model.centerline.push_back(point);
      model.width_left.push_back(static_cast<float>(sample.left));
      model.width_right.push_back(static_cast<float>(sample.right));
    }
    if (metadata["raceline"]) {
      const auto raceline_path = path / metadata["raceline"].as<std::string>();
      std::ifstream file(raceline_path);
      if (!file) throw std::invalid_argument("cannot open raceline CSV");
      std::string line;
      double previous_s = -1;
      std::vector<Sample> raceline_samples;
      while (std::getline(file, line)) {
        line = line.substr(0, line.find('#'));
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::stringstream row(line);
        std::string field;
        std::vector<double> values;
        while (std::getline(row, field, ';')) {
          std::size_t used = 0;
          const double value = std::stod(field, &used);
          if (!std::isfinite(value) || field.find_first_not_of(" \t\r", used) != std::string::npos)
            throw std::invalid_argument("invalid raceline value");
          values.push_back(value);
        }
        if (
          values.size() != 7 || line.back() == ';' || values[0] < 0 || values[0] <= previous_s ||
          values[5] < 0)
          throw std::invalid_argument(
            "raceline requires seven finite columns, increasing s and nonnegative speed");
        previous_s = values[0];
        const auto sd = candidate->frenet(values[1], values[2]);
        const auto widths = candidate->width_at(sd.first);
        if (widths.first - sd.second <= 0 || widths.second + sd.second <= 0)
          throw std::invalid_argument("raceline must stay between surveyed edges");
        geometry_msgs::msg::Point point;
        point.x = values[1];
        point.y = values[2];
        raceline_samples.push_back(
          {point.x, point.y, widths.second + sd.second, widths.first - sd.second});
        if (
          std::abs(values[4]) > std::numeric_limits<float>::max() ||
          values[5] > std::numeric_limits<float>::max())
          throw std::invalid_argument("raceline exceeds TrackModel representation");
        model.raceline.push_back(point);
        model.raceline_kappa.push_back(static_cast<float>(values[4]));
        model.raceline_v.push_back(static_cast<float>(values[5]));
      }
      const Track raceline_check(raceline_samples, closed);
      if (raceline_check.samples().size() != raceline_samples.size())
        throw std::invalid_argument("raceline cannot contain duplicate points");
      if (model.raceline.size() < 3)
        throw std::invalid_argument("raceline needs at least three points");
    }
    // All parsing and construction precede the swap. A failed reload preserves
    // the last known-good model, geometry and geofence.
    track_ = std::move(candidate);
    model_ = std::move(model);
    // Evaluate the existing pose immediately against new geometry before the
    // planner receives a new reference.
    publish_geofence();
    publish_model();
    RCLCPP_INFO(get_logger(), "loaded '%s', %.3f m", track_->id().c_str(), track_->length());
  }
  void publish_model()
  {
    model_.header.stamp = now();
    pub_model_->publish(model_);
  }
  void publish_geofence()
  {
    // The controller holds authority until the first verdict. Avoid inventing a
    // startup fault before localization has produced its first observation.
    if (!ego_) return;
    GeofenceState message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    message.margin = static_cast<float>(margin_);
    const double age = (now() - rclcpp::Time(ego_->header.stamp)).seconds();
    const double x = ego_->pose.position.x, y = ego_->pose.position.y;
    if (ego_->header.frame_id != "map" || !std::isfinite(x) || !std::isfinite(y)) {
      message.violation = true;
      message.reason = GeofenceState::REASON_POSE_INVALID;
    } else {
      const double distance = track_->distance_to_edge(x, y) + margin_;
      message.distance_to_edge = static_cast<float>(distance);
      if (age < -.05 || age > ego_max_age_) {
        message.violation = true;
        message.reason = GeofenceState::REASON_POSE_STALE;
      } else if (covariance_violation(ego_->covariance, variance_max_)) {
        message.violation = true;
        message.reason = GeofenceState::REASON_COVARIANCE;
      } else if (distance < 0) {
        message.violation = true;
        message.reason = GeofenceState::REASON_OUTSIDE_TRACK;
      }
    }
    pub_geofence_->publish(message);
  }
  double margin_, variance_max_, ego_max_age_;
  std::unique_ptr<Track> track_;
  TrackModel model_;
  EgoState::SharedPtr ego_;
  rclcpp::Publisher<TrackModel>::SharedPtr pub_model_;
  rclcpp::Publisher<GeofenceState>::SharedPtr pub_geofence_;
  rclcpp::Subscription<EgoState>::SharedPtr sub_ego_;
  rclcpp::Service<LoadTrack>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mcq_track
RCLCPP_COMPONENTS_REGISTER_NODE(mcq_track::TrackServer)
