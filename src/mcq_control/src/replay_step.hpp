// Optional transport for deterministic tests of the real ROS node. New nodes
// can reuse this helper with a topic map; the replay driver has no node logic.
#pragma once

#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace mcq_replay
{
class ReplayStep
{
public:
  explicit ReplayStep(rclcpp::Node & node) : node_(node)
  {
    enabled_ = node_.declare_parameter<bool>("replay_mode", false);
    if (enabled_) {
      received_ = node_.create_publisher<std_msgs::msg::Header>("replay/received", qos());
    }
  }

  bool enabled() const { return enabled_; }

  rclcpp::QoS qos() const
  {
    if (enabled_) {
      return rclcpp::QoS(rclcpp::KeepAll()).reliable();
    }
    return rclcpp::SensorDataQoS();
  }

  void received(const std::string & topic, const std_msgs::msg::Header & header)
  {
    if (enabled_) {
      auto receipt = header;
      receipt.frame_id = topic;
      received_->publish(receipt);
    }
  }

  void start(std::function<void(const rclcpp::Time &)> step)
  {
    tick_ = node_.create_subscription<std_msgs::msg::Header>(
      "replay/tick", qos(),
      [step](const std_msgs::msg::Header::SharedPtr msg) { step(rclcpp::Time(msg->stamp)); });
  }

private:
  rclcpp::Node & node_;
  bool enabled_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr received_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr tick_;
};
}  // namespace mcq_replay
