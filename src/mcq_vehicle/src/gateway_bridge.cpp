// SocketCAN bridge to the safety gateway (docs/02-architecture.md section 4).
//
// Reads GATEWAY_* frames, assembles VehicleState, GatewayStatus and Imu, and
// packs every VehicleCommand into a JETSON_COMMAND frame with the CRC. It
// never invents commands: when the controller stops publishing, the frames
// stop and the gateway's heartbeat timeout does its job.

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

#include "gateway/crc8.h"
#include "mcq_msgs/msg/gateway_status.hpp"
#include "mcq_msgs/msg/vehicle_command.hpp"
#include "mcq_msgs/msg/vehicle_state.hpp"
#include "mcqueen.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mcq_vehicle
{

using mcq_msgs::msg::GatewayStatus;
using mcq_msgs::msg::VehicleCommand;
using mcq_msgs::msg::VehicleState;
using sensor_msgs::msg::Imu;

class GatewayBridge : public rclcpp::Node
{
public:
  explicit GatewayBridge(const rclcpp::NodeOptions & options)
  : rclcpp::Node("gateway_bridge", options)
  {
    interface_ = declare_parameter<std::string>("interface", "can0");
    const double poll_hz = declare_parameter<double>("poll_hz", 1000.0);
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");

    pub_state_ = create_publisher<VehicleState>("vehicle_state", 10);
    pub_status_ = create_publisher<GatewayStatus>("gateway_status", 10);
    pub_imu_ = create_publisher<Imu>("imu/data_raw", 50);
    sub_cmd_ = create_subscription<VehicleCommand>(
      "vehicle_command", 10, [this](const VehicleCommand::SharedPtr msg) { send_command(*msg); });

    open_socket();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / poll_hz), [this]() { poll(); });
  }

  ~GatewayBridge() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  bool open_socket()
  {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "socket(PF_CAN): %s", std::strerror(errno));
      return false;
    }
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "no CAN interface '%s': %s", interface_.c_str(), std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind(%s): %s", interface_.c_str(), std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    RCLCPP_INFO(get_logger(), "listening on %s", interface_.c_str());
    return true;
  }

  void poll()
  {
    if (fd_ < 0) {
      // Retry once a second so the node can start before the interface is up.
      const auto now = std::chrono::steady_clock::now();
      if (now - last_open_attempt_ > std::chrono::seconds(1)) {
        last_open_attempt_ = now;
        open_socket();
      }
      return;
    }
    struct can_frame frame;
    for (;;) {
      const ssize_t n = ::read(fd_, &frame, sizeof(frame));
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "read(%s): %s", interface_.c_str(),
            std::strerror(errno));
        }
        break;
      }
      if (n < static_cast<ssize_t>(sizeof(frame))) {
        break;
      }
      handle(frame);
    }
  }

  void handle(const struct can_frame & f)
  {
    if (f.len < 8 || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {
      return;
    }
    const uint32_t id = f.can_id & CAN_SFF_MASK;
    switch (id) {
      case MCQUEEN_GATEWAY_STEERING_FRAME_ID: {
        struct mcqueen_gateway_steering_t m;
        mcqueen_gateway_steering_unpack(&m, f.data, 8);
        state_.steering_angle =
          static_cast<float>(mcqueen_gateway_steering_steering_angle_decode(m.steering_angle));
        state_.steering_rate =
          static_cast<float>(mcqueen_gateway_steering_steering_rate_decode(m.steering_rate));
        state_.brake_pressure =
          static_cast<float>(mcqueen_gateway_steering_brake_pressure_decode(m.brake_pressure));
        break;
      }
      case MCQUEEN_GATEWAY_WHEELS_FRAME_ID: {
        struct mcqueen_gateway_wheels_t m;
        mcqueen_gateway_wheels_unpack(&m, f.data, 8);
        state_.wheel_speed[0] =
          static_cast<float>(mcqueen_gateway_wheels_wheel_speed_fl_decode(m.wheel_speed_fl));
        state_.wheel_speed[1] =
          static_cast<float>(mcqueen_gateway_wheels_wheel_speed_fr_decode(m.wheel_speed_fr));
        state_.wheel_speed[2] =
          static_cast<float>(mcqueen_gateway_wheels_wheel_speed_rl_decode(m.wheel_speed_rl));
        state_.wheel_speed[3] =
          static_cast<float>(mcqueen_gateway_wheels_wheel_speed_rr_decode(m.wheel_speed_rr));
        break;
      }
      case MCQUEEN_GATEWAY_MOTOR_FRAME_ID: {
        struct mcqueen_gateway_motor_t m;
        mcqueen_gateway_motor_unpack(&m, f.data, 8);
        state_.motor_rpm = static_cast<float>(mcqueen_gateway_motor_motor_rpm_decode(m.motor_rpm));
        state_.motor_current =
          static_cast<float>(mcqueen_gateway_motor_motor_current_decode(m.motor_current));
        state_.motor_power =
          static_cast<float>(mcqueen_gateway_motor_motor_power_decode(m.motor_power));
        state_.battery_voltage =
          static_cast<float>(mcqueen_gateway_motor_battery_voltage_decode(m.battery_voltage));
        break;
      }
      case MCQUEEN_GATEWAY_STATUS_FRAME_ID: {
        // The status frame is the 100 Hz anchor: publish everything gathered.
        struct mcqueen_gateway_status_t m;
        mcqueen_gateway_status_unpack(&m, f.data, 8);
        const auto stamp = now();
        state_.header.stamp = stamp;
        state_.header.frame_id = "base_link";
        state_.mode = m.mode;
        state_.fault_flags = m.fault_flags;
        state_.heartbeat_echo = m.heartbeat_echo;
        pub_state_->publish(state_);

        GatewayStatus st;
        st.header = state_.header;
        st.mode = m.mode;
        st.fault_flags = m.fault_flags;
        st.fault_latched = latched_ |= m.fault_flags;
        st.heartbeat_echo = m.heartbeat_echo;
        st.frames_rejected = m.frames_rejected;
        st.rc_link_ok = m.rc_link_ok != 0;
        st.remote_estop_asserted = m.remote_estop != 0;
        st.contactor_closed = m.contactor_closed != 0;
        st.tx_auto_switch = m.tx_auto_switch != 0;
        st.speed = static_cast<float>(mcqueen_gateway_status_speed_decode(m.speed));
        pub_status_->publish(st);
        break;
      }
      case MCQUEEN_GATEWAY_IMU_ACCEL_FRAME_ID: {
        struct mcqueen_gateway_imu_accel_t m;
        mcqueen_gateway_imu_accel_unpack(&m, f.data, 8);
        imu_.linear_acceleration.x = mcqueen_gateway_imu_accel_accel_x_decode(m.accel_x);
        imu_.linear_acceleration.y = mcqueen_gateway_imu_accel_accel_y_decode(m.accel_y);
        imu_.linear_acceleration.z = mcqueen_gateway_imu_accel_accel_z_decode(m.accel_z);
        accel_stamp_ = m.timestamp_ms;
        have_accel_ = true;
        publish_imu_if_complete();
        break;
      }
      case MCQUEEN_GATEWAY_IMU_GYRO_FRAME_ID: {
        struct mcqueen_gateway_imu_gyro_t m;
        mcqueen_gateway_imu_gyro_unpack(&m, f.data, 8);
        imu_.angular_velocity.x = mcqueen_gateway_imu_gyro_gyro_x_decode(m.gyro_x);
        imu_.angular_velocity.y = mcqueen_gateway_imu_gyro_gyro_y_decode(m.gyro_y);
        imu_.angular_velocity.z = mcqueen_gateway_imu_gyro_gyro_z_decode(m.gyro_z);
        gyro_stamp_ = m.timestamp_ms;
        have_gyro_ = true;
        publish_imu_if_complete();
        break;
      }
      case MCQUEEN_GATEWAY_LIMITS_FRAME_ID: {
        struct mcqueen_gateway_limits_t m;
        mcqueen_gateway_limits_unpack(&m, f.data, 8);
        RCLCPP_INFO_ONCE(
          get_logger(),
          "gateway limits: steer %.3f rad, rate %.2f rad/s, speed cap %.2f m/s, power cap %.0f W",
          mcqueen_gateway_limits_steer_angle_max_decode(m.steer_angle_max),
          mcqueen_gateway_limits_steer_rate_max_decode(m.steer_rate_max),
          mcqueen_gateway_limits_speed_cap_decode(m.speed_cap),
          mcqueen_gateway_limits_power_cap_decode(m.power_cap));
        break;
      }
      default:
        break;
    }
  }

  void publish_imu_if_complete()
  {
    if (!have_accel_ || !have_gyro_ || accel_stamp_ != gyro_stamp_) {
      return;
    }
    imu_.header.stamp = now();
    imu_.header.frame_id = imu_frame_;
    imu_.orientation_covariance[0] = -1.0;  // no orientation estimate
    pub_imu_->publish(imu_);
    have_accel_ = have_gyro_ = false;
  }

  template <typename T>
  static T quantize(float value, float scale, float lo, float hi)
  {
    const float clamped = std::min(std::max(value, lo), hi);
    return static_cast<T>(std::lround(clamped / scale));
  }

  void send_command(const VehicleCommand & cmd)
  {
    if (fd_ < 0) {
      return;
    }
    if (!std::isfinite(cmd.steering_angle) || !std::isfinite(cmd.throttle) || !std::isfinite(cmd.brake)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "non-finite command field, not sent");
      return;
    }
    struct mcqueen_jetson_command_t c;
    std::memset(&c, 0, sizeof(c));
    // Round, as the gateway does; the generated *_encode helpers truncate.
    c.steering_angle = quantize<int16_t>(cmd.steering_angle, 0.0001f, -3.2768f, 3.2767f);
    c.throttle = quantize<uint16_t>(cmd.throttle, 0.001f, 0.0f, 1.0f);
    c.brake = quantize<uint16_t>(cmd.brake, 0.001f, 0.0f, 1.0f);
    c.lat_enable = cmd.lat_enable ? 1 : 0;
    c.long_enable = cmd.long_enable ? 1 : 0;
    c.request_urgent_stop = cmd.request_urgent_stop ? 1 : 0;
    c.heartbeat = cmd.heartbeat;

    struct can_frame f;
    std::memset(&f, 0, sizeof(f));
    f.can_id = MCQUEEN_JETSON_COMMAND_FRAME_ID;
    f.len = 8;
    mcqueen_jetson_command_pack(f.data, &c, 8);
    f.data[7] = gw_crc8(f.data, 7);
    if (::write(fd_, &f, sizeof(f)) != static_cast<ssize_t>(sizeof(f))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "write(%s): %s", interface_.c_str(), std::strerror(errno));
    }
  }

  std::string interface_;
  std::string imu_frame_;
  int fd_{-1};
  std::chrono::steady_clock::time_point last_open_attempt_{};
  VehicleState state_;
  Imu imu_;
  uint32_t latched_{0};
  uint16_t accel_stamp_{0};
  uint16_t gyro_stamp_{0};
  bool have_accel_{false};
  bool have_gyro_{false};
  rclcpp::Publisher<VehicleState>::SharedPtr pub_state_;
  rclcpp::Publisher<GatewayStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mcq_vehicle

RCLCPP_COMPONENTS_REGISTER_NODE(mcq_vehicle::GatewayBridge)
