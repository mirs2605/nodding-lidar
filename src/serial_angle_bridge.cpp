#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "nodding_lidar/geometry.hpp"

class SerialAngleBridge : public rclcpp::Node
{
public:
  SerialAngleBridge()
  : Node("serial_angle_bridge")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    state_topic_ = declare_parameter<std::string>("state_topic", "/servo_state");
    joint_name_ = declare_parameter<std::string>("joint_name", "nodding_joint");
    center_angle_deg_ = declare_parameter<double>("center_angle_deg", 90.0);
    angle_sign_ = declare_parameter<double>("angle_sign", 1.0);
    if (!std::isfinite(center_angle_deg_) || !std::isfinite(angle_sign_) || angle_sign_ == 0.0) {
      throw std::invalid_argument("center_angle_deg must be finite and angle_sign must be nonzero");
    }

    fd_ = open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
              "Cannot open servo serial port " + serial_port_ + ": " + std::strerror(errno));
    }

    termios settings{};
    if (tcgetattr(fd_, &settings) != 0) {
      const std::string error = std::strerror(errno);
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("Cannot read serial settings: " + error);
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &settings) != 0) {
      const std::string error = std::strerror(errno);
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("Cannot configure serial port: " + error);
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&SerialAngleBridge::read_serial, this));
    RCLCPP_INFO(
      get_logger(), "Reading estimated servo angle from %s at 115200 baud",
      serial_port_.c_str());
  }

  ~SerialAngleBridge() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void parse_line(const std::string & line)
  {
    unsigned long device_time_ms = 0;
    double angle_deg = 0.0;
    double velocity_deg_s = 0.0;
    char trailing = '\0';
    if (std::sscanf(
        line.c_str(), "%lu,%lf,%lf%c", &device_time_ms, &angle_deg, &velocity_deg_s,
        &trailing) != 3 || !std::isfinite(angle_deg) || angle_deg < 0.0 || angle_deg > 180.0 ||
      !std::isfinite(velocity_deg_s))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid servo CSV line (expected timestamp,angle,angular_velocity): '%s'",
        line.c_str());
      return;
    }
    (void)device_time_ms;

    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = {joint_name_};
    state.position = {
      angle_sign_ * (angle_deg - center_angle_deg_) *
      nodding_lidar::kPi / 180.0};
    state.velocity = {
      angle_sign_ * velocity_deg_s * nodding_lidar::kPi / 180.0};
    publisher_->publish(state);
  }

  void read_serial()
  {
    char bytes[128];
    while (true) {
      const ssize_t count = read(fd_, bytes, sizeof(bytes));
      if (count < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Serial read failed: %s", std::strerror(errno));
        }
        return;
      }
      if (count == 0) {
        return;
      }

      for (ssize_t i = 0; i < count; ++i) {
        const char c = bytes[i];
        if (c == '\n') {
          if (!discard_line_ && !line_.empty()) {
            parse_line(line_);
          }
          line_.clear();
          discard_line_ = false;
        } else if (!discard_line_ && c != '\r') {
          if (line_.size() < 127) {
            line_ += c;
          } else {
            line_.clear();
            discard_line_ = true;
          }
        }
      }
    }
  }

  int fd_{-1};
  bool discard_line_{false};
  double center_angle_deg_{90.0};
  double angle_sign_{1.0};
  std::string serial_port_;
  std::string state_topic_;
  std::string joint_name_;
  std::string line_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<SerialAngleBridge>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_angle_bridge"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
