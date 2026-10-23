#include <chrono>
#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nodding_lidar/geometry.hpp"

class ServoTfNode : public rclcpp::Node
{
public:
  ServoTfNode()
  : Node("servo_tf_node"),
    last_state_time_(std::chrono::steady_clock::now())
  {
    state_topic_ = declare_parameter<std::string>("state_topic", "/servo_state");
    joint_name_ = declare_parameter<std::string>("joint_name", "nodding_joint");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "servo_link");
    child_frame_ = declare_parameter<std::string>("child_frame", "laser");
    lidar_height_ = declare_parameter<double>("lidar_height", 0.0325);
    state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 0.5);
    if (state_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("state_timeout_sec must be positive");
    }

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, rclcpp::QoS(10),
      std::bind(&ServoTfNode::on_state, this, std::placeholders::_1));
    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&ServoTfNode::check_state, this));
    RCLCPP_INFO(
      get_logger(), "Broadcasting %s -> %s from JointState '%s' (angle in radians)",
      parent_frame_.c_str(), child_frame_.c_str(), state_topic_.c_str());
  }

private:
  void on_state(const sensor_msgs::msg::JointState::ConstSharedPtr state)
  {
    const auto joint = std::find(state->name.begin(), state->name.end(), joint_name_);
    if (joint == state->name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "JointState does not contain joint '%s'",
        joint_name_.c_str());
      return;
    }
    const auto index = static_cast<size_t>(std::distance(state->name.begin(), joint));
    if (index >= state->position.size() || !std::isfinite(state->position[index])) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "JointState has no finite position for '%s'",
        joint_name_.c_str());
      return;
    }
    if (state->header.stamp.sec == 0 && state->header.stamp.nanosec == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring servo angle with a zero timestamp");
      return;
    }

    const nodding_lidar::Pose pose =
      nodding_lidar::servo_to_lidar_pose(state->position[index], lidar_height_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = state->header;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = pose.t.x;
    transform.transform.translation.y = pose.t.y;
    transform.transform.translation.z = pose.t.z;
    transform.transform.rotation.x = pose.qx;
    transform.transform.rotation.y = pose.qy;
    transform.transform.rotation.z = pose.qz;
    transform.transform.rotation.w = pose.qw;
    broadcaster_->sendTransform(transform);

    last_state_time_ = std::chrono::steady_clock::now();
    have_state_ = true;
  }

  void check_state()
  {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_state_time_).count();
    if (!have_state_ || elapsed > state_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No valid servo angle received recently; dynamic TF updates have stopped");
    }
  }

  std::string state_topic_;
  std::string joint_name_;
  std::string parent_frame_;
  std::string child_frame_;
  double lidar_height_{0.0325};
  double state_timeout_sec_{0.5};
  bool have_state_{false};
  std::chrono::steady_clock::time_point last_state_time_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<ServoTfNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_tf_node"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
