#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <iterator>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nodding_lidar/geometry.hpp"

namespace
{

struct PendingScan
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  double t0;         // スキャン開始時刻 [s]
  double duration;   // 推定したスキャン時間 [s]
  std::chrono::steady_clock::time_point received;
};

struct ServoSample
{
  double stamp{0.0};
  double position{0.0};
  double velocity{0.0};
};

struct ProjectedPoint
{
  nodding_lidar::Vec3 position;
  float intensity{0.0F};
};

bool interpolate_servo_angle(
  const std::deque<ServoSample> & samples, double stamp, double & position)
{
  if (samples.empty() || stamp < samples.front().stamp || stamp > samples.back().stamp) {
    return false;
  }
  if (stamp == samples.back().stamp) {
    position = samples.back().position;
    return true;
  }

  const auto right = std::upper_bound(
    samples.begin(), samples.end(), stamp,
    [](double t, const ServoSample & sample) {return t < sample.stamp;});
  if (right == samples.begin() || right == samples.end()) {
    return false;
  }
  const ServoSample & b = *right;
  const ServoSample & a = *std::prev(right);
  const double duration = b.stamp - a.stamp;
  if (duration <= 0.0) {
    return false;
  }
  const double u = (stamp - a.stamp) / duration;
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
  const double h10 = u3 - 2.0 * u2 + u;
  const double h01 = -2.0 * u3 + 3.0 * u2;
  const double h11 = u3 - u2;
  position = h00 * a.position + h10 * duration * a.velocity +
    h01 * b.position + h11 * duration * b.velocity;
  return std::isfinite(position);
}

tf2::Vector3 apply_transform(
  const geometry_msgs::msg::TransformStamped & transform, const nodding_lidar::Vec3 & point)
{
  const auto & t = transform.transform.translation;
  const auto & r = transform.transform.rotation;
  const tf2::Transform tf(
    tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(t.x, t.y, t.z));
  return tf * tf2::Vector3(point.x, point.y, point.z);
}

double to_sec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time to_msg_time(double sec)
{
  const int64_t nanoseconds = static_cast<int64_t>(sec * 1e9);
  int64_t seconds = nanoseconds / 1000000000LL;
  int64_t remainder = nanoseconds % 1000000000LL;
  if (remainder < 0) {
    --seconds;
    remainder += 1000000000LL;
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>(remainder);
  return stamp;
}

tf2::TimePoint to_tf_time(double sec)
{
  return tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9)));
}

}  // namespace

class ScanToCloud : public rclcpp::Node
{
public:
  ScanToCloud()
  : Node("scan_to_cloud")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/points");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "");  // 空なら /scan の frame_id
    servo_state_topic_ = declare_parameter<std::string>("servo_state_topic", "/servo_state");
    servo_parent_frame_ = declare_parameter<std::string>("servo_parent_frame", "servo_link");
    lidar_height_ = declare_parameter<double>("lidar_height", 0.0325);

    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    tf_cache_sec_ = declare_parameter<double>("tf_cache_sec", 5.0);

    default_scan_time_ = declare_parameter<double>("default_scan_time", 0.1);
    stamp_is_scan_end_ = declare_parameter<bool>("stamp_is_scan_end", false);
    reverse_time_order_ = declare_parameter<bool>("reverse_time_order", false);
    reliable_output_ = declare_parameter<bool>("reliable_output", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      get_clock(), tf2::durationFromSec(tf_cache_sec_));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanToCloud::on_scan, this, std::placeholders::_1));
    servo_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      servo_state_topic_, rclcpp::QoS(100),
      std::bind(&ScanToCloud::on_servo_state, this, std::placeholders::_1));

    rclcpp::QoS pub_qos(5);
    if (!reliable_output_) {
      pub_qos.best_effort();
    }
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, pub_qos);

    // TFの到着待ちのスキャンを、定期的に処理する
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&ScanToCloud::process_pending, this));

    RCLCPP_INFO(
      get_logger(),
      "scan_to_cloud started: %s -> %s (frame=%s, one cloud/scan, Hermite servo interpolation)",
      scan_topic_.c_str(), cloud_topic_.c_str(), output_frame_.c_str());
  }

private:
  void on_servo_state(const sensor_msgs::msg::JointState::ConstSharedPtr state)
  {
    if ((state->header.stamp.sec == 0 && state->header.stamp.nanosec == 0) ||
      state->position.empty() || state->velocity.empty() ||
      !std::isfinite(state->position[0]) || !std::isfinite(state->velocity[0]))
    {
      return;
    }
    const ServoSample sample{
      to_sec(state->header.stamp), state->position[0], state->velocity[0]};
    if (!servo_samples_.empty() && sample.stamp <= servo_samples_.back().stamp) {
      return;
    }
    servo_samples_.push_back(sample);
    while (servo_samples_.size() > 2000) {
      servo_samples_.pop_front();
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    PendingScan item;
    item.scan = scan;
    item.duration = std::isfinite(scan->scan_time) && scan->scan_time > 0.0 ?
      static_cast<double>(scan->scan_time) : default_scan_time_;
    if (std::isfinite(scan->time_increment) && scan->time_increment > 0.0F) {
      item.duration = std::max(
        item.duration,
        static_cast<double>(scan->time_increment) * static_cast<double>(scan->ranges.size()));
    }
    const double stamp = to_sec(scan->header.stamp);
    item.t0 = stamp_is_scan_end_ ? stamp - item.duration : stamp;
    item.received = std::chrono::steady_clock::now();
    if (scan->ranges.empty() || scan->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring LaserScan with no ranges or frame_id");
      return;
    }
    if (!lidar_frame_.empty() && scan->header.frame_id != lidar_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring LaserScan in frame '%s'; expected lidar frame '%s'",
        scan->header.frame_id.c_str(), lidar_frame_.c_str());
      return;
    }
    pending_.push_back(item);
    process_pending();
  }

  // TFが届いたスキャンから順に変換する
  void process_pending()
  {
    while (!pending_.empty()) {
      const PendingScan & item = pending_.front();
      const std::string src = servo_parent_frame_;
      const double t_end = item.t0 + item.duration;

      const double t_last_beam = item.t0 +
        (item.scan->ranges.size() > 1 ?
        static_cast<double>(item.scan->ranges.size() - 1) *
        (item.scan->time_increment > 0.0F ? item.scan->time_increment :
        item.duration / static_cast<double>(item.scan->ranges.size())) : 0.0);
      double ignored_angle = 0.0;
      if (!interpolate_servo_angle(servo_samples_, item.t0, ignored_angle) ||
        !interpolate_servo_angle(servo_samples_, t_last_beam, ignored_angle))
      {
        const double waited = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - item.received).count();
        if (waited > tf_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping scan: servo angle/velocity samples do not cover scan interval");
          pending_.pop_front();
          continue;
        }
        break;
      }

      std::string err;
      if (!tf_buffer_->canTransform(fixed_frame_, src, to_tf_time(t_end), &err)) {
        const double waited = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - item.received).count();
        if (waited > tf_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping scan: TF unavailable (%s <- %s) after %.2f s: %s",
            fixed_frame_.c_str(), src.c_str(), waited, err.c_str());
          pending_.pop_front();
          continue;
        }
        break;
      }

      std::string err0;
      if (!tf_buffer_->canTransform(fixed_frame_, src, to_tf_time(item.t0), &err0)) {
        const double waited = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - item.received).count();
        if (waited > tf_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping scan: start-time TF unavailable (%s <- %s): %s",
            fixed_frame_.c_str(), src.c_str(), err0.c_str());
          pending_.pop_front();
          continue;
        }
        break;
      }

      convert_scan(item, src);
      pending_.pop_front();
    }
  }

  // Reconstruct each beam pose from angle/velocity samples, then apply the
  // robot pose at that beam's acquisition time.
  bool convert_scan(const PendingScan & item, const std::string & src)
  {
    sensor_msgs::msg::LaserScan scan = *item.scan;
    double time_increment = scan.time_increment;
    if (!std::isfinite(time_increment) || time_increment <= 0.0) {
      time_increment = item.duration / static_cast<double>(scan.ranges.size());
    }
    if (reverse_time_order_) {
      std::reverse(scan.ranges.begin(), scan.ranges.end());
      std::reverse(scan.intensities.begin(), scan.intensities.end());
      scan.angle_min += static_cast<float>(scan.ranges.size() - 1) * scan.angle_increment;
      scan.angle_max = scan.angle_min + static_cast<float>(scan.ranges.size() - 1) *
        (-scan.angle_increment);
      scan.angle_increment = -scan.angle_increment;
    }

    // Express all beams in the robot frame at this scan's start time.
    geometry_msgs::msg::TransformStamped reference_transform;
    try {
      reference_transform = tf_buffer_->lookupTransform(
        output_frame_, fixed_frame_,
        rclcpp::Time(to_msg_time(item.t0), get_clock()->get_clock_type()));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping scan: cannot get %s <- %s at reference time: %s",
        output_frame_.c_str(), fixed_frame_.c_str(), e.what());
      return false;
    }

    const bool has_intensity = scan.intensities.size() == scan.ranges.size();
    std::vector<ProjectedPoint> points;
    points.reserve(scan.ranges.size());
    const rclcpp::Clock::SharedPtr clock = get_clock();
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }
      const double beam_time = item.t0 + static_cast<double>(i) * time_increment;
      double angle = 0.0;
      if (!interpolate_servo_angle(servo_samples_, beam_time, angle)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping scan: no servo angle interpolation at beam time");
        return false;
      }

      geometry_msgs::msg::TransformStamped fixed_from_servo;
      try {
        fixed_from_servo = tf_buffer_->lookupTransform(
          fixed_frame_, src,
          rclcpp::Time(to_msg_time(beam_time), clock->get_clock_type()));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping scan: cannot get %s <- %s at beam time: %s",
          fixed_frame_.c_str(), src.c_str(), e.what());
        return false;
      }

      const double scan_angle = scan.angle_min +
        static_cast<double>(i) * scan.angle_increment;
      const nodding_lidar::Vec3 point_lidar{
        range * std::cos(scan_angle), range * std::sin(scan_angle), 0.0};
      const nodding_lidar::Pose lidar_pose =
        nodding_lidar::servo_to_lidar_pose(angle, lidar_height_);
      const nodding_lidar::Vec3 point_servo =
        nodding_lidar::apply_pose(lidar_pose, point_lidar);
      const tf2::Vector3 point_fixed = apply_transform(fixed_from_servo, point_servo);
      points.push_back({
        {point_fixed.x(), point_fixed.y(), point_fixed.z()},
        has_intensity ? scan.intensities[i] : 0.0F});
    }

    if (points.empty()) {
      return true;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = fixed_frame_;
    cloud.header.stamp = to_msg_time(item.t0);
    sensor_msgs::msg::PointCloud2Modifier modifier(cloud);
    if (has_intensity) {
      modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    } else {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(points.size());
    sensor_msgs::msg::PointCloud2Iterator<float> x_iter(cloud, "x");
    sensor_msgs::msg::PointCloud2Iterator<float> y_iter(cloud, "y");
    sensor_msgs::msg::PointCloud2Iterator<float> z_iter(cloud, "z");
    std::unique_ptr<sensor_msgs::msg::PointCloud2Iterator<float>> intensity_iter;
    if (has_intensity) {
      intensity_iter = std::make_unique<sensor_msgs::msg::PointCloud2Iterator<float>>(
        cloud, "intensity");
    }
    for (const auto & point : points) {
      *x_iter = static_cast<float>(point.position.x);
      *y_iter = static_cast<float>(point.position.y);
      *z_iter = static_cast<float>(point.position.z);
      ++x_iter;
      ++y_iter;
      ++z_iter;
      if (intensity_iter) {
        **intensity_iter = point.intensity;
        ++(*intensity_iter);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_in_reference;
    tf2::doTransform(cloud, cloud_in_reference, reference_transform);
    cloud_pub_->publish(cloud_in_reference);
    return true;
  }

  // パラメータ
  std::string scan_topic_, cloud_topic_, output_frame_, fixed_frame_, lidar_frame_;
  std::string servo_state_topic_, servo_parent_frame_;
  double tf_timeout_{0.1};
  double tf_cache_sec_{5.0};
  double default_scan_time_{0.1};
  double lidar_height_{0.0325};
  bool stamp_is_scan_end_{false};
  bool reverse_time_order_{false};
  bool reliable_output_{true};

  // 状態
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PendingScan> pending_;
  std::deque<ServoSample> servo_samples_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToCloud>());
  rclcpp::shutdown();
  return 0;
}
