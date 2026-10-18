#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/time.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

struct PendingScan
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  double t0;         // スキャン開始時刻 [s]
  double duration;   // 推定したスキャン時間 [s]
  std::chrono::steady_clock::time_point received;
};

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

bool same_layout(
  const sensor_msgs::msg::PointCloud2 & lhs,
  const sensor_msgs::msg::PointCloud2 & rhs)
{
  if (lhs.point_step != rhs.point_step || lhs.fields.size() != rhs.fields.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.fields.size(); ++i) {
    const auto & a = lhs.fields[i];
    const auto & b = rhs.fields[i];
    if (a.name != b.name || a.offset != b.offset || a.datatype != b.datatype ||
      a.count != b.count)
    {
      return false;
    }
  }
  return true;
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

    frames_per_cloud_ = declare_parameter<int>("frames_per_cloud", 15);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    tf_cache_sec_ = declare_parameter<double>("tf_cache_sec", 5.0);

    default_scan_time_ = declare_parameter<double>("default_scan_time", 1.0 / 15.0);
    stamp_is_scan_end_ = declare_parameter<bool>("stamp_is_scan_end", false);
    reverse_time_order_ = declare_parameter<bool>("reverse_time_order", false);
    max_frame_duration_ = declare_parameter<double>("max_frame_duration", 2.0);
    reliable_output_ = declare_parameter<bool>("reliable_output", true);

    frames_per_cloud_ = std::max(1, frames_per_cloud_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      get_clock(), tf2::durationFromSec(tf_cache_sec_));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanToCloud::on_scan, this, std::placeholders::_1));

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
      "scan_to_cloud started: %s -> %s (frame=%s, %d scans/cloud, laser_geometry)",
      scan_topic_.c_str(), cloud_topic_.c_str(), output_frame_.c_str(),
      frames_per_cloud_);
  }

private:
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
    pending_.push_back(item);
    process_pending();
  }

  // TFが届いたスキャンから順に変換する
  void process_pending()
  {
    while (!pending_.empty()) {
      const PendingScan & item = pending_.front();
      const std::string src =
        lidar_frame_.empty() ? item.scan->header.frame_id : lidar_frame_;
      const double t_end = item.t0 + item.duration;

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

  // laser_geometry interpolates the TF over the acquisition interval.
  bool convert_scan(const PendingScan & item, const std::string & src)
  {
    sensor_msgs::msg::LaserScan scan = *item.scan;
    scan.header.frame_id = src;
    scan.header.stamp = to_msg_time(item.t0);
    if (!std::isfinite(scan.time_increment) || scan.time_increment <= 0.0F) {
      scan.time_increment = static_cast<float>(
        item.duration / static_cast<double>(scan.ranges.size()));
    }
    if (reverse_time_order_) {
      std::reverse(scan.ranges.begin(), scan.ranges.end());
      std::reverse(scan.intensities.begin(), scan.intensities.end());
      scan.angle_min += static_cast<float>(scan.ranges.size() - 1) * scan.angle_increment;
      scan.angle_max = scan.angle_min + static_cast<float>(scan.ranges.size() - 1) *
        (-scan.angle_increment);
      scan.angle_increment = -scan.angle_increment;
    }

    // laser_geometry deskews each beam into a fixed frame. Then move the
    // result into the robot frame at the beginning of this accumulated cloud,
    // so scans collected while the base moves still form one rigid cloud.
    if (frames_in_buffer_ > 0 && item.t0 - last_scan_t0_ > max_frame_duration_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping incomplete cloud after %.2f s scan gap", item.t0 - last_scan_t0_);
      frames_in_buffer_ = 0;
      accumulated_cloud_ = sensor_msgs::msg::PointCloud2();
    }

    if (frames_in_buffer_ == 0) {
      reference_t0_ = item.t0;
      try {
        reference_transform_ = tf_buffer_->lookupTransform(
          output_frame_, fixed_frame_,
          rclcpp::Time(to_msg_time(reference_t0_), get_clock()->get_clock_type()));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping scan: cannot get %s <- %s at reference time: %s",
          output_frame_.c_str(), fixed_frame_.c_str(), e.what());
        return false;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    try {
      projector_.transformLaserScanToPointCloud(
        fixed_frame_, scan, cloud, *tf_buffer_, -1.0,
        laser_geometry::channel_option::Default);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropping scan: laser_geometry TF lookup failed: %s",
        e.what());
      return false;
    }

    if (cloud.width == 0 || cloud.data.empty()) {
      return true;
    }

    if (frames_in_buffer_ > 0 && !same_layout(accumulated_cloud_, cloud)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping incomplete cloud because PointCloud2 layouts changed");
      frames_in_buffer_ = 0;
      accumulated_cloud_ = sensor_msgs::msg::PointCloud2();
      reference_t0_ = item.t0;
      try {
        reference_transform_ = tf_buffer_->lookupTransform(
          output_frame_, fixed_frame_,
          rclcpp::Time(to_msg_time(reference_t0_), get_clock()->get_clock_type()));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping scan after cloud layout change: cannot get reference transform: %s",
          e.what());
        return false;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_in_reference;
    tf2::doTransform(cloud, cloud_in_reference, reference_transform_);
    cloud = std::move(cloud_in_reference);

    if (frames_in_buffer_ == 0) {
      accumulated_cloud_ = cloud;
    } else {
      accumulated_cloud_.data.insert(
        accumulated_cloud_.data.end(), cloud.data.begin(), cloud.data.end());
      accumulated_cloud_.width += cloud.width;
      accumulated_cloud_.row_step = accumulated_cloud_.width * accumulated_cloud_.point_step;
      accumulated_cloud_.is_dense = accumulated_cloud_.is_dense && cloud.is_dense;
    }
    last_scan_t0_ = item.t0;
    ++frames_in_buffer_;
    if (frames_in_buffer_ >= frames_per_cloud_) {
      cloud_pub_->publish(accumulated_cloud_);
      frames_in_buffer_ = 0;
      accumulated_cloud_ = sensor_msgs::msg::PointCloud2();
    }
    return true;
  }

  // パラメータ
  std::string scan_topic_, cloud_topic_, output_frame_, fixed_frame_, lidar_frame_;
  int frames_per_cloud_{15};
  double tf_timeout_{0.1};
  double tf_cache_sec_{5.0};
  double default_scan_time_{1.0 / 15.0};
  bool stamp_is_scan_end_{false};
  bool reverse_time_order_{false};
  double max_frame_duration_{2.0};
  bool reliable_output_{true};

  // 状態
  laser_geometry::LaserProjection projector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PendingScan> pending_;
  sensor_msgs::msg::PointCloud2 accumulated_cloud_;
  geometry_msgs::msg::TransformStamped reference_transform_;
  double reference_t0_{0.0};
  double last_scan_t0_{0.0};
  int frames_in_buffer_{0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
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
