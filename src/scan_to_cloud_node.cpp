// 点群ノード (TF2方式)。
//   /scan (LaserScan) と、首振りの動的TFから、FAST-LIO2に渡せる点群 (/points) を作る。
//
// 処理:
//   1. 1回転分のスキャンについて、スキャン内の (tf_samples_per_scan + 1) 点の時刻でTFを取得する
//   2. 各点の時刻に合わせて、そのTFの間を補間して(回転は球面補間、位置は線形補間)、座標を変換する
//   3. frames_per_cloud 回転分をためて、1つの点群にまとめ、各点に time と ring を付ける
//
// 制約 (必ず読むこと):
//   - 点ごとの時刻は、time_increment から求める。0の場合は scan_time / 点数 で代用する。
//     これは、実際の各点の測定時刻と一致するとは限らない。
//   - TFは指令値ベースの推定角度であり、実測値ではない。
//   - スキャンの時刻のTFがまだ届いていない場合は、tf_timeout まで待ち、超えたら捨てる
//     (最新のTFで代用はしない)。
//
// 動作確認はしていません。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nodding_lidar/geometry.hpp"

namespace
{

struct CloudPoint
{
  double t;  // 絶対時刻 [s]
  float x, y, z;
  float intensity;
  uint16_t ring;
};

struct PendingScan
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  double t0;         // スキャン開始時刻 [s]
  double scan_time;  // 1回転の時間 [s]
};

double to_sec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

tf2::TimePoint to_tf_time(double sec)
{
  return tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9)));
}

tf2::Transform to_tf2(const geometry_msgs::msg::TransformStamped & m)
{
  const auto & q = m.transform.rotation;
  const auto & t = m.transform.translation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
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
    output_frame_ = declare_parameter<std::string>("output_frame", "servo_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "");  // 空なら /scan の frame_id

    frames_per_cloud_ = declare_parameter<int>("frames_per_cloud", 4);
    tf_samples_ = declare_parameter<int>("tf_samples_per_scan", 8);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    tf_cache_sec_ = declare_parameter<double>("tf_cache_sec", 5.0);

    default_scan_time_ = declare_parameter<double>("default_scan_time", 1.0 / 15.0);
    stamp_is_scan_end_ = declare_parameter<bool>("stamp_is_scan_end", false);
    reverse_time_order_ = declare_parameter<bool>("reverse_time_order", false);
    max_frame_duration_ = declare_parameter<double>("max_frame_duration", 0.5);
    reliable_output_ = declare_parameter<bool>("reliable_output", true);

    frames_per_cloud_ = std::max(1, frames_per_cloud_);
    tf_samples_ = std::max(1, tf_samples_);

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
      "scan_to_cloud 開始: %s -> %s (出力フレーム=%s, %d回転/フレーム, TF補間=%d区間)",
      scan_topic_.c_str(), cloud_topic_.c_str(), output_frame_.c_str(),
      frames_per_cloud_, tf_samples_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    PendingScan item;
    item.scan = scan;
    item.scan_time = scan->scan_time >
      0.0 ? static_cast<double>(scan->scan_time) : default_scan_time_;
    const double stamp = to_sec(scan->header.stamp);
    item.t0 = stamp_is_scan_end_ ? stamp - item.scan_time : stamp;
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
      const double t_end = item.t0 + item.scan_time;

      std::string err;
      if (!tf_buffer_->canTransform(output_frame_, src, to_tf_time(t_end), &err)) {
        if (now().seconds() - t_end > tf_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "TFを取得できないため、スキャンを捨てました (%s <- %s, %.2f秒待ちました): %s",
            output_frame_.c_str(), src.c_str(), tf_timeout_, err.c_str());
          pending_.pop_front();
          continue;
        }
        break;  // まだ待つ
      }

      std::string err0;
      if (!tf_buffer_->canTransform(output_frame_, src, to_tf_time(item.t0), &err0)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "スキャン開始時刻のTFがありません (TFのbufferより古い可能性): %s", err0.c_str());
        pending_.pop_front();
        continue;
      }

      if (convert_scan(item, src)) {
        ++frames_in_buffer_;
        if (frames_in_buffer_ >= frames_per_cloud_) {
          publish_cloud();
        }
      }
      pending_.pop_front();
    }
  }

  // 1回転分のスキャンを、出力フレームの3D点にして buffer_ に積む
  bool convert_scan(const PendingScan & item, const std::string & src)
  {
    const sensor_msgs::msg::LaserScan & scan = *item.scan;

    // スキャン内の (K+1) 個の時刻でTFを取得する
    const int K = tf_samples_;
    std::vector<tf2::Transform> T;
    T.reserve(static_cast<size_t>(K) + 1);
    try {
      for (int k = 0; k <= K; ++k) {
        const double tk = item.t0 + item.scan_time * static_cast<double>(k) /
          static_cast<double>(K);
        T.push_back(to_tf2(tf_buffer_->lookupTransform(output_frame_, src, to_tf_time(tk))));
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TFの取得に失敗したため、スキャンを捨てました: %s", e.what());
      return false;
    }

    // 古い未完成のフレームは捨てる (スキャンが途切れた場合)
    if (!buffer_.empty() && item.t0 - buffer_t0_ > max_frame_duration_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "スキャンの間隔が空いたため、未完成のフレームを捨てました (%.2f秒)", item.t0 - buffer_t0_);
      buffer_.clear();
      frames_in_buffer_ = 0;
    }
    if (buffer_.empty()) {
      buffer_t0_ = item.t0;
    }

    const size_t n = scan.ranges.size();
    const bool has_intensity = scan.intensities.size() == n;

    for (size_t i = 0; i < n; ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        continue;
      }

      const double offset = nodding_lidar::point_time_offset(
        i, n, static_cast<double>(scan.time_increment), item.scan_time, reverse_time_order_);

      int k = 0;
      double frac = 0.0;
      nodding_lidar::sample_segment(offset, item.scan_time, K, k, frac);

      // 点の時刻に合わせて、前後のTFを補間する
      const tf2::Quaternion q = T[static_cast<size_t>(k)].getRotation().slerp(
        T[static_cast<size_t>(k) + 1].getRotation(), frac);
      const tf2::Vector3 o = T[static_cast<size_t>(k)].getOrigin().lerp(
        T[static_cast<size_t>(k) + 1].getOrigin(), frac);
      const tf2::Transform Ti(q, o);

      // LiDARのフレーム内の点 (センサの高さは、TFの平行移動に含まれる)
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const tf2::Vector3 p_out = Ti * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);

      CloudPoint p;
      p.t = item.t0 + offset;
      p.x = static_cast<float>(p_out.x());
      p.y = static_cast<float>(p_out.y());
      p.z = static_cast<float>(p_out.z());
      p.intensity = has_intensity ? scan.intensities[i] : 0.0f;
      p.ring = static_cast<uint16_t>(frames_in_buffer_);
      buffer_.push_back(p);
    }
    return true;
  }

  // buffer_ の点を、1つのPointCloud2にして送る
  void publish_cloud()
  {
    if (buffer_.empty()) {
      frames_in_buffer_ = 0;
      return;
    }

    std::sort(
      buffer_.begin(), buffer_.end(),
      [](const CloudPoint & a, const CloudPoint & b) {return a.t < b.t;});
    const double t_start = buffer_.front().t;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(t_start * 1e9), RCL_ROS_TIME);
    msg.header.frame_id = output_frame_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(buffer_.size());
    msg.is_bigendian = false;
    msg.is_dense = true;

    auto add_field = [&msg](const std::string & name, uint32_t offset, uint8_t type) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = type;
        f.count = 1;
        msg.fields.push_back(f);
      };
    add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
    add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
    add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
    add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    add_field("ring", 16, sensor_msgs::msg::PointField::UINT16);
    add_field("time", 20, sensor_msgs::msg::PointField::FLOAT32);
    msg.point_step = 24;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<size_t>(msg.row_step));

    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> it_r(msg, "ring");
    sensor_msgs::PointCloud2Iterator<float> it_t(msg, "time");

    for (const auto & p : buffer_) {
      *it_x = p.x;
      *it_y = p.y;
      *it_z = p.z;
      *it_i = p.intensity;
      *it_r = p.ring;
      *it_t = static_cast<float>(p.t - t_start);  // フレーム先頭からの経過秒
      ++it_x; ++it_y; ++it_z; ++it_i; ++it_r; ++it_t;
    }

    cloud_pub_->publish(msg);
    buffer_.clear();
    frames_in_buffer_ = 0;
  }

  // パラメータ
  std::string scan_topic_, cloud_topic_, output_frame_, lidar_frame_;
  int frames_per_cloud_{4};
  int tf_samples_{8};
  double tf_timeout_{0.1};
  double tf_cache_sec_{5.0};
  double default_scan_time_{1.0 / 15.0};
  bool stamp_is_scan_end_{false};
  bool reverse_time_order_{false};
  double max_frame_duration_{0.5};
  bool reliable_output_{true};

  // 状態
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PendingScan> pending_;
  std::vector<CloudPoint> buffer_;
  double buffer_t0_{0.0};
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
