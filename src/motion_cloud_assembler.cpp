#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nodding_lidar/motion_window.hpp"

namespace nodding_lidar
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
}

tf2::Transform pose(const geometry_msgs::msg::TransformStamped & message)
{
  const auto & t = message.transform.translation;
  const auto & q = message.transform.rotation;
  tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
  if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
    !std::isfinite(rotation.length2()) || rotation.length2() < 1e-12)
  {
    throw std::runtime_error("Non-finite or zero-quaternion TF");
  }
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(t.x, t.y, t.z));
}
}  // namespace

/** @brief 計測時刻のTFで点群を重ね、旋回量に応じて古い点を除いて照合用点群を出す。
 *
 * cloudはスキャン内補償済みのbase_link点群、odometryは車輪＋IMUによる移動推定。
 * 出力は最新スキャンの基準時刻のbase_link座標で、x・y・z [m] のみを含む。
 * 機器への信号と速度指令は出さない。欠測は診断し、時刻の異なるTFで補完しない。
 */
class MotionCloudAssembler : public rclcpp::Node
{
public:
  /** @brief topic・frame・保持上限を読み込み、点群・オドメトリの購読を開始する。
   *
   * 不正な設定値は例外で起動を中断する。診断は1 Hz、TF待ちの確認は100 Hz。
   */
  MotionCloudAssembler()
  : Node("motion_cloud_assembler")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    const double window = positive("max_window_s", 2.0);
    const double angle = positive("max_turn_span_rad", 0.2);
    max_age_ = positive("max_input_age_s", 1.0);
    wait_s_ = positive("tf_wait_s", 0.3);
    const double gap = positive("max_motion_gap_s", 0.15);
    period_ = 1.0 / positive("max_output_rate_hz", 2.0);
    voxel_ = declare_parameter<double>("voxel_size_m", 0.1);
    if (!std::isfinite(voxel_) || voxel_ < 0.0 || (voxel_ > 0.0 && voxel_ < 1e-6)) {
      throw std::invalid_argument("voxel_size_m must be zero or >= 0.000001 m");
    }
    max_scans_ = count("max_scans", 32);
    max_points_ = count("max_points", 200000);
    min_points_ = count("min_points", 100);
    max_pending_ = count("max_pending_scans", 8);
    if (min_points_ > max_points_ || base_frame_.empty() || fixed_frame_.empty() ||
      base_frame_ == fixed_frame_)
    {
      throw std::invalid_argument("Invalid point limits or frame names");
    }
    history_ = std::make_unique<MotionHistory>(window + max_age_ + wait_s_ + gap, gap);
    window_ = std::make_unique<MotionWindow>(window, angle, max_scans_, max_points_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("matching_cloud", 2);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 2);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {on_cloud(message);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odometry(*message);});
    process_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {process();});
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostic();});
  }

private:
  struct Pending
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point received;
  };
  struct Centroid {double x{0}, y{0}, z{0}; std::size_t n{0};};

  double positive(const std::string & name, double value)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0.0) {
      throw std::invalid_argument(name + " must be positive and finite");
    }
    return result;
  }

  std::size_t count(const std::string & name, int64_t value)
  {
    const auto result = declare_parameter<int64_t>(name, value);
    if (result <= 0) {throw std::invalid_argument(name + " must be positive");}
    return static_cast<std::size_t>(result);
  }

  void check_clock()
  {
    const auto now = get_clock()->now().nanoseconds();
    if (last_clock_ && now < *last_clock_) {
      pending_.clear();
      history_->clear();
      window_->clear();
      last_input_.reset();
      last_output_.reset();
      turn_span_.reset();
      motion_available_ = false;
      window_s_ = 0.0;
      output_points_ = 0;
      state_ = "clock_reset";
    }
    last_clock_ = now;
  }

  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    check_clock();
    const auto & v = message.twist.twist.angular;
    const double age = get_clock()->now().seconds() - seconds(message.header.stamp);
    if (message.header.frame_id != fixed_frame_ || message.child_frame_id.empty() ||
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
      age > max_age_ || age < -wait_s_)
    {
      ++invalid_odometry_;
      return;
    }
    try {
      tf2::Vector3 angular(v.x, v.y, v.z);
      if (message.child_frame_id != base_frame_) {
        const auto t = tf_buffer_->lookupTransform(
          base_frame_, message.child_frame_id,
          rclcpp::Time(message.header.stamp, get_clock()->get_clock_type()));
        angular = pose(t).getBasis() * angular;
      }
      if (!history_->add(seconds(message.header.stamp), angular.z())) {
        ++invalid_odometry_;
      }
    } catch (const std::exception &) {
      ++invalid_odometry_;
    }
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    check_clock();
    const double stamp = seconds(message->header.stamp);
    const double age = get_clock()->now().seconds() - stamp;
    const auto points = static_cast<std::size_t>(message->width) * message->height;
    if (message->header.frame_id != base_frame_ || (last_input_ && stamp <= *last_input_) ||
      points == 0 || points > max_points_ || age > max_age_ || age < -wait_s_)
    {
      ++dropped_;
      if (message->header.frame_id != base_frame_) {
        state_ = "invalid_cloud_frame";
      } else if (last_input_ && stamp <= *last_input_) {
        state_ = "non_increasing_cloud_stamp";
      } else if (points == 0 || points > max_points_) {
        state_ = "invalid_input_point_count";
      } else {
        state_ = "stale_or_future_cloud";
      }
      return;
    }
    last_input_ = stamp;
    pending_.push_back({message, std::chrono::steady_clock::now()});
    while (pending_.size() > max_pending_) {
      pending_.pop_front();
      ++dropped_;
    }
  }

  void process()
  {
    check_clock();
    while (!pending_.empty()) {
      const auto & pending = pending_.front();
      const auto & cloud = *pending.cloud;
      const double stamp = seconds(cloud.header.stamp);
      const double age = get_clock()->now().seconds() - stamp;
      const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - pending.received).count();
      if (age > max_age_ || age < -wait_s_) {
        state_ = "stale_or_future_cloud";
        ++dropped_;
        pending_.pop_front();
        continue;
      }
      const auto started = std::chrono::steady_clock::now();
      try {
        // 時刻0はTFで「最新」の意味になるため、計測時刻として使用しない。
        if (stamp <= 0.0) {throw std::runtime_error("Zero measurement stamp");}
        const auto transform = pose(tf_buffer_->lookupTransform(
            fixed_frame_, base_frame_,
            rclcpp::Time(cloud.header.stamp, get_clock()->get_clock_type())));
        // オドメトリの後続サンプルがまだ届かない場合だけ、有限時間待つ。
        if (!history_->span(stamp, stamp) && waited < wait_s_) {break;}
        accept(cloud, transform);
        processing_wall_s_ = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      } catch (const tf2::TransformException &) {
        if (waited < wait_s_) {break;}
        ++dropped_;
        state_ = "tf_unavailable_at_stamp";
      } catch (const std::exception &) {
        ++dropped_;
        state_ = "invalid_cloud_or_transform";
      }
      pending_.pop_front();
    }
  }

  void accept(const sensor_msgs::msg::PointCloud2 & cloud, const tf2::Transform & transform)
  {
    const std::size_t n = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (n == 0 || n > max_points_ || cloud.point_step == 0 ||
      cloud.row_step != static_cast<std::size_t>(cloud.width) * cloud.point_step ||
      cloud.data.size() != static_cast<std::size_t>(cloud.row_step) * cloud.height ||
      cloud.is_bigendian)
    {
      throw std::runtime_error("Cloud must have bounded, packed, little-endian points");
    }
    for (const auto * name : {"x", "y", "z"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [name](const auto & f) {return f.name == name;});
      if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        field->count != 1 || static_cast<std::size_t>(field->offset) + sizeof(float) >
        cloud.point_step)
      {
        throw std::runtime_error("Cloud requires FLOAT32 x, y, z fields");
      }
    }
    MotionScan scan{seconds(cloud.header.stamp), {}};
    scan.points.reserve(n);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (std::size_t i = 0; i < n; ++i, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
      const auto p = transform * tf2::Vector3(*x, *y, *z);
      MotionPoint point{static_cast<float>(p.x()), static_cast<float>(p.y()),
        static_cast<float>(p.z())};
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        scan.points.push_back(point);
      }
    }
    if (!window_->add(std::move(scan), *history_)) {
      throw std::runtime_error("Empty or unordered point cloud");
    }
    const auto & scans = window_->scans();
    window_s_ = scans.back().stamp - scans.front().stamp;
    turn_span_ = history_->span(scans.front().stamp, scans.back().stamp);
    motion_available_ = history_->span(scans.back().stamp, scans.back().stamp).has_value();
    state_ = motion_available_ ? "ready" : "single_scan_without_motion_history";
    if (last_output_ && scans.back().stamp - *last_output_ < period_ - 1e-9) {return;}

    std::vector<MotionPoint> points;
    if (voxel_ > 0.0) {
      std::map<std::tuple<double, double, double>, Centroid> voxels;
      for (const auto & s : scans) {
        for (const auto & p : s.points) {
          auto & cell = voxels[{std::floor(p.x / voxel_), std::floor(p.y / voxel_),
            std::floor(p.z / voxel_)}];
          cell.x += p.x; cell.y += p.y; cell.z += p.z; ++cell.n;
        }
      }
      points.reserve(voxels.size());
      for (const auto & entry : voxels) {
        const auto & c = entry.second;
        points.push_back({static_cast<float>(c.x / c.n), static_cast<float>(c.y / c.n),
          static_cast<float>(c.z / c.n)});
      }
    } else {
      points.reserve(window_->point_count());
      for (const auto & s : scans) {
        points.insert(points.end(), s.points.begin(), s.points.end());
      }
    }
    output_points_ = points.size();
    if (points.size() < min_points_) {
      state_ = "insufficient_points";
      return;
    }
    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud.header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> ox(output, "x"), oy(output, "y"), oz(output, "z");
    const auto inverse = transform.inverse();
    for (const auto & p : points) {
      const auto v = inverse * tf2::Vector3(p.x, p.y, p.z);
      *ox = static_cast<float>(v.x()); *oy = static_cast<float>(v.y());
      *oz = static_cast<float>(v.z()); ++ox; ++oy; ++oz;
    }
    output.is_dense = true;
    cloud_pub_->publish(output);
    last_output_ = seconds(output.header.stamp);
    latency_s_ = get_clock()->now().seconds() - *last_output_;
  }

  void publish_diagnostic()
  {
    check_clock();
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_name();
    status.hardware_id = "nodding_lidar";
    const double now = get_clock()->now().seconds();
    const bool stale = !last_output_ || now - *last_output_ > max_age_ + period_;
    status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (state_ == "ready" ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = stale ? "matching_cloud_not_updated: " + state_ : state_;
    const auto value = [&status](const std::string & key, const auto & v) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key; item.value = std::to_string(v); status.values.push_back(item);
      };
    value("window_s", window_s_);
    value("turn_span_rad", turn_span_.value_or(-1.0));
    value("motion_available", motion_available_);
    value("retained_scans", window_->scans().size());
    value("retained_points", window_->point_count());
    value("output_points", output_points_);
    value("pending_scans", pending_.size());
    value("dropped_scans", dropped_);
    value("invalid_odometry", invalid_odometry_);
    value("last_output_age_s", last_output_ ? now - *last_output_ : -1.0);
    value("last_output_latency_s", latency_s_);
    value("processing_wall_s", processing_wall_s_);
    output.status.push_back(status);
    diagnostic_pub_->publish(output);
  }

  std::string base_frame_, fixed_frame_, state_{"waiting_for_input"};
  double max_age_, wait_s_, period_, voxel_, window_s_{0}, latency_s_{0}, processing_wall_s_{0};
  std::size_t max_scans_, max_points_, min_points_, max_pending_, output_points_{0};
  std::size_t dropped_{0}, invalid_odometry_{0};
  bool motion_available_{false};
  std::optional<double> last_input_, last_output_, turn_span_;
  std::optional<int64_t> last_clock_;
  std::unique_ptr<MotionHistory> history_;
  std::unique_ptr<MotionWindow> window_;
  std::deque<Pending> pending_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr process_timer_, diagnostic_timer_;
};
}  // namespace nodding_lidar

/** @brief 点群蓄積ノードを実行し、設定・実行エラー時は非ゼロで終了する。
 * @param argc ROS引数の個数。
 * @param argv ROS引数列。
 * @return 正常終了時0、例外発生時1。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<nodding_lidar::MotionCloudAssembler>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("motion_cloud_assembler"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
