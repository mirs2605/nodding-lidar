// 角度ノード。
//   - ±amplitude_deg の首振りの角度を生成し、PCA9685経由でサーボに指令する
//   - 推定した角度を sensor_msgs/JointState (/servo_state) で publish する
//   - servo_link -> lidar_link の動的TFを publish する
//
// 重要: エンコーダがないため、角度は「指令値ベースの推定」です。実際の角度ではありません。
//   - 推定角度 = angle_scale * 指令角度
//   - スタンプ = 指令時刻 + time_offset  (time_offset = サーボの遅れ τ [秒]、正の値で遅らせる)
//
// 動作確認はしていません。use_hardware:=false で、サーボなしでも動作を確認できます。

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nodding_lidar/geometry.hpp"
#include "nodding_lidar/pca9685.hpp"

class ServoNode : public rclcpp::Node
{
public:
  ServoNode()
  : Node("servo_node")
  {
    // --- 波形 ---
    amplitude_deg_ = declare_parameter<double>("amplitude_deg", 15.0);
    max_amplitude_deg_ = declare_parameter<double>("max_amplitude_deg", 30.0);
    period_sec_ = declare_parameter<double>("period_sec", 3.0);
    smoothness_ = declare_parameter<double>("smoothness", 0.3);
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);

    // --- 補正 ---
    angle_scale_ = declare_parameter<double>("angle_scale", 1.0);
    time_offset_ = declare_parameter<double>("time_offset", 0.0);

    // --- 出力 ---
    topic_ = declare_parameter<std::string>("topic", "/servo_state");
    joint_name_ = declare_parameter<std::string>("joint_name", "nodding_joint");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "servo_link");
    child_frame_ = declare_parameter<std::string>("child_frame", "lidar_link");
    lidar_height_ = declare_parameter<double>("lidar_height", 0.0325);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    // --- サーボ / PCA9685 ---
    use_hardware_ = declare_parameter<bool>("use_hardware", true);
    i2c_bus_ = declare_parameter<int>("i2c_bus", 1);
    i2c_address_ = declare_parameter<int>("i2c_address", 0x40);
    channel_ = declare_parameter<int>("channel", 0);
    pwm_freq_hz_ = declare_parameter<double>("pwm_freq_hz", 50.0);
    osc_hz_ = declare_parameter<double>("osc_hz", 25.0e6);
    servo_center_us_ = declare_parameter<double>("servo_center_us", 1500.0);
    servo_us_per_deg_ = declare_parameter<double>("servo_us_per_deg", 11.11);
    servo_sign_ = declare_parameter<double>("servo_sign", 1.0);
    min_pulse_us_ = declare_parameter<double>("min_pulse_us", 1000.0);
    max_pulse_us_ = declare_parameter<double>("max_pulse_us", 2000.0);

    if (amplitude_deg_ > max_amplitude_deg_) {
      RCLCPP_WARN(
        get_logger(), "amplitude_deg=%.1f が上限 max_amplitude_deg=%.1f を超えたため、制限します",
        amplitude_deg_, max_amplitude_deg_);
      amplitude_deg_ = max_amplitude_deg_;
    }
    if (update_rate_hz_ < 50.0) {
      RCLCPP_WARN(get_logger(), "update_rate_hz=%.1f は50Hz未満です。TFの補間精度が下がります", update_rate_hz_);
    }

    if (use_hardware_) {
      try {
        pca_ = std::make_unique<nodding_lidar::Pca9685>(
          i2c_bus_, i2c_address_, pwm_freq_hz_, osc_hz_);
        pca_->set_pulse_us(channel_, servo_center_us_);
      } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), "PCA9685の初期化に失敗しました: %s", e.what());
        throw;
      }
    } else {
      RCLCPP_WARN(get_logger(), "use_hardware=false: サーボは動かさず、角度の生成と publish だけを行います");
    }

    pub_ = create_publisher<sensor_msgs::msg::JointState>(topic_, rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    start_ = now();
    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ServoNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "servo_node 開始: 振幅±%.1f度, 周期%.2f秒, %s -> %s, TF=%s。"
      "角度はエンコーダなしの『指令値ベースの推定』です",
      amplitude_deg_, period_sec_, parent_frame_.c_str(), child_frame_.c_str(),
      publish_tf_ ? "on" : "off");
  }

  ~ServoNode() override
  {
    // 終了時は、サーボを中央に戻す (その位置を保持する)
    if (pca_) {
      try {
        pca_->set_pulse_us(channel_, servo_center_us_);
      } catch (const std::exception &) {
      }
    }
  }

private:
  void on_timer()
  {
    const rclcpp::Time t_cmd = now();
    const double t = (t_cmd - start_).seconds();

    // 指令角度 [rad]
    const double amp_rad = amplitude_deg_ * nodding_lidar::kPi / 180.0;
    const double cmd = amp_rad * nodding_lidar::nodding_wave(t, period_sec_, smoothness_);

    // サーボへ指令
    if (pca_) {
      try {
        const double us = nodding_lidar::angle_to_pulse_us(
          cmd, servo_center_us_, servo_us_per_deg_, servo_sign_, min_pulse_us_, max_pulse_us_);
        pca_->set_pulse_us(channel_, us);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "サーボへの指令に失敗しました: %s", e.what());
      }
    }

    // 推定した姿勢 (指令値ベース)
    const double est = angle_scale_ * cmd;
    const rclcpp::Time stamp = t_cmd + rclcpp::Duration::from_seconds(time_offset_);

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {joint_name_};
    js.position = {est};
    pub_->publish(js);

    if (tf_broadcaster_) {
      const nodding_lidar::Pose pose = nodding_lidar::servo_to_lidar_pose(est, lidar_height_);
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = parent_frame_;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = pose.t.x;
      tf.transform.translation.y = pose.t.y;
      tf.transform.translation.z = pose.t.z;
      tf.transform.rotation.x = pose.qx;
      tf.transform.rotation.y = pose.qy;
      tf.transform.rotation.z = pose.qz;
      tf.transform.rotation.w = pose.qw;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // パラメータ
  double amplitude_deg_, max_amplitude_deg_, period_sec_, smoothness_, update_rate_hz_;
  double angle_scale_, time_offset_;
  std::string topic_, joint_name_, parent_frame_, child_frame_;
  double lidar_height_;
  bool publish_tf_, use_hardware_;
  int i2c_bus_, i2c_address_, channel_;
  double pwm_freq_hz_, osc_hz_;
  double servo_center_us_, servo_us_per_deg_, servo_sign_, min_pulse_us_, max_pulse_us_;

  // 状態
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<nodding_lidar::Pca9685> pca_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<ServoNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_node"), "終了: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
