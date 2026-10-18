// ROSに依存しない幾何計算。角度ノードと点群ノードの両方から使い、単体テストもできる。
//
// 座標系の約束 (servo_link):
//   X: 前方, Y: 左 (= 回転軸), Z: 上
//   角度 phi は Y軸まわりの右手系回転 [rad]。正の角度で、前方 (+X) が下がる向き。
//   R_y(phi) = [[ c, 0, s],
//               [ 0, 1, 0],
//               [-s, 0, c]]
//   LiDARのセンサ原点は、回転軸から lidar_height だけ上 (回転する側の +Z) にある。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace nodding_lidar
{

constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose
{
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  Vec3 t;
};

// Y軸まわりに phi [rad] 回転
inline Vec3 rotate_y(double phi, const Vec3 & v)
{
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
}

// LiDAR平面内の点 (距離 range, スキャン角 scan_angle) を servo_link の座標に変換する。
// センサは回転軸から lidar_height だけ上にあるので、平面内の点の高さを lidar_height として回転させる。
inline Vec3 project_point(double range, double scan_angle, double phi, double lidar_height)
{
  const Vec3 p{range * std::cos(scan_angle), range * std::sin(scan_angle), lidar_height};
  return rotate_y(phi, p);
}

// servo_link -> lidar_link の姿勢。
//   回転: R_y(phi)  (クォータニオン (0, sin(phi/2), 0, cos(phi/2)))
//   平行移動: R_y(phi) * (0, 0, lidar_height)  (LiDARの位置も回転軸のまわりを動く)
inline Pose servo_to_lidar_pose(double phi, double lidar_height)
{
  Pose p;
  p.qx = 0.0;
  p.qy = std::sin(phi / 2.0);
  p.qz = 0.0;
  p.qw = std::cos(phi / 2.0);
  p.t = rotate_y(phi, Vec3{0.0, 0.0, lidar_height});
  return p;
}

// 姿勢を点に適用: v' = R(q) * v + t
inline Vec3 apply_pose(const Pose & p, const Vec3 & v)
{
  const double tx = 2.0 * (p.qy * v.z - p.qz * v.y);
  const double ty = 2.0 * (p.qz * v.x - p.qx * v.z);
  const double tz = 2.0 * (p.qx * v.y - p.qy * v.x);
  Vec3 r;
  r.x = v.x + p.qw * tx + (p.qy * tz - p.qz * ty) + p.t.x;
  r.y = v.y + p.qw * ty + (p.qz * tx - p.qx * tz) + p.t.y;
  r.z = v.z + p.qw * tz + (p.qx * ty - p.qy * tx) + p.t.z;
  return r;
}

// 首振りの波形。-1 〜 +1 を返す。t=0 で0から始まり、上向き(正)に進む。
//   smoothness = 0: 三角波 (一定の速さで往復)
//   smoothness = 1: 正弦波 (折り返しがなめらか)
inline double nodding_wave(double t, double period, double smoothness)
{
  if (period <= 0.0) {
    return 0.0;
  }
  double ph = std::fmod(t, period) / period;
  if (ph < 0.0) {
    ph += 1.0;
  }
  double tri = 0.0;
  if (ph < 0.25) {
    tri = 4.0 * ph;
  } else if (ph < 0.75) {
    tri = 2.0 - 4.0 * ph;
  } else {
    tri = 4.0 * ph - 4.0;
  }
  const double s = std::clamp(smoothness, 0.0, 1.0);
  const double sine = std::sin(2.0 * kPi * ph);
  return (1.0 - s) * tri + s * sine;
}

// 角度 [rad] をサーボのパルス幅 [us] に変換 (範囲を超えないよう制限する)
inline double angle_to_pulse_us(
  double angle_rad, double center_us, double us_per_deg, double sign,
  double min_us, double max_us)
{
  const double deg = angle_rad * 180.0 / kPi;
  return std::clamp(center_us + sign * deg * us_per_deg, min_us, max_us);
}

// スキャン内のi番目の点の、スキャン開始からの経過時刻 [s]。
//   time_increment > 0 のときはそれを使い、0のときは scan_time / 点数 で決める。
//   reverse = true のときは、時刻の進む向きと配列の向きが逆。
inline double point_time_offset(
  std::size_t i, std::size_t n, double time_increment, double scan_time, bool reverse)
{
  if (n == 0) {
    return 0.0;
  }
  const std::size_t k = reverse ? (n - 1 - i) : i;
  if (time_increment > 0.0) {
    return static_cast<double>(k) * time_increment;
  }
  return scan_time * static_cast<double>(k) / static_cast<double>(n);
}

// スキャンを samples 個の区間に分けたとき、offset [s] がどの区間(k)の、どの位置(frac, 0〜1)か。
inline void sample_segment(double offset, double scan_time, int samples, int & k, double & frac)
{
  if (samples < 1) {
    samples = 1;
  }
  double pos = scan_time > 0.0 ? offset / scan_time * static_cast<double>(samples) : 0.0;
  pos = std::clamp(pos, 0.0, static_cast<double>(samples));
  k = static_cast<int>(std::floor(pos));
  if (k >= samples) {
    k = samples - 1;
  }
  frac = pos - static_cast<double>(k);
}

}  // namespace nodding_lidar
