#ifndef NODDING_LIDAR__MOTION_WINDOW_HPP_
#define NODDING_LIDAR__MOTION_WINDOW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace nodding_lidar
{
/** @brief odom座標の距離点 [m]。無効値は格納前に除く。 */
struct MotionPoint
{
  float x, y, z;
};

/** @brief 1スキャンの基準時刻 [s] と、その時刻のTFでodomへ変換した点列。 */
struct MotionScan
{
  double stamp;
  std::vector<MotionPoint> points;
};

/** @brief 車体yaw角速度 [rad/s] の履歴から絶対角速度の積分 [rad] を求める。
 *
 * 履歴のない区間やmax_gap_sを超えるサンプル間を外挿しない。
 * 左右切返しは符号付き速度を線形補間し、絶対値を区分積分する。
 */
class MotionHistory
{
public:
  /** @brief 履歴保持時間とサンプル間隔の許容上限を設定する。
   * @param history_s 履歴保持時間 [s]。正の有限値。
   * @param max_gap_s サンプル間隔の上限 [s]。正の有限値。
   */
  MotionHistory(double history_s, double max_gap_s)
  : history_s_(history_s), max_gap_s_(max_gap_s)
  {
    if (!std::isfinite(history_s) || history_s <= 0.0 ||
      !std::isfinite(max_gap_s) || max_gap_s <= 0.0)
    {
      throw std::invalid_argument("Motion history limits must be positive and finite");
    }
  }

  /** @brief 有限で時刻が増加する角速度を保存する。重複・過去の入力は無視する。
   * @param stamp 計測時刻 [s]。
   * @param rate 車体z軸まわり角速度 [rad/s]。反時計回りが正。
   * @return 保存した場合true。
   */
  bool add(double stamp, double rate)
  {
    if (!std::isfinite(stamp) || !std::isfinite(rate) ||
      (!samples_.empty() && stamp <= samples_.back().stamp))
    {
      return false;
    }
    samples_.push_back({stamp, rate});
    while (samples_.size() > 2 && samples_[1].stamp < stamp - history_s_) {
      samples_.pop_front();
    }
    // 時間幅内でも入力頻度が異常に高い場合のメモリを制限する。
    while (samples_.size() > 10000) {
      samples_.pop_front();
    }
    return true;
  }

  /** @brief 指定区間での絶対角速度の積分を求める。
   * @param begin 積分開始時刻 [s]。
   * @param end 積分終了時刻 [s]。begin以上。
   * @return 積分値 [rad]。履歴不足、長い欠測、無効な区間はnullopt。
   */
  std::optional<double> span(double begin, double end) const
  {
    if (!std::isfinite(begin) || !std::isfinite(end) || begin > end ||
      samples_.empty() || begin < samples_.front().stamp || end > samples_.back().stamp)
    {
      return std::nullopt;
    }
    double area = 0.0;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      const auto & a = samples_[i - 1];
      const auto & b = samples_[i];
      const double lo = std::max(begin, a.stamp);
      const double hi = std::min(end, b.stamp);
      if (hi <= lo) {continue;}
      const double dt = b.stamp - a.stamp;
      if (dt > max_gap_s_) {return std::nullopt;}
      const double v0 = a.rate + (b.rate - a.rate) * (lo - a.stamp) / dt;
      const double v1 = a.rate + (b.rate - a.rate) * (hi - a.stamp) / dt;
      if (v0 * v1 < 0.0) {
        const double s = std::abs(v0) + std::abs(v1);
        area += (hi - lo) * (v0 * v0 + v1 * v1) / (2.0 * s);
      } else {
        area += (hi - lo) * (std::abs(v0) + std::abs(v1)) / 2.0;
      }
    }
    return area;
  }

  /** @brief 時刻逆行時などに全履歴を破棄する。 */
  void clear() {samples_.clear();}

private:
  struct Sample {double stamp, rate;};
  double history_s_, max_gap_s_;
  std::deque<Sample> samples_;
};

/** @brief 時間・旋回量・スキャン数・点数の上限内でodom座標の点群を保持する。 */
class MotionWindow
{
public:
  /** @brief 有限の保持上限を設定する。
   * @param seconds 時間幅の上限 [s]。正の有限値。
   * @param radians 絶対角速度積分の上限 [rad]。正の有限値。
   * @param scans スキャン数の上限。1以上。
   * @param points 点数の上限。1以上。
   */
  MotionWindow(double seconds, double radians, std::size_t scans, std::size_t points)
  : seconds_(seconds), radians_(radians), max_scans_(scans), max_points_(points)
  {
    if (!std::isfinite(seconds) || seconds <= 0.0 || !std::isfinite(radians) ||
      radians <= 0.0 || scans == 0 || points == 0)
    {
      throw std::invalid_argument("Motion window limits must be positive and finite");
    }
  }

  /** @brief 点群を追加して古い点群を除き、最新時刻までの有効な履歴だけを残す。
   * @param scan 時刻が増加する、有限のodom座標点群。
   * @param history 車体角速度履歴。欠測時は単発点群だけを残す。
   * @return 保存した場合true。重複時刻、過去時刻、空または点数超過の入力はfalse。
   */
  bool add(MotionScan scan, const MotionHistory & history)
  {
    if (!std::isfinite(scan.stamp) || scan.points.empty() || scan.points.size() > max_points_ ||
      (!scans_.empty() && scan.stamp <= scans_.back().stamp))
    {
      return false;
    }
    point_count_ += scan.points.size();
    scans_.push_back(std::move(scan));
    while (scans_.size() > 1) {
      const double first = scans_.front().stamp;
      const double last = scans_.back().stamp;
      const auto angle = history.span(first, last);
      if (last - first <= seconds_ && scans_.size() <= max_scans_ &&
        point_count_ <= max_points_ && angle && *angle <= radians_)
      {
        break;
      }
      point_count_ -= scans_.front().points.size();
      scans_.pop_front();
    }
    return true;
  }

  /** @brief 保持中の点群を計測時刻順に返す。
   * @return このインスタンスが所有する点群列への参照。
   */
  const std::deque<MotionScan> & scans() const {return scans_;}
  /** @brief 保持点数を返す。
   * @return 全スキャンの点数の合計。
   */
  std::size_t point_count() const {return point_count_;}
  /** @brief 全点群と保持点数を初期化する。 */
  void clear() {scans_.clear(); point_count_ = 0;}

private:
  double seconds_, radians_;
  std::size_t max_scans_, max_points_, point_count_{0};
  std::deque<MotionScan> scans_;
};
}  // namespace nodding_lidar
#endif  // NODDING_LIDAR__MOTION_WINDOW_HPP_
