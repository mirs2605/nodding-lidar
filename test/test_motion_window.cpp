#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "nodding_lidar/motion_window.hpp"

namespace
{
void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

nodding_lidar::MotionScan scan(double t, std::size_t n = 1)
{
  return {t, std::vector<nodding_lidar::MotionPoint>(n, {1, 2, 3})};
}
}  // namespace

/** @brief 時刻補間・切返し・欠測と、蓄積の時間・角度・メモリ上限を検証する。
 * @return 全条件を満たせば0、失敗時1。
 */
int main()
{
  using nodding_lidar::MotionHistory;
  using nodding_lidar::MotionWindow;
  try {
    MotionHistory stationary(10, 0.2);
    MotionHistory turning(10, 0.2);
    for (int i = 0; i <= 30; ++i) {
      stationary.add(1.0 + i * 0.1, 0.0);
      turning.add(1.0 + i * 0.1, 0.4);
    }
    require(!turning.add(2.0, 0.4), "Past odometry must be ignored");
    require(!turning.add(5.0, std::numeric_limits<double>::quiet_NaN()), "Reject NaN");
    require(!turning.span(0.0, 1.0), "Do not extrapolate");
    require(std::abs(*turning.span(1.05, 1.95) - 0.36) < 1e-9, "Interpolate boundaries");
    MotionWindow straight(2, 0.2, 32, 1000);
    MotionWindow turn(2, 0.2, 32, 1000);
    for (int i = 0; i <= 30; ++i) {
      const double t = 1.0 + i * 0.1;
      straight.add(scan(t), stationary);
      turn.add(scan(t), turning);
    }
    require(straight.scans().size() >= 20, "Straight motion retains long window");
    require(turn.scans().size() <= 6 && turn.scans().size() >= 5, "Shorten during turn");
    require(!turn.add(scan(4.0), turning), "Ignore duplicate scan");

    MotionHistory reversal(10, 2);
    reversal.add(1, 1); reversal.add(2, -1);
    require(std::abs(*reversal.span(1, 2) - 0.5) < 1e-9, "Reversal cannot cancel span");
    MotionHistory gap(10, 0.2);
    gap.add(1, 0); gap.add(2, 0);
    require(!gap.span(1, 2), "Missing motion segment must not be interpolated");
    MotionWindow missing(2, 0.2, 32, 100);
    missing.add(scan(1), gap); missing.add(scan(2), gap);
    require(missing.scans().size() == 1, "Fallback to single scan on missing motion");

    MotionWindow bounds(2, 0.2, 3, 4);
    bounds.add(scan(1, 2), stationary); bounds.add(scan(1.1, 2), stationary);
    bounds.add(scan(1.2, 2), stationary);
    require(bounds.point_count() == 4 && bounds.scans().size() == 2, "Bound retained points");
    require(!bounds.add(scan(1.3, 5), stationary), "Reject oversized single scan");
    MotionWindow count(2, 0.2, 2, 100);
    for (int i = 0; i < 4; ++i) {count.add(scan(1 + i * 0.1), stationary);}
    require(count.scans().size() == 2, "Bound scan count");
    count.clear(); gap.clear();
    require(count.point_count() == 0 && count.scans().empty(), "Reset point window");
    require(!gap.span(1, 2), "Reset motion history");
    bool invalid = false;
    try {MotionWindow bad(-1, 1, 1, 1);} catch (const std::invalid_argument &) {invalid = true;}
    require(invalid, "Reject invalid limits");
    std::cout << "Motion window checks passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
