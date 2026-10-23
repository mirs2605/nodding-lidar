// geometry.hpp の単体テスト。ROSなしで動く。失敗すると、終了コードが0以外になる。
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "nodding_lidar/geometry.hpp"

using namespace nodding_lidar;  // NOLINT

static int g_failures = 0;

static void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

static bool near(double a, double b, double eps = 1e-4)
{
  return std::fabs(a - b) < eps;
}

static double deg(double d) {return d * kPi / 180.0;}

int main()
{
  const double h = 0.0325;

  // 1. 回転角0度: 前方1mの点は (1, 0, h)
  {
    const Vec3 p = project_point(1.0, 0.0, 0.0, h);
    check(near(p.x, 1.0) && near(p.y, 0.0) && near(p.z, h), "phi=0 forward point");
  }

  // 2. 回転角15度: 前方1mの点は (0.9743, 0, -0.2274)。正の角度で前方が下がる。
  {
    const Vec3 p = project_point(1.0, 0.0, deg(15.0), h);
    check(near(p.x, 0.9743) && near(p.y, 0.0) && near(p.z, -0.2274), "phi=15deg forward point");
    check(p.z < 0.0, "positive angle lowers the front");
  }

  // 3. 回転角-15度: 前方が上がる
  {
    const Vec3 p = project_point(1.0, 0.0, deg(-15.0), h);
    check(p.z > 0.0, "negative angle raises the front");
  }

  // 4. 回転軸(Y軸)上の点(真横)は、y座標が回転しても変わらない
  {
    const Vec3 p = project_point(1.0, deg(90.0), deg(15.0), h);
    check(near(p.y, 1.0), "side point y is unchanged by rotation");
  }

  // 5. クォータニオン+平行移動の姿勢が、行列による計算と一致する
  {
    const double phis[] = {-15.0, -7.5, 0.0, 3.0, 15.0, 40.0};
    const double angs[] = {-170.0, -90.0, 0.0, 45.0, 135.0};
    const double ranges[] = {0.2, 1.0, 7.5};
    bool all_ok = true;
    for (double pd : phis) {
      for (double ad : angs) {
        for (double r : ranges) {
          const Pose pose = servo_to_lidar_pose(deg(pd), h);
          const Vec3 in_lidar{r * std::cos(deg(ad)), r * std::sin(deg(ad)), 0.0};
          const Vec3 a = apply_pose(pose, in_lidar);
          const Vec3 b = project_point(r, deg(ad), deg(pd), h);
          if (!(near(a.x, b.x, 1e-9) && near(a.y, b.y, 1e-9) && near(a.z, b.z, 1e-9))) {
            all_ok = false;
          }
        }
      }
    }
    check(all_ok, "pose (quaternion+translation) matches matrix projection");
  }

  // 6. 首振りの波形 (周期4秒)
  {
    check(near(nodding_wave(0.0, 4.0, 0.0), 0.0), "wave t=0");
    check(near(nodding_wave(1.0, 4.0, 0.0), 1.0), "wave t=T/4 (triangle)");
    check(near(nodding_wave(2.0, 4.0, 0.0), 0.0), "wave t=T/2 (triangle)");
    check(near(nodding_wave(3.0, 4.0, 0.0), -1.0), "wave t=3T/4 (triangle)");
    check(near(nodding_wave(4.0, 4.0, 0.0), 0.0), "wave t=T (triangle)");
    check(near(nodding_wave(0.5, 4.0, 0.0), 0.5), "wave t=T/8 (triangle)");
    check(near(nodding_wave(1.0, 4.0, 1.0), 1.0), "wave t=T/4 (sine)");
    check(near(nodding_wave(1.0, 4.0, 0.3), 1.0), "wave t=T/4 (mixed)");
    check(near(nodding_wave(-1.0, 4.0, 0.0), -1.0), "wave negative time wraps");
  }

  // 7. パルス幅
  {
    check(near(angle_to_pulse_us(0.0, 1500.0, 11.11, 1.0, 600.0, 2400.0), 1500.0), "pulse center");
    check(
      near(angle_to_pulse_us(deg(15.0), 1500.0, 11.11, 1.0, 600.0, 2400.0), 1666.65, 0.01),
      "pulse +15deg");
    check(
      near(angle_to_pulse_us(deg(15.0), 1500.0, 11.11, -1.0, 600.0, 2400.0), 1333.35, 0.01),
      "pulse sign flip");
    check(
      near(angle_to_pulse_us(deg(90.0), 1500.0, 20.0, 1.0, 600.0, 2400.0), 2400.0),
      "pulse clamp");
  }

  // 8. 点ごとの時刻
  {
    check(near(point_time_offset(50, 100, 0.0, 0.1, false), 0.05), "time offset from scan_time");
    check(near(point_time_offset(0, 100, 0.0, 0.1, true), 0.099), "time offset reversed");
    check(near(point_time_offset(10, 100, 0.001, 0.1, false), 0.01),
      "time offset from time_increment");
    check(near(point_time_offset(0, 0, 0.0, 0.1, false), 0.0), "time offset n=0");
  }

  // 9. スキャン内の区間の選び方 (8区間)
  {
    int k = -1;
    double f = -1.0;
    sample_segment(0.0, 0.08, 8, k, f);
    check(k == 0 && near(f, 0.0), "segment start");
    sample_segment(0.08, 0.08, 8, k, f);
    check(k == 7 && near(f, 1.0), "segment end");
    sample_segment(0.025, 0.08, 8, k, f);
    check(k == 2 && near(f, 0.5), "segment middle");
    sample_segment(0.5, 0.08, 8, k, f);
    check(k == 7 && near(f, 1.0), "segment clamp");
  }

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return EXIT_SUCCESS;
  }
  std::printf("%d test(s) failed\n", g_failures);
  return EXIT_FAILURE;
}
