#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/buffer_core.h"

int main()
{
  tf2::BufferCore tf(tf2::durationFromSec(10.0));
  for (int sec : {1, 3}) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp.sec = sec;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "laser";
    transform.transform.translation.x = static_cast<double>(sec - 1);
    transform.transform.rotation.w = 1.0;
    if (!tf.setTransform(transform, "test", false)) {
      std::fprintf(stderr, "Failed to add test transform at %d seconds\n", sec);
      return EXIT_FAILURE;
    }
  }

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 1;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.0F;
  scan.time_increment = 0.5F;
  scan.scan_time = 1.0F;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  scan.ranges = {1.0F, 1.0F, std::numeric_limits<float>::quiet_NaN()};

  sensor_msgs::msg::PointCloud2 cloud;
  laser_geometry::LaserProjection projector;
  projector.transformLaserScanToPointCloud(
    "base_link", scan, cloud, tf, -1.0, laser_geometry::channel_option::None);

  if (cloud.header.frame_id != "base_link" || cloud.width != 2) {
    std::fprintf(
      stderr, "Expected 2 valid points in base_link; got %u points in '%s'\n",
      cloud.width, cloud.header.frame_id.c_str());
    return EXIT_FAILURE;
  }

  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  if (!std::isfinite(*x) || std::fabs(*x - 1.0F) > 1e-4F) {
    std::fprintf(stderr, "Unexpected first point x coordinate: %f\n", *x);
    return EXIT_FAILURE;
  }
  ++x;
  if (!std::isfinite(*x) || *x <= 1.0F) {
    std::fprintf(stderr, "Expected the later point to use a later TF: %f\n", *x);
    return EXIT_FAILURE;
  }

  std::puts("LaserProjection applies time-varying TF and filters invalid ranges");
  return EXIT_SUCCESS;
}
