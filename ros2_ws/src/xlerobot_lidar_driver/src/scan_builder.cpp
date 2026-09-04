#include "xlerobot_lidar_driver/scan_builder.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace xlerobot_lidar_driver
{

ScanBuilder::ScanBuilder(ScanBuilderOptions options, ScanFilter filter)
: options_(std::move(options)), filter_(std::move(filter))
{
  points_.reserve(1000);
}

void ScanBuilder::addPoint(const LidarPoint & point)
{
  points_.push_back(point);
}

void ScanBuilder::addPoints(const std::vector<LidarPoint> & points)
{
  points_.insert(points_.end(), points.begin(), points.end());
}

bool ScanBuilder::empty() const
{
  return points_.empty();
}

size_t ScanBuilder::pointCount() const
{
  return points_.size();
}

void ScanBuilder::clear()
{
  points_.clear();
}

BuildScanResult ScanBuilder::build(const builtin_interfaces::msg::Time & stamp, double scan_time_s)
{
  filter_.removeOutliers(points_);

  BuildScanResult result;
  auto & scan = result.scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = options_.frame_id;
  scan.angle_min = 0.0;
  scan.angle_max = 2.0 * M_PI;
  scan.angle_increment = (2.0 * M_PI) / static_cast<double>(options_.scan_size);
  scan.range_min = options_.range_min_m;
  scan.range_max = options_.range_max_m;
  scan.ranges.assign(options_.scan_size, std::numeric_limits<float>::infinity());
  scan.intensities.assign(options_.scan_size, 0.0f);

  for (const auto & point : points_) {
    // The lidar is mounted opposite to the ROS scan convention used by this
    // robot, so raw packet angles are mirrored before assigning /scan bins.
    double index_angle = 2.0 * M_PI - point.angle_rad;
    if (index_angle >= 2.0 * M_PI) {
      index_angle -= 2.0 * M_PI;
    }
    if (index_angle < 0.0) {
      index_angle += 2.0 * M_PI;
    }

    const size_t index = static_cast<size_t>(index_angle / scan.angle_increment);
    if (index >= options_.scan_size) {
      continue;
    }
    if (filter_.isFixedMasked(index_angle, point.distance_m)) {
      continue;
    }
    if (scan.ranges[index] == std::numeric_limits<float>::infinity() ||
      point.distance_m < scan.ranges[index])
    {
      scan.ranges[index] = static_cast<float>(point.distance_m);
      scan.intensities[index] = static_cast<float>(point.quality);
    }
  }

  result.auto_mask_clusters = filter_.applyAutoBodyPostsMask(scan);
  scan.scan_time = (scan_time_s > 0.0) ? scan_time_s : options_.scan_time_s;
  scan.time_increment = scan.scan_time / static_cast<double>(options_.scan_size);
  clear();
  return result;
}

}  // namespace xlerobot_lidar_driver
