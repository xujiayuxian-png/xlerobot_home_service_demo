#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "xlerobot_lidar_driver/lidar_types.hpp"
#include "xlerobot_lidar_driver/scan_filter.hpp"

namespace xlerobot_lidar_driver
{

struct ScanBuilderOptions
{
  std::string frame_id = "laser";
  size_t scan_size = 720;
  double range_min_m = 0.05;
  double range_max_m = 8.0;
  double scan_time_s = 1.0 / 7.0;
};

struct BuildScanResult
{
  sensor_msgs::msg::LaserScan scan;
  std::vector<AutoMaskCluster> auto_mask_clusters;
};

class ScanBuilder
{
public:
  ScanBuilder(ScanBuilderOptions options, ScanFilter filter);

  void addPoint(const LidarPoint & point);
  void addPoints(const std::vector<LidarPoint> & points);
  bool empty() const;
  size_t pointCount() const;
  void clear();

  BuildScanResult build(const builtin_interfaces::msg::Time & stamp, double scan_time_s = 0.0);

private:
  ScanBuilderOptions options_;
  ScanFilter filter_;
  std::vector<LidarPoint> points_;
};

}  // namespace xlerobot_lidar_driver
