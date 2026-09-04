#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "xlerobot_lidar_driver/lidar_types.hpp"

namespace xlerobot_lidar_driver
{

struct RadiusFilterConfig
{
  bool enabled = true;
  double radius_m = 0.10;
  int min_neighbors = 1;
};

struct FixedMaskConfig
{
  bool enabled = false;
  std::vector<double> angle_ranges_deg;
  double max_range_m = 0.0;
};

struct AutoBodyPostsMaskConfig
{
  bool enabled = false;
  double min_range_m = 0.12;
  double max_range_m = 0.35;
  double padding_deg = 3.0;
  double cluster_gap_deg = 4.0;
  int max_clusters = 4;
  int min_cluster_points = 1;
};

struct AutoMaskCluster
{
  int start_index = 0;
  int end_index = 0;
  int closest_index = 0;
  int point_count = 0;
  double closest_range = 0.0;
};

class ScanFilter
{
public:
  ScanFilter() = default;
  ScanFilter(
    RadiusFilterConfig radius_config,
    FixedMaskConfig fixed_mask_config,
    AutoBodyPostsMaskConfig auto_body_posts_config);

  void removeOutliers(std::vector<LidarPoint> & points) const;
  bool isFixedMasked(double angle_rad, double distance_m) const;
  std::vector<AutoMaskCluster> applyAutoBodyPostsMask(sensor_msgs::msg::LaserScan & scan) const;

  const RadiusFilterConfig & radiusConfig() const;
  const FixedMaskConfig & fixedMaskConfig() const;
  const AutoBodyPostsMaskConfig & autoBodyPostsConfig() const;

  static double normalizeDegrees(double deg);
  static int normalizeIndex(int index, int size);

private:
  void maskCluster(sensor_msgs::msg::LaserScan & scan, const AutoMaskCluster & cluster, int padding)
  const;

  RadiusFilterConfig radius_config_;
  FixedMaskConfig fixed_mask_config_;
  AutoBodyPostsMaskConfig auto_body_posts_config_;
};

std::string summarizeAutoMaskClusters(
  const std::vector<AutoMaskCluster> & clusters,
  const sensor_msgs::msg::LaserScan & scan);

}  // namespace xlerobot_lidar_driver
