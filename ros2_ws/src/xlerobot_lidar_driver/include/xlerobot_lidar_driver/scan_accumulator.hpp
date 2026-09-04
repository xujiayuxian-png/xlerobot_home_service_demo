#pragma once

#include <vector>

#include "xlerobot_lidar_driver/lidar_types.hpp"

namespace xlerobot_lidar_driver
{

struct CompletedScan
{
  std::vector<LidarPoint> points;
};

class ScanAccumulator
{
public:
  std::vector<CompletedScan> addPacket(const LidarPacket & packet);
  void clear();

private:
  CompletedScan finishCurrentScan();

  std::vector<LidarPoint> points_;
  double last_point_angle_ = 0.0;
  bool use_ct_scan_boundaries_ = false;
  bool angle_wrap_seen_since_ct_ = false;
};

}  // namespace xlerobot_lidar_driver
