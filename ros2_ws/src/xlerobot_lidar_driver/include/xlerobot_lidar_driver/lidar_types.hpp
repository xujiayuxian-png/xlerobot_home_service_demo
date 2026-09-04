#pragma once

#include <cstdint>
#include <vector>

namespace xlerobot_lidar_driver
{

struct LidarPoint
{
  double angle_rad = 0.0;
  double distance_m = 0.0;
  uint8_t quality = 0;
  double x = 0.0;
  double y = 0.0;
};

struct LidarPacket
{
  uint8_t ct = 0;
  uint8_t lsn = 0;
  uint16_t fsa_raw = 0;
  uint16_t lsa_raw = 0;
  bool is_scan_start = false;
  std::vector<LidarPoint> points;
};

}  // namespace xlerobot_lidar_driver
