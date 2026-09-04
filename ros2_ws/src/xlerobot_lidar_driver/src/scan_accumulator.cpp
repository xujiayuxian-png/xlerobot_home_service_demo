#include "xlerobot_lidar_driver/scan_accumulator.hpp"

#include <cmath>
#include <utility>

namespace xlerobot_lidar_driver
{

namespace
{
constexpr double kAngleWrapThresholdRad = M_PI;
}  // namespace

std::vector<CompletedScan> ScanAccumulator::addPacket(const LidarPacket & packet)
{
  std::vector<CompletedScan> completed_scans;
  bool suppress_angle_wrap_check = packet.is_scan_start;

  if (packet.is_scan_start) {
    use_ct_scan_boundaries_ = true;
    angle_wrap_seen_since_ct_ = false;
    if (!points_.empty()) {
      completed_scans.push_back(finishCurrentScan());
    }
  }

  for (const auto & point : packet.points) {
    if (!suppress_angle_wrap_check &&
      point.angle_rad < last_point_angle_ - kAngleWrapThresholdRad)
    {
      if (!use_ct_scan_boundaries_) {
        completed_scans.push_back(finishCurrentScan());
      } else if (angle_wrap_seen_since_ct_) {
        completed_scans.push_back(finishCurrentScan());
        use_ct_scan_boundaries_ = false;
        angle_wrap_seen_since_ct_ = false;
      } else {
        angle_wrap_seen_since_ct_ = true;
      }
    }

    points_.push_back(point);
    last_point_angle_ = point.angle_rad;
    suppress_angle_wrap_check = false;
  }

  return completed_scans;
}

void ScanAccumulator::clear()
{
  points_.clear();
  last_point_angle_ = 0.0;
  use_ct_scan_boundaries_ = false;
  angle_wrap_seen_since_ct_ = false;
}

CompletedScan ScanAccumulator::finishCurrentScan()
{
  CompletedScan completed;
  completed.points = std::move(points_);
  points_.clear();
  last_point_angle_ = 0.0;
  return completed;
}

}  // namespace xlerobot_lidar_driver
