#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace xlerobot_navigation
{

struct LocalizationQuality
{
  double position_stddev_m{std::numeric_limits<double>::infinity()};
  double yaw_stddev_rad{std::numeric_limits<double>::infinity()};
};

struct ConvergenceConfig
{
  double min_rotation_rad{3.14};
  double position_stddev_m{0.15};
  double yaw_stddev_rad{0.10};
  double hold_s{1.0};
};

struct ScanMapScore
{
  std::size_t matched_beams{0};
  std::size_t evaluated_beams{0};

  double score() const;
};

ScanMapScore score_scan_endpoints(
  const std::vector<std::int8_t> & cells,
  std::size_t width,
  std::size_t height,
  double resolution_m,
  double origin_x_m,
  double origin_y_m,
  double origin_yaw_rad,
  const std::vector<std::array<double, 2>> & endpoints_map,
  std::int8_t occupied_threshold,
  double match_radius_m);

double normalize_angle_delta(double angle_rad);
LocalizationQuality quality_from_covariance(const std::array<double, 36> & covariance);

class ConvergenceTracker
{
public:
  explicit ConvergenceTracker(ConvergenceConfig config);

  void reset();
  bool update(const LocalizationQuality & quality, double rotated_rad, double dt_s);
  bool is_within_threshold(
    const LocalizationQuality & quality, double rotated_rad) const;
  double hold_elapsed_s() const;
  std::string phase() const;

private:
  ConvergenceConfig config_;
  double hold_elapsed_s_{0.0};
  bool within_threshold_{false};
};

}  // namespace xlerobot_navigation
