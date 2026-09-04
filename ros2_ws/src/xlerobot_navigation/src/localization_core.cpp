#include "xlerobot_navigation/localization_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xlerobot_navigation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

}  // namespace

double normalize_angle_delta(double angle_rad)
{
  return std::remainder(angle_rad, kTwoPi);
}

LocalizationQuality quality_from_covariance(const std::array<double, 36> & covariance)
{
  const double x_variance = std::max(0.0, covariance[0]);
  const double y_variance = std::max(0.0, covariance[7]);
  const double yaw_variance = std::max(0.0, covariance[35]);
  return {
    std::sqrt(x_variance + y_variance),
    std::sqrt(yaw_variance),
  };
}

double ScanMapScore::score() const
{
  if (evaluated_beams == 0U) {
    return 0.0;
  }
  return static_cast<double>(matched_beams) / static_cast<double>(evaluated_beams);
}

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
  double match_radius_m)
{
  if (width == 0U || height == 0U || cells.size() != width * height ||
    !std::isfinite(resolution_m) || resolution_m <= 0.0 ||
    !std::isfinite(match_radius_m) || match_radius_m < 0.0)
  {
    throw std::invalid_argument("scan-map grid parameters are invalid");
  }

  const double cos_yaw = std::cos(origin_yaw_rad);
  const double sin_yaw = std::sin(origin_yaw_rad);
  const int radius_cells = static_cast<int>(std::ceil(match_radius_m / resolution_m));
  ScanMapScore result;

  for (const auto & endpoint : endpoints_map) {
    if (!std::isfinite(endpoint[0]) || !std::isfinite(endpoint[1])) {
      continue;
    }
    const double dx = endpoint[0] - origin_x_m;
    const double dy = endpoint[1] - origin_y_m;
    const double grid_x_m = cos_yaw * dx + sin_yaw * dy;
    const double grid_y_m = -sin_yaw * dx + cos_yaw * dy;
    const int center_x = static_cast<int>(std::floor(grid_x_m / resolution_m));
    const int center_y = static_cast<int>(std::floor(grid_y_m / resolution_m));
    if (center_x < 0 || center_y < 0 || center_x >= static_cast<int>(width) ||
      center_y >= static_cast<int>(height))
    {
      continue;
    }

    ++result.evaluated_beams;
    bool matched = false;
    for (int offset_y = -radius_cells; offset_y <= radius_cells && !matched; ++offset_y) {
      for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x) {
        if (offset_x * offset_x + offset_y * offset_y > radius_cells * radius_cells) {
          continue;
        }
        const int cell_x = center_x + offset_x;
        const int cell_y = center_y + offset_y;
        if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(width) ||
          cell_y >= static_cast<int>(height))
        {
          continue;
        }
        const auto value = cells[
          static_cast<std::size_t>(cell_y) * width + static_cast<std::size_t>(cell_x)];
        if (value >= occupied_threshold) {
          matched = true;
          break;
        }
      }
    }
    if (matched) {
      ++result.matched_beams;
    }
  }
  return result;
}

ConvergenceTracker::ConvergenceTracker(ConvergenceConfig config)
: config_(config)
{
  if (config_.min_rotation_rad < 0.0 || config_.position_stddev_m <= 0.0 ||
    config_.yaw_stddev_rad <= 0.0 || config_.hold_s < 0.0)
  {
    throw std::invalid_argument("localization convergence limits are invalid");
  }
}

void ConvergenceTracker::reset()
{
  hold_elapsed_s_ = 0.0;
  within_threshold_ = false;
}

bool ConvergenceTracker::update(
  const LocalizationQuality & quality, double rotated_rad, double dt_s)
{
  within_threshold_ = is_within_threshold(quality, rotated_rad);
  if (!within_threshold_) {
    hold_elapsed_s_ = 0.0;
    return false;
  }
  hold_elapsed_s_ += std::max(0.0, dt_s);
  return hold_elapsed_s_ >= config_.hold_s;
}

bool ConvergenceTracker::is_within_threshold(
  const LocalizationQuality & quality, double rotated_rad) const
{
  return rotated_rad >= config_.min_rotation_rad &&
         std::isfinite(quality.position_stddev_m) &&
         std::isfinite(quality.yaw_stddev_rad) &&
         quality.position_stddev_m <= config_.position_stddev_m &&
         quality.yaw_stddev_rad <= config_.yaw_stddev_rad;
}

double ConvergenceTracker::hold_elapsed_s() const
{
  return hold_elapsed_s_;
}

std::string ConvergenceTracker::phase() const
{
  return within_threshold_ ? "converging" : "rotating";
}

}  // namespace xlerobot_navigation
