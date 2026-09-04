#include "xlerobot_lidar_driver/scan_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace xlerobot_lidar_driver
{

ScanFilter::ScanFilter(
  RadiusFilterConfig radius_config,
  FixedMaskConfig fixed_mask_config,
  AutoBodyPostsMaskConfig auto_body_posts_config)
: radius_config_(std::move(radius_config)),
  fixed_mask_config_(std::move(fixed_mask_config)),
  auto_body_posts_config_(std::move(auto_body_posts_config))
{
}

void ScanFilter::removeOutliers(std::vector<LidarPoint> & points) const
{
  if (!radius_config_.enabled || points.empty()) {
    return;
  }

  std::vector<LidarPoint> clean_points;
  clean_points.reserve(points.size());
  const double r2 = radius_config_.radius_m * radius_config_.radius_m;

  for (size_t i = 0; i < points.size(); ++i) {
    int neighbors = 0;
    const auto & p1 = points[i];

    for (size_t j = 0; j < points.size(); ++j) {
      if (i == j) {
        continue;
      }
      const double dx = p1.x - points[j].x;
      const double dy = p1.y - points[j].y;
      if ((dx * dx + dy * dy) < r2) {
        neighbors++;
      }
      if (neighbors >= radius_config_.min_neighbors) {
        break;
      }
    }

    if (neighbors >= radius_config_.min_neighbors) {
      clean_points.push_back(p1);
    }
  }

  points = std::move(clean_points);
}

bool ScanFilter::isFixedMasked(double angle_rad, double distance_m) const
{
  if (!fixed_mask_config_.enabled) {
    return false;
  }
  if (fixed_mask_config_.angle_ranges_deg.size() < 2) {
    return false;
  }
  if (fixed_mask_config_.max_range_m > 0.0 && distance_m > fixed_mask_config_.max_range_m) {
    return false;
  }

  const double deg = normalizeDegrees(angle_rad * 180.0 / M_PI);
  for (size_t i = 0; i + 1 < fixed_mask_config_.angle_ranges_deg.size(); i += 2) {
    const double start = normalizeDegrees(fixed_mask_config_.angle_ranges_deg[i]);
    const double end = normalizeDegrees(fixed_mask_config_.angle_ranges_deg[i + 1]);
    const bool inside = (start <= end) ? (deg >= start && deg <= end) : (deg >= start ||
      deg <= end);
    if (inside) {
      return true;
    }
  }

  return false;
}

std::vector<AutoMaskCluster> ScanFilter::applyAutoBodyPostsMask(
  sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<AutoMaskCluster> masked_clusters;
  if (!auto_body_posts_config_.enabled || scan.ranges.empty()) {
    return masked_clusters;
  }

  std::vector<int> candidate_indices;
  candidate_indices.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range >= auto_body_posts_config_.min_range_m &&
      range <= auto_body_posts_config_.max_range_m)
    {
      candidate_indices.push_back(static_cast<int>(i));
    }
  }

  if (candidate_indices.empty()) {
    return masked_clusters;
  }

  const int scan_size = static_cast<int>(scan.ranges.size());
  const int cluster_gap = std::max(
    1,
    static_cast<int>(
      std::ceil((auto_body_posts_config_.cluster_gap_deg * M_PI / 180.0) / scan.angle_increment)));
  const int padding = std::max(
    1,
    static_cast<int>(
      std::ceil((auto_body_posts_config_.padding_deg * M_PI / 180.0) / scan.angle_increment)));

  std::vector<AutoMaskCluster> clusters;
  AutoMaskCluster current{
    candidate_indices.front(),
    candidate_indices.front(),
    candidate_indices.front(),
    1,
    scan.ranges[candidate_indices.front()],
  };

  for (size_t i = 1; i < candidate_indices.size(); ++i) {
    const int index = candidate_indices[i];
    if (index - current.end_index <= cluster_gap) {
      current.end_index = index;
      current.point_count += 1;
      if (scan.ranges[index] < current.closest_range) {
        current.closest_range = scan.ranges[index];
        current.closest_index = index;
      }
    } else {
      clusters.push_back(current);
      current = AutoMaskCluster{index, index, index, 1, scan.ranges[index]};
    }
  }
  clusters.push_back(current);

  if (clusters.size() > 1) {
    AutoMaskCluster & first = clusters.front();
    AutoMaskCluster & last = clusters.back();
    const int wrap_gap = first.start_index + scan_size - last.end_index;
    if (wrap_gap <= cluster_gap) {
      last.end_index = first.end_index;
      last.point_count += first.point_count;
      if (first.closest_range < last.closest_range) {
        last.closest_range = first.closest_range;
        last.closest_index = first.closest_index;
      }
      clusters.erase(clusters.begin());
    }
  }

  clusters.erase(
    std::remove_if(
      clusters.begin(),
      clusters.end(),
      [this](const AutoMaskCluster & cluster) {
        return cluster.point_count < auto_body_posts_config_.min_cluster_points;
      }),
    clusters.end());

  if (clusters.empty()) {
    return masked_clusters;
  }

  std::sort(
    clusters.begin(),
    clusters.end(),
    [](const AutoMaskCluster & a, const AutoMaskCluster & b) {
      return a.closest_range < b.closest_range;
    });

  const int max_clusters = std::min(
    static_cast<int>(clusters.size()),
    std::max(0, auto_body_posts_config_.max_clusters));
  for (int i = 0; i < max_clusters; ++i) {
    maskCluster(scan, clusters[i], padding);
    masked_clusters.push_back(clusters[i]);
  }

  return masked_clusters;
}

const RadiusFilterConfig & ScanFilter::radiusConfig() const
{
  return radius_config_;
}

const FixedMaskConfig & ScanFilter::fixedMaskConfig() const
{
  return fixed_mask_config_;
}

const AutoBodyPostsMaskConfig & ScanFilter::autoBodyPostsConfig() const
{
  return auto_body_posts_config_;
}

double ScanFilter::normalizeDegrees(double deg)
{
  deg = std::fmod(deg, 360.0);
  if (deg < 0.0) {
    deg += 360.0;
  }
  return deg;
}

int ScanFilter::normalizeIndex(int index, int size)
{
  int normalized = index % size;
  if (normalized < 0) {
    normalized += size;
  }
  return normalized;
}

void ScanFilter::maskCluster(
  sensor_msgs::msg::LaserScan & scan,
  const AutoMaskCluster & cluster,
  int padding) const
{
  const int scan_size = static_cast<int>(scan.ranges.size());
  const int start = cluster.start_index - padding;
  const int end = cluster.end_index + padding;

  if (cluster.start_index > cluster.end_index) {
    for (int idx = start; idx < scan_size; ++idx) {
      scan.ranges[normalizeIndex(idx, scan_size)] = std::numeric_limits<float>::infinity();
    }
    for (int idx = 0; idx <= end; ++idx) {
      scan.ranges[normalizeIndex(idx, scan_size)] = std::numeric_limits<float>::infinity();
    }
    return;
  }

  for (int idx = start; idx <= end; ++idx) {
    scan.ranges[normalizeIndex(idx, scan_size)] = std::numeric_limits<float>::infinity();
  }
}

std::string summarizeAutoMaskClusters(
  const std::vector<AutoMaskCluster> & clusters,
  const sensor_msgs::msg::LaserScan & scan)
{
  std::string summary;
  for (const auto & cluster : clusters) {
    const double angle_deg = ScanFilter::normalizeDegrees(
      (scan.angle_min + cluster.closest_index * scan.angle_increment) * 180.0 / M_PI);
    char range_text[16];
    std::snprintf(range_text, sizeof(range_text), "%.2f", cluster.closest_range);
    summary += " [" + std::to_string(static_cast<int>(std::round(angle_deg))) +
      "deg " + std::string(range_text) + "m]";
  }
  return summary;
}

}  // namespace xlerobot_lidar_driver
