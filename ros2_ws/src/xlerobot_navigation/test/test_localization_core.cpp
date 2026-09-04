#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "xlerobot_navigation/localization_core.hpp"

namespace xlerobot_navigation
{
namespace
{

TEST(LocalizationQualityTest, ConvertsPlanarCovarianceToStandardDeviation)
{
  std::array<double, 36> covariance{};
  covariance[0] = 0.09;
  covariance[7] = 0.16;
  covariance[35] = 0.01;

  const auto quality = quality_from_covariance(covariance);

  EXPECT_NEAR(quality.position_stddev_m, 0.5, 1e-12);
  EXPECT_NEAR(quality.yaw_stddev_rad, 0.1, 1e-12);
}

TEST(LocalizationQualityTest, ClampsNegativeVarianceBeforeSquareRoot)
{
  std::array<double, 36> covariance{};
  covariance[0] = -1.0;
  covariance[7] = 0.04;
  covariance[35] = -0.1;

  const auto quality = quality_from_covariance(covariance);

  EXPECT_NEAR(quality.position_stddev_m, 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(quality.yaw_stddev_rad, 0.0);
}

TEST(ConvergenceTrackerTest, RequiresRotationQualityAndStableHold)
{
  ConvergenceTracker tracker({1.0, 0.15, 0.10, 0.5});
  const LocalizationQuality good{0.05, 0.04};

  EXPECT_FALSE(tracker.update(good, 0.9, 0.3));
  EXPECT_FALSE(tracker.update(good, 1.1, 0.2));
  EXPECT_TRUE(tracker.update(good, 1.2, 0.3));
  EXPECT_EQ(tracker.phase(), "converging");
}

TEST(ConvergenceTrackerTest, PoorQualityResetsHold)
{
  ConvergenceTracker tracker({0.5, 0.15, 0.10, 0.5});
  const LocalizationQuality good{0.05, 0.04};
  const LocalizationQuality poor{0.30, 0.04};

  EXPECT_FALSE(tracker.update(good, 1.0, 0.4));
  EXPECT_FALSE(tracker.update(poor, 1.1, 0.1));
  EXPECT_DOUBLE_EQ(tracker.hold_elapsed_s(), 0.0);
  EXPECT_FALSE(tracker.update(good, 1.2, 0.3));
}

TEST(LocalizationMathTest, NormalizesLargeAngleDelta)
{
  EXPECT_NEAR(normalize_angle_delta(3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(normalize_angle_delta(-3.0 * M_PI), M_PI, 1e-12);
}

TEST(ScanMapConsistencyTest, MatchesOccupiedCellsWithinRadius)
{
  std::vector<std::int8_t> cells(100, 0);
  cells[5U * 10U + 5U] = 100;
  const std::vector<std::array<double, 2>> endpoints{
    {5.1, 5.1},
    {5.9, 5.1},
    {2.1, 2.1},
    {20.0, 20.0},
  };
  const auto score = score_scan_endpoints(
    cells, 10, 10, 1.0, 0.0, 0.0, 0.0, endpoints, 65, 1.0);
  EXPECT_EQ(score.evaluated_beams, 3U);
  EXPECT_EQ(score.matched_beams, 2U);
  EXPECT_NEAR(score.score(), 2.0 / 3.0, 1.0e-9);
}

TEST(ScanMapConsistencyTest, HonorsMapOriginYaw)
{
  std::vector<std::int8_t> cells(16, 0);
  cells[1U * 4U + 2U] = 100;
  const double half_pi = 1.5707963267948966;
  const std::vector<std::array<double, 2>> endpoints{{9.0, 22.0}};
  const auto score = score_scan_endpoints(
    cells, 4, 4, 1.0, 10.0, 20.0, half_pi, endpoints, 65, 0.0);
  EXPECT_EQ(score.evaluated_beams, 1U);
  EXPECT_EQ(score.matched_beams, 1U);
}

TEST(ScanMapConsistencyTest, WrongPoseProducesLowScore)
{
  std::vector<std::int8_t> cells(100, 0);
  for (std::size_t y = 0; y < 10U; ++y) {
    cells[y * 10U + 8U] = 100;
  }
  const std::vector<std::array<double, 2>> correctly_projected{
    {8.1, 1.1}, {8.1, 3.1}, {8.1, 5.1}, {8.1, 7.1},
  };
  const std::vector<std::array<double, 2>> kidnapped_projection{
    {4.1, 1.1}, {4.1, 3.1}, {4.1, 5.1}, {4.1, 7.1},
  };
  const auto good = score_scan_endpoints(
    cells, 10, 10, 1.0, 0.0, 0.0, 0.0, correctly_projected, 65, 0.2);
  const auto bad = score_scan_endpoints(
    cells, 10, 10, 1.0, 0.0, 0.0, 0.0, kidnapped_projection, 65, 0.2);
  EXPECT_DOUBLE_EQ(good.score(), 1.0);
  EXPECT_DOUBLE_EQ(bad.score(), 0.0);
}

}  // namespace
}  // namespace xlerobot_navigation
