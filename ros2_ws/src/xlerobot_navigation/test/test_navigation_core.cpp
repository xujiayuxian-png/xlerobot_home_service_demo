#include <cmath>

#include <gtest/gtest.h>

#include "xlerobot_navigation/navigation_core.hpp"

namespace xlerobot_navigation
{
namespace
{

TEST(NavigationPlanTest, ComputesPredockAlongTargetHeading)
{
  NamedPlace table;
  table.id = "table";
  table.target = {1.0, 2.0, M_PI_2};
  table.nav_offset_m = 0.4;
  table.dock = true;

  const auto plan = make_navigation_plan(table);

  EXPECT_NEAR(plan.predock.x, 1.0, 1e-12);
  EXPECT_NEAR(plan.predock.y, 1.6, 1e-12);
  EXPECT_NEAR(plan.spin_target_yaw, M_PI_2, 1e-12);
}

TEST(NavigationPlanTest, RejectsNegativeOffset)
{
  NamedPlace place;
  place.id = "invalid";
  place.nav_offset_m = -0.1;
  EXPECT_THROW(make_navigation_plan(place), std::invalid_argument);
}

TEST(NavigationPlanTest, MeasuresLateralErrorAtFinalTableHeading)
{
  const Pose2D target{1.0, 2.0, M_PI_2};
  EXPECT_NEAR(
    target_frame_lateral_error(Pose2D{0.9, 1.6, 0.0}, target),
    -0.1, 1e-12);
}

TEST(StandoffPlanTest, PlacesGoalBetweenRobotAndTargetFacingTarget)
{
  const auto goal = make_standoff_pose(
    Pose2D{2.0, 1.0, 0.0}, Pose2D{0.0, 1.0, 0.0}, 0.8);
  EXPECT_NEAR(goal.x, 1.2, 1e-12);
  EXPECT_NEAR(goal.y, 1.0, 1e-12);
  EXPECT_NEAR(goal.yaw, 0.0, 1e-12);
}

TEST(StandoffPlanTest, RejectsCoincidentOrInvalidGeometry)
{
  EXPECT_THROW(
    make_standoff_pose(Pose2D{}, Pose2D{}, 0.8),
    std::invalid_argument);
  EXPECT_THROW(
    make_standoff_pose(Pose2D{1.0, 0.0, 0.0}, Pose2D{}, -0.1),
    std::invalid_argument);
}

TEST(DockControllerTest, AlignsYawBeforeApproach)
{
  DockConfig config;
  config.yaw_align_stable_s = 0.2;
  DockController controller(config);
  const Pose2D target{0.2 * std::cos(0.5), 0.2 * std::sin(0.5), 0.5};

  const auto align = controller.update(Pose2D{0.0, 0.0, 0.0}, target, 0.1);
  EXPECT_EQ(align.phase, DockPhase::kAlignYaw);
  EXPECT_DOUBLE_EQ(align.linear_x, 0.0);
  EXPECT_GT(align.angular_z, 0.0);

  EXPECT_EQ(
    controller.update(Pose2D{0.0, 0.0, 0.5}, target, 0.1).phase,
    DockPhase::kAlignYaw);
  const auto approach = controller.update(Pose2D{0.0, 0.0, 0.5}, target, 0.1);
  EXPECT_EQ(approach.phase, DockPhase::kApproach);
  EXPECT_DOUBLE_EQ(approach.linear_x, 0.0);

  const auto command = controller.update(Pose2D{0.0, 0.0, 0.5}, target, 0.1);
  EXPECT_EQ(command.phase, DockPhase::kApproach);
  EXPECT_GT(command.linear_x, 0.0);
}

TEST(DockControllerTest, PreservesVerifiedSpeedAndAccelerationLimits)
{
  DockController controller(DockConfig{});
  const auto first = controller.update(Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 1.0}, 0.05);
  EXPECT_LE(std::abs(first.angular_z), 0.28 * 0.05 + 1e-12);
  EXPECT_LE(std::abs(first.angular_z), 0.16 + 1e-12);
}

TEST(DockControllerTest, PreservesStraightApproachAfterYawAlignment)
{
  DockConfig config;
  config.yaw_align_stable_s = 0.0;
  DockController controller(config);
  const Pose2D target{0.4, 0.08, 0.0};

  const auto enter_approach = controller.update(Pose2D{}, target, 0.1);
  ASSERT_EQ(enter_approach.phase, DockPhase::kApproach);

  const auto command = controller.update(Pose2D{}, target, 0.1);
  EXPECT_EQ(command.phase, DockPhase::kApproach);
  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
  EXPECT_EQ(command.message, "low-speed approach");
}

TEST(DockControllerTest, RejectsLateralErrorAfterYawAlignment)
{
  DockConfig config;
  config.yaw_align_stable_s = 0.0;
  DockController controller(config);

  const auto failed = controller.update(
    Pose2D{0.0, 0.0, 0.0}, Pose2D{0.2, 0.3, 0.0}, 0.1);

  EXPECT_EQ(failed.phase, DockPhase::kFailed);
  EXPECT_NE(failed.message.find("lateral"), std::string::npos);
}

TEST(DockControllerTest, CompletesOnlyInsideAllTolerances)
{
  DockController controller(DockConfig{});
  const auto complete = controller.update(
    Pose2D{0.995, 2.0, 0.01}, Pose2D{1.0, 2.0, 0.0}, 0.1);

  EXPECT_EQ(complete.phase, DockPhase::kComplete);
  EXPECT_DOUBLE_EQ(complete.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(complete.angular_z, 0.0);
}

TEST(NavigationMathTest, NormalizesAcrossPiBoundary)
{
  EXPECT_NEAR(normalize_angle(3.0 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(normalize_angle(-3.0 * M_PI), -M_PI, 1e-12);
}

}  // namespace
}  // namespace xlerobot_navigation
