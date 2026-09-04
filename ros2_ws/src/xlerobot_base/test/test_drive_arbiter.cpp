#include <gtest/gtest.h>

#include <limits>

#include <geometry_msgs/msg/twist.hpp>

#include "xlerobot_base/drive_arbiter.hpp"

namespace xlerobot_base
{
namespace
{

geometry_msgs::msg::Twist command(double linear, double angular)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = linear;
  result.angular.z = angular;
  return result;
}

DriveArbiterConfig config()
{
  DriveArbiterConfig result;
  result.max_linear_accel_mps2 = 10.0;
  result.max_angular_accel_radps2 = 10.0;
  return result;
}

TEST(DriveArbiterTest, FreshNavigationIsSelected)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.2), 1.0);

  const auto selected = arbiter.select(1.1, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kNavigation);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.1);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, 0.2);
}

TEST(DriveArbiterTest, NavigationHasPriorityWhileFresh)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.0), 1.0);
  arbiter.update_teleop(command(0.05, 0.3), 1.1);

  const auto selected = arbiter.select(1.2, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kNavigation);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.1);
}

TEST(DriveArbiterTest, DockingHasPriorityOverNavigationAndTeleop)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.0), 1.0);
  arbiter.update_teleop(command(0.05, 0.1), 1.0);
  arbiter.update_docking(command(0.02, -0.05), 1.0);

  const auto selected = arbiter.select(1.1, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kDocking);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.02);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, -0.05);
}

TEST(DriveArbiterTest, StaleDockingFallsBackToFreshNavigation)
{
  DriveArbiter arbiter(config());
  arbiter.update_docking(command(0.02, 0.0), 1.0);
  arbiter.update_navigation(command(0.1, 0.0), 1.4);

  const auto selected = arbiter.select(1.5, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kNavigation);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.1);
}

TEST(DriveArbiterTest, TeleopRunsAfterNavigationExpires)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.0), 1.0);
  arbiter.update_teleop(command(0.05, 0.3), 1.5);

  const auto selected = arbiter.select(1.6, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kTeleop);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.05);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, 0.3);
}

TEST(DriveArbiterTest, StaleCommandsStopImmediately)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.2), 1.0);
  EXPECT_EQ(arbiter.select(1.1, 0.1).source, DriveSource::kNavigation);

  const auto selected = arbiter.select(2.0, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kNone);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, 0.0);
}

TEST(DriveArbiterTest, ClampsVelocityAndAcceleration)
{
  auto limits = config();
  limits.max_linear_accel_mps2 = 0.2;
  limits.max_angular_accel_radps2 = 0.4;
  DriveArbiter arbiter(limits);
  arbiter.update_navigation(command(1.0, 2.0), 1.0);

  const auto selected = arbiter.select(1.1, 0.1);

  EXPECT_TRUE(selected.clamped);
  EXPECT_NEAR(selected.command.linear.x, 0.02, 1e-12);
  EXPECT_NEAR(selected.command.angular.z, 0.04, 1e-12);
}

TEST(DriveArbiterTest, StopLatchBypassesAccelerationLimit)
{
  auto limits = config();
  limits.max_linear_accel_mps2 = 0.01;
  DriveArbiter arbiter(limits);
  arbiter.update_navigation(command(0.1, 0.2), 1.0);
  EXPECT_GT(arbiter.select(1.1, 1.0).command.linear.x, 0.0);

  arbiter.set_stopped(true);
  const auto selected = arbiter.select(1.2, 0.001);

  EXPECT_TRUE(arbiter.stopped());
  EXPECT_EQ(selected.source, DriveSource::kNone);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, 0.0);
  EXPECT_FALSE(selected.clamped);
}

TEST(DriveArbiterTest, CommandsReceivedWhileStoppedAreDiscarded)
{
  DriveArbiter arbiter(config());
  arbiter.set_stopped(true);
  arbiter.update_navigation(command(0.1, 0.2), 1.0);
  arbiter.update_docking(command(0.02, 0.1), 1.0);
  arbiter.update_teleop(command(0.05, 0.3), 1.0);

  arbiter.set_stopped(false);
  const auto selected = arbiter.select(1.1, 0.1);

  EXPECT_FALSE(arbiter.stopped());
  EXPECT_EQ(selected.source, DriveSource::kNone);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, 0.0);
}

TEST(DriveArbiterTest, ClearingStopDoesNotRevivePreStopCommand)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.2), 1.0);
  EXPECT_EQ(arbiter.select(1.1, 0.1).source, DriveSource::kNavigation);

  arbiter.set_stopped(true);
  arbiter.set_stopped(false);
  EXPECT_EQ(arbiter.select(1.2, 0.1).source, DriveSource::kNone);

  arbiter.update_navigation(command(0.05, -0.1), 1.3);
  const auto selected = arbiter.select(1.4, 0.1);
  EXPECT_EQ(selected.source, DriveSource::kNavigation);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.05);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, -0.1);
}

TEST(DriveArbiterTest, NonFiniteCommandInvalidatesItsSource)
{
  DriveArbiter arbiter(config());
  arbiter.update_navigation(command(0.1, 0.2), 1.0);
  EXPECT_EQ(arbiter.select(1.1, 0.1).source, DriveSource::kNavigation);

  auto invalid = command(0.05, 0.1);
  invalid.linear.y = std::numeric_limits<double>::quiet_NaN();
  arbiter.update_navigation(invalid, 1.2);
  const auto selected = arbiter.select(1.3, 0.1);

  EXPECT_EQ(selected.source, DriveSource::kNone);
  EXPECT_DOUBLE_EQ(selected.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.command.angular.z, 0.0);
}

TEST(DriveArbiterTest, RejectsNonFiniteSafetyConfiguration)
{
  auto limits = config();
  limits.max_linear_mps = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)DriveArbiter{limits}, std::invalid_argument);

  limits = config();
  limits.navigation_timeout_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)DriveArbiter{limits}, std::invalid_argument);
}

}  // namespace
}  // namespace xlerobot_base
