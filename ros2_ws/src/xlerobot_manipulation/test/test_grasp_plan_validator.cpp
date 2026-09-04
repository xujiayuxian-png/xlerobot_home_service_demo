#include <gtest/gtest.h>

#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "xlerobot_manipulation/grasp_plan_validator.hpp"

namespace xlerobot_manipulation
{
namespace
{

GraspPlanValidationConfig config()
{
  return {{
    {"j0", -1.0, 1.0},
    {"j1", -2.0, 2.0}}, 3.0, 0.05};
}

trajectory_msgs::msg::JointTrajectory safe_trajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"j0", "j1"};
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {0.0, 0.0};
  trajectory.points.push_back(start);
  trajectory_msgs::msg::JointTrajectoryPoint finish;
  finish.positions = {0.2, -0.4};
  finish.time_from_start.sec = 1;
  trajectory.points.push_back(finish);
  return trajectory;
}

TEST(GraspPlanValidatorTest, AcceptsBoundedContinuousPlan)
{
  GraspPlanValidator validator(config());
  const auto result = validator.validate(safe_trajectory(), {0.0, 0.0});
  EXPECT_TRUE(result) << result.message;
}

TEST(GraspPlanValidatorTest, AcceptsFiniteMoveItDerivatives)
{
  GraspPlanValidator validator(config());
  auto trajectory = safe_trajectory();
  auto & finish = trajectory.points.back();
  finish.time_from_start.sec = 0;
  finish.time_from_start.nanosec = 100000000;
  finish.positions = {0.05, -0.10};
  finish.velocities = {10.0, -10.0};
  finish.accelerations = {50.0, -50.0};
  const auto result = validator.validate(trajectory, {0.0, 0.0});
  EXPECT_TRUE(result) << result.message;
}

TEST(GraspPlanValidatorTest, RejectsLayoutDimensionsAndStartDiscontinuity)
{
  GraspPlanValidator validator(config());
  auto trajectory = safe_trajectory();
  trajectory.joint_names = {"j1", "j0"};
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
  trajectory = safe_trajectory();
  trajectory.points.front().positions[0] = 0.2;
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
  trajectory = safe_trajectory();
  trajectory.points.back().positions = {0.2};
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
}

TEST(GraspPlanValidatorTest, RejectsPositionDerivativeShapeAndTimeFailures)
{
  GraspPlanValidator validator(config());
  auto trajectory = safe_trajectory();
  trajectory.points.back().positions[0] = 1.1;
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
  trajectory = safe_trajectory();
  trajectory.points.back().time_from_start.sec = 0;
  trajectory.points.back().time_from_start.nanosec = 100000000;
  trajectory.points.back().positions[0] = 0.2;
  EXPECT_TRUE(validator.validate(trajectory, {0.0, 0.0}));
  trajectory = safe_trajectory();
  trajectory.points.back().velocities = {1.1};
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
  trajectory = safe_trajectory();
  trajectory.points.back().accelerations = {4.1};
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
  trajectory = safe_trajectory();
  trajectory.points.back().time_from_start.sec = 4;
  EXPECT_FALSE(validator.validate(trajectory, {0.0, 0.0}));
}

}  // namespace
}  // namespace xlerobot_manipulation
