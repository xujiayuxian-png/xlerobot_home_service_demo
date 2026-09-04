#pragma once

#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace xlerobot_manipulation
{

struct PlannedJointLimit
{
  std::string name;
  double lower;
  double upper;
};

struct GraspPlanValidationConfig
{
  std::vector<PlannedJointLimit> joints;
  double max_duration_s{10.0};
  double start_tolerance_rad{0.05};
};

struct GraspPlanValidation
{
  bool accepted{false};
  std::string message;

  explicit operator bool() const {return accepted;}
};

class GraspPlanValidator
{
public:
  explicit GraspPlanValidator(GraspPlanValidationConfig config);

  GraspPlanValidation validate(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<double> & measured_positions) const;

private:
  GraspPlanValidation reject(const std::string & message) const;

  GraspPlanValidationConfig config_;
};

}  // namespace xlerobot_manipulation
