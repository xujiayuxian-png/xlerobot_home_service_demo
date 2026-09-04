#include "xlerobot_manipulation/grasp_plan_validator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xlerobot_manipulation
{

namespace
{

double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool finite_vector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
             return std::isfinite(value);
    });
}

}  // namespace

GraspPlanValidator::GraspPlanValidator(GraspPlanValidationConfig config)
: config_(std::move(config))
{
  if (config_.joints.empty() || !std::isfinite(config_.max_duration_s) ||
    config_.max_duration_s <= 0.0 || !std::isfinite(config_.start_tolerance_rad) ||
    config_.start_tolerance_rad < 0.0)
  {
    throw std::invalid_argument("grasp plan validation config is invalid");
  }
  for (const auto & joint : config_.joints) {
    if (joint.name.empty() || !std::isfinite(joint.lower) || !std::isfinite(joint.upper) ||
      joint.lower >= joint.upper)
    {
      throw std::invalid_argument("grasp plan joint limit is invalid");
    }
  }
}

GraspPlanValidation GraspPlanValidator::validate(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & measured_positions) const
{
  const size_t count = config_.joints.size();
  std::vector<std::string> expected_names;
  expected_names.reserve(count);
  for (const auto & joint : config_.joints) {
    expected_names.push_back(joint.name);
  }
  if (trajectory.joint_names != expected_names) {
    return reject("trajectory joint order does not match the right arm contract");
  }
  if (trajectory.points.empty()) {
    return reject("trajectory contains no points");
  }
  if (measured_positions.size() != count || !finite_vector(measured_positions)) {
    return reject("measured start state is incomplete or nonfinite");
  }

  double previous_time_s = 0.0;
  for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points[point_index];
    if (point.positions.size() != count || !finite_vector(point.positions)) {
      return reject("trajectory point positions are incomplete or nonfinite");
    }
    if ((!point.velocities.empty() &&
      (point.velocities.size() != count || !finite_vector(point.velocities))) ||
      (!point.accelerations.empty() &&
      (point.accelerations.size() != count || !finite_vector(point.accelerations))))
    {
      return reject("trajectory derivatives have invalid dimensions or values");
    }
    const double time_s = seconds(point.time_from_start);
    if (!std::isfinite(time_s) || time_s < 0.0 || time_s > config_.max_duration_s ||
      (point_index > 0 && time_s <= previous_time_s))
    {
      return reject("trajectory timing is nonmonotonic or exceeds its bound");
    }
    for (size_t joint_index = 0; joint_index < count; ++joint_index) {
      const auto & limit = config_.joints[joint_index];
      const double position = point.positions[joint_index];
      if (position < limit.lower || position > limit.upper) {
        return reject("trajectory position exceeds the configured joint limit");
      }
      if (point_index == 0 && time_s == 0.0 &&
        std::abs(position - measured_positions[joint_index]) > config_.start_tolerance_rad)
      {
        return reject("zero-time trajectory point is discontinuous from measured state");
      }
    }
    previous_time_s = time_s;
  }
  return {true, "trajectory structure and position envelope are valid"};
}

GraspPlanValidation GraspPlanValidator::reject(const std::string & message) const
{
  return {false, message};
}

}  // namespace xlerobot_manipulation
