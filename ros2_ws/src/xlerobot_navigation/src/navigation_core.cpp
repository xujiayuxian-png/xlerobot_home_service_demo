#include "xlerobot_navigation/navigation_core.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace xlerobot_navigation
{

namespace
{

double clamp(double value, double lower, double upper)
{
  return std::clamp(value, lower, upper);
}

}  // namespace

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

NavigationPlan make_navigation_plan(const NamedPlace & place)
{
  if (place.id.empty() || place.frame_id.empty()) {
    throw std::invalid_argument("named place id and frame must not be empty");
  }
  if (!std::isfinite(place.target.x) || !std::isfinite(place.target.y) ||
    !std::isfinite(place.target.yaw) || !std::isfinite(place.nav_offset_m) ||
    place.nav_offset_m < 0.0)
  {
    throw std::invalid_argument("named place values must be finite and offset non-negative");
  }
  NavigationPlan plan;
  plan.place = place;
  plan.predock = place.target;
  plan.predock.x -= place.nav_offset_m * std::cos(place.target.yaw);
  plan.predock.y -= place.nav_offset_m * std::sin(place.target.yaw);
  plan.spin_target_yaw = normalize_angle(place.target.yaw);
  return plan;
}

double target_frame_lateral_error(const Pose2D & current, const Pose2D & target)
{
  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  return -std::sin(target.yaw) * dx + std::cos(target.yaw) * dy;
}

Pose2D make_standoff_pose(
  const Pose2D & target,
  const Pose2D & robot,
  double standoff_m,
  double minimum_separation_m)
{
  const double values[] = {
    target.x, target.y, robot.x, robot.y, standoff_m, minimum_separation_m};
  if (!std::all_of(std::begin(values), std::end(values), [](double value) {
      return std::isfinite(value);
    }) || standoff_m <= 0.0 || minimum_separation_m <= 0.0)
  {
    throw std::invalid_argument("standoff geometry must be finite and positive");
  }
  const double dx = robot.x - target.x;
  const double dy = robot.y - target.y;
  const double separation_m = std::hypot(dx, dy);
  if (separation_m < minimum_separation_m) {
    throw std::invalid_argument("target is too close to compute a standoff pose");
  }
  Pose2D goal;
  goal.x = target.x + standoff_m * dx / separation_m;
  goal.y = target.y + standoff_m * dy / separation_m;
  goal.yaw = std::atan2(target.y - goal.y, target.x - goal.x);
  return goal;
}

DockController::DockController(DockConfig config)
: config_(config)
{
  if (config_.position_tolerance_m <= 0.0 || config_.lateral_tolerance_m <= 0.0 ||
    config_.yaw_tolerance_rad <= 0.0 ||
    config_.max_linear_speed_mps <= 0.0 || config_.max_angular_speed_radps <= 0.0 ||
    config_.linear_kp <= 0.0 || config_.angular_kp <= 0.0 ||
    config_.yaw_align_release_rad <= 0.0 || config_.yaw_align_stable_s < 0.0 ||
    config_.yaw_realign_threshold_rad <= 0.0 ||
    config_.max_angular_accel_radps2 <= 0.0)
  {
    throw std::invalid_argument("dock controller limits must be positive");
  }
}

void DockController::reset()
{
  phase_ = DockPhase::kAlignYaw;
  yaw_stable_s_ = 0.0;
  previous_angular_z_ = 0.0;
}

DockStep DockController::update(const Pose2D & current, const Pose2D & target, double dt_s)
{
  auto step = errors(current, target);
  step.phase = phase_;
  if (aligned(step)) {
    phase_ = DockPhase::kComplete;
    step.phase = phase_;
    step.message = "dock tolerances satisfied";
    return step;
  }
  if (phase_ == DockPhase::kComplete || phase_ == DockPhase::kFailed) {
    step.phase = phase_;
    return step;
  }
  if (phase_ == DockPhase::kApproach &&
    std::abs(step.lateral_error_m) > config_.lateral_tolerance_m)
  {
    phase_ = DockPhase::kFailed;
    step.phase = phase_;
    step.message = "lateral error exceeded docking limit";
    return step;
  }

  const double safe_dt = std::max(0.0, dt_s);
  if (phase_ == DockPhase::kAlignYaw) {
    if (std::abs(step.yaw_error_rad) <= config_.yaw_align_release_rad) {
      yaw_stable_s_ += safe_dt;
      if (yaw_stable_s_ >= config_.yaw_align_stable_s) {
        if (std::abs(step.lateral_error_m) > config_.lateral_tolerance_m) {
          phase_ = DockPhase::kFailed;
          step.phase = phase_;
          step.message = "lateral error too large after yaw alignment";
          return step;
        }
        phase_ = DockPhase::kApproach;
        previous_angular_z_ = 0.0;
        step.phase = phase_;
        step.message = "yaw stable; entering low-speed approach";
        return step;
      }
    } else {
      yaw_stable_s_ = 0.0;
    }
    step.angular_z = angular_command(step.yaw_error_rad, safe_dt);
    step.message = "aligning yaw";
    return step;
  }

  if (std::abs(step.yaw_error_rad) > config_.yaw_realign_threshold_rad) {
    phase_ = DockPhase::kAlignYaw;
    yaw_stable_s_ = 0.0;
    previous_angular_z_ = 0.0;
    step.phase = phase_;
    step.message = "yaw drifted; returning to alignment";
    return step;
  }
  step.linear_x = clamp(
    config_.linear_kp * step.forward_error_m,
    -config_.max_linear_speed_mps,
    config_.max_linear_speed_mps);
  step.angular_z = angular_command(step.yaw_error_rad, safe_dt);
  step.message = "low-speed approach";
  return step;
}

DockPhase DockController::phase() const
{
  return phase_;
}

DockStep DockController::errors(const Pose2D & current, const Pose2D & target) const
{
  DockStep step;
  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  step.forward_error_m = std::cos(current.yaw) * dx + std::sin(current.yaw) * dy;
  step.lateral_error_m = -std::sin(current.yaw) * dx + std::cos(current.yaw) * dy;
  step.yaw_error_rad = normalize_angle(target.yaw - current.yaw);
  return step;
}

bool DockController::aligned(const DockStep & step) const
{
  return std::abs(step.forward_error_m) <= config_.position_tolerance_m &&
         std::abs(step.lateral_error_m) <= config_.lateral_tolerance_m &&
         std::abs(step.yaw_error_rad) <= config_.yaw_tolerance_rad;
}

double DockController::angular_command(double error, double dt_s)
{
  double command = std::abs(error) <= config_.angular_deadband_rad ?
    0.0 :
    clamp(
    config_.angular_kp * error,
    -config_.max_angular_speed_radps,
    config_.max_angular_speed_radps);
  if (command * previous_angular_z_ < 0.0 &&
    std::abs(error) <= config_.angular_sign_change_deadband_rad)
  {
    command = 0.0;
  }
  const double max_delta = config_.max_angular_accel_radps2 * dt_s;
  if (max_delta > 0.0) {
    command = clamp(
      command,
      previous_angular_z_ - max_delta,
      previous_angular_z_ + max_delta);
  }
  previous_angular_z_ = command;
  return command;
}

std::string to_string(DockPhase phase)
{
  switch (phase) {
    case DockPhase::kAlignYaw:
      return "align_yaw";
    case DockPhase::kApproach:
      return "approach";
    case DockPhase::kComplete:
      return "complete";
    case DockPhase::kFailed:
      return "failed";
    default:
      return "unknown";
  }
}

}  // namespace xlerobot_navigation
