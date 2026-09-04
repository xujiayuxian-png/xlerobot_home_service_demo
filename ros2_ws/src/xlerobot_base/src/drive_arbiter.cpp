#include "xlerobot_base/drive_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xlerobot_base
{

namespace
{

double clamp_with_flag(double value, double lower, double upper, bool & clamped)
{
  const double output = std::clamp(value, lower, upper);
  clamped = clamped || output != value;
  return output;
}

double limit_delta(double desired, double previous, double maximum_delta, bool & clamped)
{
  const double delta = std::clamp(desired - previous, -maximum_delta, maximum_delta);
  clamped = clamped || delta != desired - previous;
  return previous + delta;
}

bool finite_command(const geometry_msgs::msg::Twist & command)
{
  return
    std::isfinite(command.linear.x) &&
    std::isfinite(command.linear.y) &&
    std::isfinite(command.linear.z) &&
    std::isfinite(command.angular.x) &&
    std::isfinite(command.angular.y) &&
    std::isfinite(command.angular.z);
}

}  // namespace

DriveArbiter::DriveArbiter(DriveArbiterConfig config)
: config_(config)
{
  const auto positive_finite = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  if (!positive_finite(config_.navigation_timeout_s) ||
    !positive_finite(config_.docking_timeout_s) ||
    !positive_finite(config_.teleop_timeout_s) ||
    !positive_finite(config_.max_linear_mps) ||
    !positive_finite(config_.max_angular_radps) ||
    !positive_finite(config_.max_linear_accel_mps2) ||
    !positive_finite(config_.max_angular_accel_radps2))
  {
    throw std::invalid_argument(
            "drive arbiter limits and timeouts must be finite and positive");
  }
}

void DriveArbiter::update_navigation(
  const geometry_msgs::msg::Twist & command, double now_s)
{
  if (stopped_) {
    return;
  }
  if (!finite_command(command)) {
    navigation_ = {geometry_msgs::msg::Twist{}, 0.0, false};
    return;
  }
  navigation_ = {command, now_s, true};
}

void DriveArbiter::update_docking(const geometry_msgs::msg::Twist & command, double now_s)
{
  if (stopped_) {
    return;
  }
  if (!finite_command(command)) {
    docking_ = {geometry_msgs::msg::Twist{}, 0.0, false};
    return;
  }
  docking_ = {command, now_s, true};
}

void DriveArbiter::update_teleop(const geometry_msgs::msg::Twist & command, double now_s)
{
  if (stopped_) {
    return;
  }
  if (!finite_command(command)) {
    teleop_ = {geometry_msgs::msg::Twist{}, 0.0, false};
    return;
  }
  teleop_ = {command, now_s, true};
}

void DriveArbiter::set_stopped(bool stopped)
{
  if (stopped_ == stopped) {
    return;
  }
  stopped_ = stopped;
  navigation_ = {geometry_msgs::msg::Twist(), 0.0, false};
  docking_ = {geometry_msgs::msg::Twist(), 0.0, false};
  teleop_ = {geometry_msgs::msg::Twist(), 0.0, false};
  previous_output_ = geometry_msgs::msg::Twist{};
}

bool DriveArbiter::stopped() const
{
  return stopped_;
}

DriveSelection DriveArbiter::select(double now_s, double dt_s)
{
  if (stopped_) {
    previous_output_ = geometry_msgs::msg::Twist{};
    return {geometry_msgs::msg::Twist(), DriveSource::kNone, false};
  }
  auto selection = choose_desired(now_s);
  if (selection.source == DriveSource::kNone) {
    previous_output_ = geometry_msgs::msg::Twist{};
    return selection;
  }
  selection = apply_limits(std::move(selection), std::max(0.0, dt_s));
  previous_output_ = selection.command;
  return selection;
}

bool DriveArbiter::fresh(
  const TimedCommand & command, double now_s, double timeout_s) const
{
  return command.received && now_s >= command.received_s &&
         now_s - command.received_s <= timeout_s;
}

DriveSelection DriveArbiter::choose_desired(double now_s) const
{
  const bool navigation_fresh = fresh(navigation_, now_s, config_.navigation_timeout_s);
  const bool docking_fresh = fresh(docking_, now_s, config_.docking_timeout_s);
  const bool teleop_fresh = fresh(teleop_, now_s, config_.teleop_timeout_s);
  if (docking_fresh) {
    return {docking_.command, DriveSource::kDocking, false};
  }
  if (config_.navigation_has_priority) {
    if (navigation_fresh) {
      return {navigation_.command, DriveSource::kNavigation, false};
    }
    if (teleop_fresh) {
      return {teleop_.command, DriveSource::kTeleop, false};
    }
  } else {
    if (teleop_fresh) {
      return {teleop_.command, DriveSource::kTeleop, false};
    }
    if (navigation_fresh) {
      return {navigation_.command, DriveSource::kNavigation, false};
    }
  }
  return {geometry_msgs::msg::Twist(), DriveSource::kNone, false};
}

DriveSelection DriveArbiter::apply_limits(DriveSelection selection, double dt_s)
{
  auto & command = selection.command;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.linear.x = clamp_with_flag(
    command.linear.x, -config_.max_linear_mps, config_.max_linear_mps, selection.clamped);
  command.angular.z = clamp_with_flag(
    command.angular.z,
    -config_.max_angular_radps,
    config_.max_angular_radps,
    selection.clamped);
  if (dt_s > 0.0) {
    command.linear.x = limit_delta(
      command.linear.x,
      previous_output_.linear.x,
      config_.max_linear_accel_mps2 * dt_s,
      selection.clamped);
    command.angular.z = limit_delta(
      command.angular.z,
      previous_output_.angular.z,
      config_.max_angular_accel_radps2 * dt_s,
      selection.clamped);
  }
  return selection;
}

std::string to_string(DriveSource source)
{
  switch (source) {
    case DriveSource::kNavigation:
      return "navigation";
    case DriveSource::kDocking:
      return "docking";
    case DriveSource::kTeleop:
      return "teleop";
    case DriveSource::kNone:
    default:
      return "none";
  }
}

}  // namespace xlerobot_base
