#pragma once

#include <string>

namespace xlerobot_navigation
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct NamedPlace
{
  std::string id;
  std::string frame_id{"map"};
  Pose2D target;
  double nav_offset_m{0.0};
  bool dock{false};
};

struct NavigationPlan
{
  NamedPlace place;
  Pose2D predock;
  double spin_target_yaw{0.0};
};

double normalize_angle(double angle);
NavigationPlan make_navigation_plan(const NamedPlace & place);
double target_frame_lateral_error(const Pose2D & current, const Pose2D & target);

Pose2D make_standoff_pose(
  const Pose2D & target,
  const Pose2D & robot,
  double standoff_m,
  double minimum_separation_m = 1.0e-3);

struct DockConfig
{
  double position_tolerance_m{0.010};
  double lateral_tolerance_m{0.08};
  double yaw_tolerance_rad{0.025};
  double max_linear_speed_mps{0.035};
  double max_angular_speed_radps{0.16};
  double linear_kp{0.45};
  double angular_kp{0.9};
  double angular_deadband_rad{0.006};
  double yaw_align_release_rad{0.018};
  double yaw_align_stable_s{0.35};
  double yaw_realign_threshold_rad{0.025};
  double angular_sign_change_deadband_rad{0.04};
  double max_angular_accel_radps2{0.28};
};

enum class DockPhase
{
  kAlignYaw,
  kApproach,
  kComplete,
  kFailed,
};

struct DockStep
{
  DockPhase phase{DockPhase::kAlignYaw};
  double linear_x{0.0};
  double angular_z{0.0};
  double forward_error_m{0.0};
  double lateral_error_m{0.0};
  double yaw_error_rad{0.0};
  std::string message;
};

class DockController
{
public:
  explicit DockController(DockConfig config);

  void reset();
  DockStep update(const Pose2D & current, const Pose2D & target, double dt_s);
  DockPhase phase() const;

private:
  DockStep errors(const Pose2D & current, const Pose2D & target) const;
  bool aligned(const DockStep & step) const;
  double angular_command(double error, double dt_s);

  DockConfig config_;
  DockPhase phase_{DockPhase::kAlignYaw};
  double yaw_stable_s_{0.0};
  double previous_angular_z_{0.0};
};

std::string to_string(DockPhase phase);

}  // namespace xlerobot_navigation
