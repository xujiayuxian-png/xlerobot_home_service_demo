#pragma once

#include <string>

#include <geometry_msgs/msg/twist.hpp>

namespace xlerobot_base
{

enum class DriveSource
{
  kNone,
  kDocking,
  kNavigation,
  kTeleop,
};

struct DriveArbiterConfig
{
  double navigation_timeout_s{0.5};
  double docking_timeout_s{0.35};
  double teleop_timeout_s{0.35};
  double max_linear_mps{0.15};
  double max_angular_radps{0.6};
  double max_linear_accel_mps2{0.35};
  double max_angular_accel_radps2{1.2};
  bool navigation_has_priority{true};
};

struct DriveSelection
{
  geometry_msgs::msg::Twist command;
  DriveSource source{DriveSource::kNone};
  bool clamped{false};
};

class DriveArbiter
{
public:
  explicit DriveArbiter(DriveArbiterConfig config);

  void update_navigation(const geometry_msgs::msg::Twist & command, double now_s);
  void update_docking(const geometry_msgs::msg::Twist & command, double now_s);
  void update_teleop(const geometry_msgs::msg::Twist & command, double now_s);
  void set_stopped(bool stopped);
  bool stopped() const;
  DriveSelection select(double now_s, double dt_s);

private:
  struct TimedCommand
  {
    geometry_msgs::msg::Twist command;
    double received_s{0.0};
    bool received{false};
  };

  bool fresh(const TimedCommand & command, double now_s, double timeout_s) const;
  DriveSelection choose_desired(double now_s) const;
  DriveSelection apply_limits(DriveSelection selection, double dt_s);

  DriveArbiterConfig config_;
  TimedCommand navigation_;
  TimedCommand docking_;
  TimedCommand teleop_;
  geometry_msgs::msg::Twist previous_output_;
  bool stopped_{false};
};

std::string to_string(DriveSource source);

}  // namespace xlerobot_base
