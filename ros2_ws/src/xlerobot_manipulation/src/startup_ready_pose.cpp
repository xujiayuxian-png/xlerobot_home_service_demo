#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono_literals;

class StartupReadyPose : public rclcpp::Node
{
public:
  StartupReadyPose()
  : Node("startup_ready_pose")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    arm_joint_names_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      {
        "right_arm_shoulder_pan", "right_arm_shoulder_lift",
        "right_arm_elbow_flex", "right_arm_wrist_flex", "right_arm_wrist_roll"
      });
    arm_ready_positions_ = declare_parameter<std::vector<double>>(
      "arm_ready_positions", {-0.11505, 1.62142, 1.62602, 0.50621, 0.00153});
    gripper_joint_ = declare_parameter<std::string>(
      "gripper_joint", "right_arm_gripper");
    gripper_ready_position_ = declare_parameter<double>("gripper_ready_position", 0.0);
    head_joint_names_ = declare_parameter<std::vector<std::string>>(
      "head_joint_names", {"head_pan_joint", "head_tilt_joint"});
    head_ready_positions_ = declare_parameter<std::vector<double>>(
      "head_ready_positions", {0.0, 0.0});
    arm_duration_s_ = declare_parameter<double>("arm_duration_s", 5.0);
    gripper_duration_s_ = declare_parameter<double>("gripper_duration_s", 2.0);
    head_duration_s_ = declare_parameter<double>("head_duration_s", 2.0);
    joint_state_timeout_s_ = declare_parameter<double>("joint_state_timeout_s", 10.0);
    controller_wait_s_ = declare_parameter<double>("controller_wait_s", 20.0);
    result_margin_s_ = declare_parameter<double>("result_margin_s", 5.0);
    arrival_tolerance_rad_ = declare_parameter<double>("arrival_tolerance_rad", 0.12);
    arm_action_ = declare_parameter<std::string>(
      "arm_action", "/right_arm_controller/follow_joint_trajectory");
    gripper_action_ = declare_parameter<std::string>(
      "gripper_action", "/right_gripper_controller/follow_joint_trajectory");
    head_action_ = declare_parameter<std::string>(
      "head_action", "/head_controller/follow_joint_trajectory");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");

    validate_parameters();
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, arm_action_);
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, gripper_action_);
    head_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, head_action_);
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(position_mutex_);
        const auto count = std::min(message->name.size(), message->position.size());
        for (std::size_t index = 0; index < count; ++index) {
          if (std::isfinite(message->position[index])) {
            positions_[message->name[index]] = message->position[index];
          }
        }
      });
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    diagnostic_timer_ = create_wall_timer(1s, [this]() {publish_diagnostic();});
    set_status(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      execution_enabled_ ? "waiting to move robot to ready pose" : "startup ready disabled");
  }

  bool initialize()
  {
    if (!execution_enabled_) {
      set_status(diagnostic_msgs::msg::DiagnosticStatus::OK, "startup ready disabled");
      return true;
    }
    set_status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "waiting for measured joint state");
    auto required = arm_joint_names_;
    required.push_back(gripper_joint_);
    required.insert(required.end(), head_joint_names_.begin(), head_joint_names_.end());
    if (!wait_for_positions(required, joint_state_timeout_s_)) {
      return fail("required right-arm or head joint state was not available");
    }
    set_status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "centering head for ready pose");
    if (!move_if_needed(
        head_client_, head_action_, head_joint_names_, head_ready_positions_, head_duration_s_))
    {
      return fail("head failed to reach centered ready pose");
    }
    set_status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "moving right arm to ready pose");
    if (!move_if_needed(
        arm_client_, arm_action_, arm_joint_names_, arm_ready_positions_, arm_duration_s_))
    {
      return fail("right arm failed to reach ready pose");
    }
    set_status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "closing gripper for ready pose");
    if (!move_if_needed(
        gripper_client_, gripper_action_, {gripper_joint_}, {gripper_ready_position_},
        gripper_duration_s_))
    {
      return fail("right gripper failed to reach ready pose");
    }
    set_status(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "right arm, gripper, and head are in ready pose");
    RCLCPP_INFO(get_logger(), "Startup ready pose complete");
    return true;
  }

private:
  void validate_parameters() const
  {
    if (arm_joint_names_.size() != 5 || arm_ready_positions_.size() != arm_joint_names_.size()) {
      throw std::invalid_argument("startup ready requires five arm joints and positions");
    }
    if (head_joint_names_.size() != 2 ||
      head_ready_positions_.size() != head_joint_names_.size())
    {
      throw std::invalid_argument("startup ready requires two head joints and positions");
    }
    if (gripper_joint_.empty() || arm_action_.empty() || gripper_action_.empty() ||
      head_action_.empty() ||
      joint_state_topic_.empty())
    {
      throw std::invalid_argument("startup ready names and topics must not be empty");
    }
    for (const auto value : arm_ready_positions_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("startup ready arm positions must be finite");
      }
    }
    for (const auto value : head_ready_positions_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("startup ready head positions must be finite");
      }
    }
    const std::vector<double> positive = {
      arm_duration_s_, gripper_duration_s_, head_duration_s_, joint_state_timeout_s_,
      controller_wait_s_,
      result_margin_s_, arrival_tolerance_rad_};
    if (!std::isfinite(gripper_ready_position_) ||
      std::any_of(positive.begin(), positive.end(), [](double value) {
        return !std::isfinite(value) || value <= 0.0;
      }))
    {
      throw std::invalid_argument("startup ready numeric parameters must be finite and positive");
    }
  }

  void set_status(uint8_t level, std::string message)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_level_ = level;
      status_message_ = std::move(message);
    }
    publish_diagnostic();
  }

  void publish_diagnostic()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "xlerobot/startup_ready";
    status.hardware_id = "two_wheel_reference";
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status.level = status_level_;
      status.message = status_message_;
    }
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  bool fail(const std::string & message)
  {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    set_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    return false;
  }

  bool wait_for_positions(const std::vector<std::string> & names, double timeout_s)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (at_positions(names, {}, false)) {
        return true;
      }
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool at_positions(
    const std::vector<std::string> & names, const std::vector<double> & targets,
    bool compare_targets) const
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto found = positions_.find(names[index]);
      if (found == positions_.end()) {
        return false;
      }
      if (compare_targets && std::abs(found->second - targets[index]) > arrival_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  bool move_if_needed(
    const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr & client,
    const std::string & action_name, const std::vector<std::string> & names,
    const std::vector<double> & targets, double duration_s)
  {
    if (at_positions(names, targets, true)) {
      RCLCPP_INFO(get_logger(), "%s already at ready target", action_name.c_str());
      return true;
    }
    if (!client->wait_for_action_server(std::chrono::duration<double>(controller_wait_s_))) {
      RCLCPP_ERROR(get_logger(), "Action server unavailable: %s", action_name.c_str());
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = names;
    goal.trajectory.header.stamp = now() + rclcpp::Duration::from_seconds(0.1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = targets;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    goal.trajectory.points.push_back(std::move(point));

    const auto send_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(
        shared_from_this(), send_future,
        std::chrono::duration<double>(controller_wait_s_)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Timed out sending ready goal to %s", action_name.c_str());
      return false;
    }
    const auto goal_handle = send_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Ready goal rejected by %s", action_name.c_str());
      return false;
    }
    const auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(
        shared_from_this(), result_future,
        std::chrono::duration<double>(duration_s + result_margin_s_)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      (void)client->async_cancel_goal(goal_handle);
      RCLCPP_ERROR(get_logger(), "Ready goal timed out on %s", action_name.c_str());
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      RCLCPP_ERROR(get_logger(), "Ready trajectory failed on %s", action_name.c_str());
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      if (at_positions(names, targets, true)) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    RCLCPP_ERROR(get_logger(), "Measured joints did not settle at %s target", action_name.c_str());
    return false;
  }

  bool execution_enabled_{false};
  std::vector<std::string> arm_joint_names_;
  std::vector<double> arm_ready_positions_;
  std::string gripper_joint_;
  double gripper_ready_position_{0.0};
  std::vector<std::string> head_joint_names_;
  std::vector<double> head_ready_positions_;
  double arm_duration_s_{5.0};
  double gripper_duration_s_{2.0};
  double head_duration_s_{2.0};
  double joint_state_timeout_s_{10.0};
  double controller_wait_s_{20.0};
  double result_margin_s_{5.0};
  double arrival_tolerance_rad_{0.12};
  std::string arm_action_;
  std::string gripper_action_;
  std::string head_action_;
  std::string joint_state_topic_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr head_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  mutable std::mutex position_mutex_;
  std::unordered_map<std::string, double> positions_;
  std::mutex status_mutex_;
  uint8_t status_level_{diagnostic_msgs::msg::DiagnosticStatus::WARN};
  std::string status_message_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<StartupReadyPose>();
  (void)node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
