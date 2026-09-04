#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "xlerobot_interfaces/action/set_maintenance_preset.hpp"
#include "xlerobot_interfaces/msg/capability_error.hpp"

namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using Preset = xlerobot_interfaces::action::SetMaintenancePreset;
using PresetHandle = rclcpp_action::ServerGoalHandle<Preset>;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;
using namespace std::chrono_literals;

struct Target
{
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client;
  std::string action_name;
  std::vector<std::string> joints;
  std::vector<double> positions;
  double duration_s{1.0};
};

class MaintenancePresetServer : public rclcpp::Node
{
public:
  MaintenancePresetServer()
  : Node("maintenance_preset_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    action_name_ = declare_parameter<std::string>("action_name", "/maintenance_preset");
    arm_action_ = declare_parameter<std::string>(
      "arm_action", "/right_arm_controller/follow_joint_trajectory");
    head_action_ = declare_parameter<std::string>(
      "head_action", "/head_controller/follow_joint_trajectory");
    gripper_action_ = declare_parameter<std::string>(
      "gripper_action", "/right_gripper_controller/follow_joint_trajectory");
    arm_joints_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll"});
    head_joints_ = declare_parameter<std::vector<std::string>>(
      "head_joint_names", {"head_pan_joint", "head_tilt_joint"});
    gripper_joints_ = declare_parameter<std::vector<std::string>>(
      "gripper_joint_names", {"right_arm_gripper"});
    arm_ready_ = declare_parameter<std::vector<double>>(
      "arm_ready_positions", {-0.11505, 1.62142, 1.62602, 0.50621, 0.00153});
    head_ready_ = declare_parameter<std::vector<double>>(
      "head_ready_positions", {0.0, 0.0});
    gripper_open_ = declare_parameter<std::vector<double>>(
      "gripper_open_positions", {1.64});
    arm_duration_s_ = declare_parameter<double>("arm_duration_s", 5.0);
    head_duration_s_ = declare_parameter<double>("head_duration_s", 1.5);
    gripper_duration_s_ = declare_parameter<double>("gripper_duration_s", 1.5);
    controller_wait_s_ = declare_parameter<double>("controller_wait_s", 5.0);
    result_margin_s_ = declare_parameter<double>("result_margin_s", 5.0);
    validate();

    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, arm_action_);
    head_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, head_action_);
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, gripper_action_);
    server_ = rclcpp_action::create_server<Preset>(
      this, action_name_,
      std::bind(
        &MaintenancePresetServer::goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&MaintenancePresetServer::cancel, this, std::placeholders::_1),
      std::bind(&MaintenancePresetServer::accepted, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Maintenance presets ready; execution_enabled=%s",
      execution_enabled_ ? "true" : "false");
  }

private:
  void validate() const
  {
    const auto valid_pair = [](const auto & names, const auto & positions) {
        return !names.empty() && names.size() == positions.size() &&
               std::all_of(positions.begin(), positions.end(), [](double value) {
                   return std::isfinite(value);
               });
      };
    if (!valid_pair(arm_joints_, arm_ready_) || !valid_pair(head_joints_, head_ready_) ||
      !valid_pair(gripper_joints_, gripper_open_))
    {
      throw std::invalid_argument("maintenance preset names and targets must match");
    }
    for (const auto value : {
        arm_duration_s_, head_duration_s_, gripper_duration_s_, controller_wait_s_,
        result_margin_s_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("maintenance preset durations must be positive");
      }
    }
  }

  rclcpp_action::GoalResponse goal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const Preset::Goal> request)
  {
    const std::unordered_set<std::string> presets = {
      Preset::Goal::ARM_READY, Preset::Goal::HEAD_READY, Preset::Goal::GRIPPER_OPEN};
    if (presets.count(request->preset) == 0 || busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel(const std::shared_ptr<PresetHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accepted(const std::shared_ptr<PresetHandle> handle)
  {
    std::thread([this, handle]() {execute(handle);}).detach();
  }

  Target target(const std::string & preset) const
  {
    if (preset == Preset::Goal::ARM_READY) {
      return {arm_client_, arm_action_, arm_joints_, arm_ready_, arm_duration_s_};
    }
    if (preset == Preset::Goal::HEAD_READY) {
      return {head_client_, head_action_, head_joints_, head_ready_, head_duration_s_};
    }
    return {
      gripper_client_, gripper_action_, gripper_joints_, gripper_open_, gripper_duration_s_};
  }

  static void feedback(
    const std::shared_ptr<PresetHandle> & handle, const std::string & phase,
    float progress, const std::string & message)
  {
    auto value = std::make_shared<Preset::Feedback>();
    value->state.phase = phase;
    value->state.progress = progress;
    value->state.message = message;
    handle->publish_feedback(value);
  }

  void execute(const std::shared_ptr<PresetHandle> & handle)
  {
    struct BusyReset
    {
      std::atomic_bool & busy;
      ~BusyReset() {busy.store(false);}
    } reset{busy_};
    auto result = std::make_shared<Preset::Result>();
    const auto request = handle->get_goal();
    if (request->dry_run || !execution_enabled_) {
      result->error.code = request->dry_run ? CapabilityError::NONE :
        CapabilityError::UNAVAILABLE;
      result->error.message = request->dry_run ? "maintenance preset dry-run complete" :
        "maintenance preset execution is disabled";
      if (request->dry_run) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
      return;
    }
    const auto selected = target(request->preset);
    feedback(handle, "WAIT_CONTROLLER", 0.1F, "waiting for normal controller action");
    if (!selected.client->wait_for_action_server(
        std::chrono::duration<double>(controller_wait_s_)))
    {
      result->error.code = CapabilityError::UNAVAILABLE;
      result->error.message = "controller action unavailable: " + selected.action_name;
      handle->abort(result);
      return;
    }
    if (handle->is_canceling()) {
      result->error.code = CapabilityError::CANCELED;
      result->error.message = "maintenance preset canceled before motion";
      handle->canceled(result);
      return;
    }

    FollowJointTrajectory::Goal trajectory;
    trajectory.trajectory.joint_names = selected.joints;
    trajectory.trajectory.header.stamp = now() + rclcpp::Duration::from_seconds(0.1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = selected.positions;
    point.time_from_start = rclcpp::Duration::from_seconds(selected.duration_s);
    trajectory.trajectory.points.push_back(std::move(point));
    feedback(handle, "MOVING", 0.35F, "executing bounded maintenance preset");
    auto send = selected.client->async_send_goal(trajectory);
    if (send.wait_for(std::chrono::duration<double>(controller_wait_s_)) !=
      std::future_status::ready)
    {
      result->error.code = CapabilityError::TIMEOUT;
      result->error.message = "timed out sending maintenance trajectory";
      handle->abort(result);
      return;
    }
    const auto controller_handle = send.get();
    if (!controller_handle) {
      result->error.code = CapabilityError::BACKEND_FAILURE;
      result->error.message = "maintenance trajectory was rejected";
      handle->abort(result);
      return;
    }
    auto future = selected.client->async_get_result(controller_handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(selected.duration_s + result_margin_s_);
    while (future.wait_for(20ms) != std::future_status::ready) {
      if (handle->is_canceling()) {
        (void)selected.client->async_cancel_goal(controller_handle);
        result->error.code = CapabilityError::CANCELED;
        result->error.message = "maintenance preset canceled";
        handle->canceled(result);
        return;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        (void)selected.client->async_cancel_goal(controller_handle);
        result->error.code = CapabilityError::TIMEOUT;
        result->error.message = "maintenance trajectory timed out";
        handle->abort(result);
        return;
      }
    }
    const auto wrapped = future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      result->error.code = CapabilityError::BACKEND_FAILURE;
      result->error.message = "maintenance controller reported failure";
      handle->abort(result);
      return;
    }
    feedback(handle, "COMPLETE", 1.0F, "maintenance preset reached");
    result->error.code = CapabilityError::NONE;
    result->error.message = "maintenance preset reached";
    handle->succeed(result);
  }

  bool execution_enabled_{false};
  std::string action_name_;
  std::string arm_action_;
  std::string head_action_;
  std::string gripper_action_;
  std::vector<std::string> arm_joints_;
  std::vector<std::string> head_joints_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> arm_ready_;
  std::vector<double> head_ready_;
  std::vector<double> gripper_open_;
  double arm_duration_s_{5.0};
  double head_duration_s_{1.5};
  double gripper_duration_s_{1.5};
  double controller_wait_s_{5.0};
  double result_margin_s_{5.0};
  std::atomic_bool busy_{false};
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr head_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Server<Preset>::SharedPtr server_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaintenancePresetServer>());
  rclcpp::shutdown();
  return 0;
}
