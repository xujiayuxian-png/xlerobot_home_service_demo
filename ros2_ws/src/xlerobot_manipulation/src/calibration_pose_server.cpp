// Copyright 2026 Lisa

#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "xlerobot_interfaces/action/move_calibration_pose.hpp"
#include "xlerobot_interfaces/msg/capability_error.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
using Follow = control_msgs::action::FollowJointTrajectory;
using Move = xlerobot_interfaces::action::MoveCalibrationPose;
using Handle = rclcpp_action::ServerGoalHandle<Move>;
using Error = xlerobot_interfaces::msg::CapabilityError;
using namespace std::chrono_literals;

std::vector<std::vector<double>> load_head_poses(const std::string & path)
{
  // The sequencer and this local motion server consume the same pose file.
  // No fallback pose table: an absent file cannot silently change motion.
  if (path.empty()) {return {};}
  const auto document = YAML::LoadFile(path);
  if (document["schema"].as<std::string>() != "xlerobot_calibration_pose_set/v1" ||
    document["workflow"].as<std::string>() != "head_camera" ||
    document["units"].as<std::string>() != "rad" ||
    !document["poses"].IsSequence() || document["poses"].size() < 12)
  {
    throw std::runtime_error("invalid head-camera calibration pose configuration");
  }
  std::vector<std::vector<double>> poses;
  std::set<std::pair<double, double>> unique;
  for (const auto & item : document["poses"]) {
    const auto pan = item["pan"].as<double>();
    const auto tilt = item["tilt"].as<double>();
    if (!std::isfinite(pan) || !std::isfinite(tilt) ||
      pan < -0.30 || pan > 0.30 || tilt < 0.60 || tilt > 1.00 ||
      !unique.emplace(pan, tilt).second)
    {
      throw std::runtime_error(
          "head-camera poses must be unique and inside the reference scan range");
    }
    poses.push_back({pan, tilt});
  }
  return poses;
}

std::vector<std::vector<double>> load_handeye_poses(const std::string & path)
{
  if (path.empty()) {return {};}
  const auto document = YAML::LoadFile(path);
  const std::vector<std::string> joints{
    "right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
    "right_arm_wrist_flex", "right_arm_wrist_roll"};
  if (document["schema"].as<std::string>() != "xlerobot_calibration_pose_set/v1" ||
    document["workflow"].as<std::string>() != "right_handeye" ||
    document["units"].as<std::string>() != "rad" ||
    document["fit_count"].as<int>() != 20 ||
    document["joint_order"].as<std::vector<std::string>>() != joints ||
    document["head_pose"].as<std::vector<double>>() != std::vector<double>{0.0, 0.796136} ||
    !document["poses"].IsSequence() || document["poses"].size() != 26)
  {
    throw std::runtime_error("invalid right-handeye calibration pose configuration");
  }
  const std::vector<std::pair<double, double>> bounds{
    // Flex envelope includes the per-pose upward visibility adjustments.
    {-0.92, 0.36}, {-0.56, 1.39}, {0.10, 1.19}, {-0.98, 0.46}, {1.50, 1.51}};
  const double offset = document["wrist_roll_offset_rad"] ?
    document["wrist_roll_offset_rad"].as<double>() : 0.0;
  if (offset != 0.0 && offset != -std::acos(-1.0) / 2.0) {
    throw std::runtime_error("unsupported hand-eye wrist mounting offset");
  }
  std::vector<std::vector<double>> poses;
  std::set<std::vector<double>> unique;
  for (const auto & item : document["poses"]) {
    const auto row = item.as<std::vector<double>>();
    if (row.size() != 5 || !unique.insert(row).second) {
      throw std::runtime_error("hand-eye requires distinct five-joint poses");
    }
    for (size_t i = 0; i < row.size(); ++i) {
      if (!std::isfinite(row[i]) || row[i] < bounds[i].first || row[i] > bounds[i].second) {
        throw std::runtime_error("hand-eye pose outside reference capture envelope");
      }
    }
    auto adjusted = row;
    adjusted[4] += offset;
    poses.push_back(adjusted);
  }
  return poses;
}

class CalibrationPoseServer : public rclcpp::Node
{
public:
  CalibrationPoseServer()
  : Node("calibration_pose_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    allowed_workflow_ = declare_parameter<std::string>("workflow_id", "");
    head_poses_ = load_head_poses(declare_parameter<std::string>("head_pose_file", ""));
    arm_poses_ = load_handeye_poses(declare_parameter<std::string>("handeye_pose_file", ""));
    const auto ready_path = declare_parameter<std::string>("ready_pose_file", "");
    if (!ready_path.empty()) {
      const auto ready = YAML::LoadFile(ready_path)["startup_ready_pose"]["ros__parameters"];
      arm_ready_ = ready["arm_ready_positions"].as<std::vector<double>>();
      head_ready_ = ready["head_ready_positions"].as<std::vector<double>>();
      if (arm_ready_.size() != 5 || head_ready_.size() != 2) {
        throw std::runtime_error("invalid shared ready pose dimensions");
      }
      for (const auto & values : {arm_ready_, head_ready_}) {
        for (const auto value : values) {
          if (!std::isfinite(value)) {throw std::runtime_error("non-finite ready pose");}
        }
      }
    }
    if (allowed_workflow_ == Move::Goal::HEAD_CAMERA && head_poses_.empty()) {
      throw std::runtime_error("head_pose_file is required for head-camera motion");
    }
    if (allowed_workflow_ == Move::Goal::RIGHT_HANDEYE && arm_poses_.empty()) {
      throw std::runtime_error("handeye_pose_file is required for right-handeye motion");
    }
    head_client_ = rclcpp_action::create_client<Follow>(
      this, "/head_controller/follow_joint_trajectory");
    arm_client_ = rclcpp_action::create_client<Follow>(
      this, "/right_arm_controller/follow_joint_trajectory");
    server_ = rclcpp_action::create_server<Move>(
      this, "/calibration/move_pose",
      std::bind(&CalibrationPoseServer::goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CalibrationPoseServer::cancel, this, std::placeholders::_1),
      std::bind(&CalibrationPoseServer::accepted, this, std::placeholders::_1));
  }

private:
  size_t count(const std::string & workflow) const
  {
    if (!allowed_workflow_.empty() && workflow != allowed_workflow_) {return 0;}
    if (workflow == Move::Goal::HEAD_CAMERA) {
      return head_poses_.size();
    }
    if (workflow == Move::Goal::RIGHT_HANDEYE) {
      return arm_poses_.size();
    }
    return 0;
  }

  rclcpp_action::GoalResponse goal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const Move::Goal> request)
  {
    const auto poses = count(request->workflow_id);
    if (
      poses == 0 || (!request->return_ready && request->pose_index >= poses) ||
      (request->return_ready && (head_ready_.empty() || arm_ready_.empty())) ||
      motion_unconfirmed_.load() ||
      busy_.exchange(true))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel(const std::shared_ptr<Handle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accepted(const std::shared_ptr<Handle> handle)
  {
    std::thread([this, handle]() {execute(handle);}).detach();
  }

  static void feedback(
    const std::shared_ptr<Handle> & handle, float progress, const std::string & message)
  {
    auto value = std::make_shared<Move::Feedback>();
    value->state.phase = "MOVING_CALIBRATION_POSE";
    value->state.progress = progress;
    value->state.message = message;
    handle->publish_feedback(value);
  }

  bool follow(
    const std::shared_ptr<Handle> & handle,
    const rclcpp_action::Client<Follow>::SharedPtr & client,
    const std::vector<std::string> & joints,
    const std::vector<double> & positions,
    double duration_s,
    std::string & error,
    bool & controller_terminal)
  {
    controller_terminal = true;
    try {
      if (!client->wait_for_action_server(5s)) {
        error = "calibration controller is unavailable";
        return false;
      }
    } catch (const std::exception & exception) {
      error = "calibration controller readiness failed: " + std::string(exception.what());
      return false;
    } catch (...) {
      error = "calibration controller readiness failed";
      return false;
    }
    Follow::Goal goal;
    goal.trajectory.joint_names = joints;
    goal.trajectory.header.stamp = now() + rclcpp::Duration::from_seconds(0.1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    goal.trajectory.points.push_back(std::move(point));
    try {
      auto sent = client->async_send_goal(goal);
      // Once the request has entered the action transport, a timeout or
      // exception can mean that the controller accepted it without the
      // response reaching us. Only an explicit rejection or a terminal result
      // makes ownership safe to release.
      controller_terminal = false;
      if (sent.wait_for(5s) != std::future_status::ready) {
        error = "calibration trajectory goal response timed out; motion ownership is unconfirmed";
        return false;
      }
      const auto controller_handle = sent.get();
      if (!controller_handle) {
        controller_terminal = true;
        error = "calibration trajectory was rejected";
        return false;
      }
      auto result = client->async_get_result(controller_handle);
      const auto try_cancel = [&client, &controller_handle]() noexcept {
          try {
            (void)client->async_cancel_goal(controller_handle);
          } catch (...) {
          }
        };
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(duration_s + 5.0);
      while (result.wait_for(20ms) != std::future_status::ready) {
        if (handle->is_canceling()) {
          try_cancel();
          const auto cancel_deadline = std::chrono::steady_clock::now() + 5s;
          while (
            result.wait_for(20ms) != std::future_status::ready &&
            std::chrono::steady_clock::now() < cancel_deadline)
          {
          }
          if (result.wait_for(0s) != std::future_status::ready) {
            error = "calibration pose canceled but controller did not reach a terminal state";
            return false;
          }
          (void)result.get();
          controller_terminal = true;
          error = "calibration pose canceled";
          return false;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          try_cancel();
          const auto cancel_deadline = std::chrono::steady_clock::now() + 5s;
          while (
            result.wait_for(20ms) != std::future_status::ready &&
            std::chrono::steady_clock::now() < cancel_deadline)
          {
          }
          if (result.wait_for(0s) != std::future_status::ready) {
            error =
              "calibration trajectory timed out and controller did not reach a terminal state";
            return false;
          }
          (void)result.get();
          controller_terminal = true;
          error = "calibration trajectory timed out";
          return false;
        }
      }
      const auto wrapped = result.get();
      controller_terminal = true;
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
        wrapped.result->error_code != Follow::Result::SUCCESSFUL)
      {
        error = "calibration controller reported failure";
        return false;
      }
      return true;
    } catch (const std::exception & exception) {
      error = "calibration controller transport failed; motion ownership is unconfirmed: " +
        std::string(exception.what());
      return false;
    } catch (...) {
      error = "calibration controller transport failed; motion ownership is unconfirmed";
      return false;
    }
  }

  void execute(const std::shared_ptr<Handle> & handle)
  {
    struct Reset
    {
      std::atomic_bool & value;
      ~Reset() {value.store(false);}
    } reset{busy_};
    const auto request = handle->get_goal();
    auto result = std::make_shared<Move::Result>();
    result->pose_count = count(request->workflow_id);
    result->pose_name = request->workflow_id + "_" + std::to_string(request->pose_index + 1);
    if (handle->is_canceling()) {
      result->error.code = Error::CANCELED;
      result->error.message = "calibration pose canceled before execution";
      handle->canceled(result);
      return;
    }
    if (request->dry_run || !execution_enabled_) {
      result->error.code = request->dry_run ? Error::NONE : Error::UNAVAILABLE;
      result->error.message = request->dry_run ?
        "calibration pose dry-run complete" : "calibration pose execution is disabled";
      if (request->dry_run) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
      return;
    }
    std::string error;
    bool controller_terminal = true;
    if (request->return_ready) {
      result->pose_name = request->workflow_id + "_ready";
      feedback(handle, 0.1F, "returning to shared reference ready pose");
      bool reached = true;
      if (request->workflow_id == Move::Goal::RIGHT_HANDEYE) {
        reached = follow(handle, arm_client_,
            {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
              "right_arm_wrist_flex", "right_arm_wrist_roll"}, arm_ready_, 5.0, error,
            controller_terminal);
      }
      if (reached) {
        reached = follow(handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
          head_ready_, 2.5, error, controller_terminal);
      }
      if (!reached) {
        if (!controller_terminal) {motion_unconfirmed_.store(true);}
        finish_failed(handle, result, error, controller_terminal);
      } else if (handle->is_canceling()) {
        result->error.code = Error::CANCELED;
        handle->canceled(result);
      } else {
        result->error.code = Error::NONE;
        result->error.message = "ready pose reached";
        handle->succeed(result);
      }
      return;
    }
    feedback(handle, 0.1F, "moving to a configured reference calibration pose");
    if (request->workflow_id == Move::Goal::RIGHT_HANDEYE && !follow(
        handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
        {0.0, 0.796136}, 2.5, error, controller_terminal))
    {
      if (!controller_terminal) {
        motion_unconfirmed_.store(true);
      }
      finish_failed(handle, result, error, controller_terminal);
      return;
    }
    const bool reached = request->workflow_id == Move::Goal::HEAD_CAMERA ?
      follow(
      handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
      head_poses_[request->pose_index], 2.5, error, controller_terminal) :
      follow(
      handle, arm_client_,
      {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll"},
      arm_poses_[request->pose_index], 3.0, error, controller_terminal);
    if (!reached) {
      if (!controller_terminal) {
        motion_unconfirmed_.store(true);
      }
      finish_failed(handle, result, error, controller_terminal);
      return;
    }
    if (handle->is_canceling()) {
      result->error.code = Error::CANCELED;
      result->error.message = "calibration pose canceled after controller completion";
      handle->canceled(result);
      return;
    }
    feedback(handle, 1.0F, "calibration pose reached; wait for a stable image");
    result->error.code = Error::NONE;
    result->error.message = "calibration pose reached";
    handle->succeed(result);
  }

  static void finish_failed(
    const std::shared_ptr<Handle> & handle,
    const std::shared_ptr<Move::Result> & result,
    const std::string & message,
    bool controller_terminal)
  {
    const bool safely_canceled = handle->is_canceling() && controller_terminal;
    result->error.code = safely_canceled ? Error::CANCELED : Error::BACKEND_FAILURE;
    result->error.message = message;
    if (safely_canceled) {
      handle->canceled(result);
    } else {
      handle->abort(result);
    }
  }

  bool execution_enabled_{false};
  std::string allowed_workflow_;
  std::vector<std::vector<double>> head_poses_;
  std::vector<std::vector<double>> arm_poses_;
  std::vector<double> arm_ready_;
  std::vector<double> head_ready_;
  std::atomic_bool busy_{false};
  std::atomic_bool motion_unconfirmed_{false};
  rclcpp_action::Client<Follow>::SharedPtr head_client_;
  rclcpp_action::Client<Follow>::SharedPtr arm_client_;
  rclcpp_action::Server<Move>::SharedPtr server_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationPoseServer>());
  rclcpp::shutdown();
  return 0;
}
