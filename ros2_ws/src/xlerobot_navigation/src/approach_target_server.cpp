#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xlerobot_interfaces/action/approach_target.hpp>
#include <xlerobot_interfaces/msg/capability_error.hpp>

#include "xlerobot_navigation/navigation_core.hpp"

namespace xlerobot_navigation
{

namespace
{

using namespace std::chrono_literals;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;
using ApproachTarget = xlerobot_interfaces::action::ApproachTarget;
using BackUp = nav2_msgs::action::BackUp;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using NavigatePose = nav2_msgs::action::NavigateToPose;

struct ApproachFailure : public std::runtime_error
{
  ApproachFailure(
    uint16_t code_value, const std::string & message_value,
    bool canceled_value = false)
  : std::runtime_error(message_value), code(code_value), canceled(canceled_value) {}

  uint16_t code;
  bool canceled;
};

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

class ApproachTargetServer : public rclcpp::Node
{
public:
  using GoalHandleApproach = rclcpp_action::ServerGoalHandle<ApproachTarget>;
  using BackUpGoalHandle = rclcpp_action::ClientGoalHandle<BackUp>;
  using PathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigatePose>;

  ApproachTargetServer()
  : Node("approach_target_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    min_standoff_m_ = declare_parameter<double>("min_standoff_m", 0.4);
    max_standoff_m_ = declare_parameter<double>("max_standoff_m", 1.5);
    fallback_standoffs_ = declare_parameter<std::vector<double>>(
      "fallback_standoff_m", {0.7, 1.0});
    min_target_separation_m_ =
      declare_parameter<double>("min_target_separation_m", 0.05);
    max_target_distance_m_ = declare_parameter<double>("max_target_distance_m", 8.0);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 1.0);
    server_timeout_s_ = declare_parameter<double>("server_timeout_s", 5.0);
    path_timeout_s_ = declare_parameter<double>("path_timeout_s", 8.0);
    nav_attempt_timeout_s_ = declare_parameter<double>("nav_attempt_timeout_s", 30.0);
    backup_distance_m_ = declare_parameter<double>("backup_distance_m", 0.20);
    backup_speed_mps_ = declare_parameter<double>("backup_speed_mps", 0.08);
    backup_timeout_s_ = declare_parameter<double>("backup_timeout_s", 5.0);
    nav_timeout_s_ = declare_parameter<double>("nav_timeout_s", 125.0);
    validate_parameters();

    const auto path_action = declare_parameter<std::string>(
      "path_action", "/compute_path_to_pose");
    const auto nav_action = declare_parameter<std::string>(
      "nav_action", "/navigate_to_pose");
    const auto backup_action = declare_parameter<std::string>(
      "backup_action", "/backup");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    path_client_ = rclcpp_action::create_client<ComputePath>(this, path_action);
    nav_client_ = rclcpp_action::create_client<NavigatePose>(this, nav_action);
    backup_client_ = rclcpp_action::create_client<BackUp>(this, backup_action);
    action_server_ = rclcpp_action::create_server<ApproachTarget>(
      this, "approach_target",
      std::bind(&ApproachTargetServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&ApproachTargetServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ApproachTargetServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "ApproachTarget ready; execution_enabled=%s",
      execution_enabled_ ? "true" : "false");
  }

  ~ApproachTargetServer() override
  {
    shutting_down_ = true;
    cancel_active_child();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void validate_parameters() const
  {
    const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
    if (map_frame_.empty() || base_frame_.empty() || !positive(min_standoff_m_) ||
      !positive(max_standoff_m_) || min_standoff_m_ > max_standoff_m_ ||
      !positive(min_target_separation_m_) || !positive(max_target_distance_m_) ||
      min_target_separation_m_ >= max_target_distance_m_ ||
      !positive(tf_timeout_s_) || !positive(server_timeout_s_) ||
      !positive(path_timeout_s_) || !positive(nav_attempt_timeout_s_) ||
      !positive(backup_distance_m_) || !positive(backup_speed_mps_) ||
      !positive(backup_timeout_s_) || !positive(nav_timeout_s_) ||
      nav_attempt_timeout_s_ > nav_timeout_s_ ||
      fallback_standoffs_.empty())
    {
      throw std::invalid_argument("approach target parameters are invalid");
    }
    if (!std::all_of(
        fallback_standoffs_.begin(), fallback_standoffs_.end(),
        [this](double value) {
          return std::isfinite(value) && value >= min_standoff_m_ && value <= max_standoff_m_;
        }))
    {
      throw std::invalid_argument("fallback standoff exceeds configured bounds");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ApproachTarget::Goal> goal)
  {
    if (goal->target.header.frame_id.empty() || !finite_point(goal->target.point) ||
      !std::isfinite(goal->standoff_m) || goal->standoff_m < min_standoff_m_ ||
      goal->standoff_m > max_standoff_m_)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (goal_reserved_ || shutting_down_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleApproach>)
  {
    cancel_active_child();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleApproach> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<GoalHandleApproach> goal_handle)
  {
    struct ReservationGuard
    {
      ApproachTargetServer * server;
      ~ReservationGuard()
      {
        server->clear_active_child();
        std::lock_guard<std::mutex> lock(server->active_mutex_);
        server->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<ApproachTarget::Result>();
    try {
      const auto & request = *goal_handle->get_goal();
      if (!request.dry_run) {
        require_motion_gate();
      }
      publish_feedback(goal_handle, "transform", 0.10F, "transforming current target");
      const auto target = target_in_map(request.target);
      const auto robot = current_robot_pose();
      const double target_distance = std::hypot(target.x - robot.x, target.y - robot.y);
      RCLCPP_INFO(
        get_logger(),
        "approach input robot=(%.3f, %.3f, %.3f) target=(%.3f, %.3f) distance=%.3fm",
        robot.x, robot.y, robot.yaw, target.x, target.y, target_distance);
      if (target_distance > max_target_distance_m_) {
        throw ApproachFailure(
                CapabilityError::SAFETY_REJECTED,
                "target exceeds the configured approach distance");
      }

      const auto distances = standoff_candidates(request.standoff_m);
      const auto navigation_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(nav_timeout_s_);
      std::string failures;
      bool backup_used = false;
      for (size_t index = 0; index < distances.size(); ) {
        check_parent(goal_handle);
        const double distance = distances[index];
        publish_feedback(
          goal_handle, "plan", 0.20F + 0.15F * static_cast<float>(index) /
          static_cast<float>(distances.size()),
          "planning collision-aware standoff candidate");
        const auto attempt_robot = current_robot_pose();
        const auto candidate = make_standoff_pose(
          target, attempt_robot, distance, min_target_separation_m_);
        auto goal = make_pose(candidate);
        const auto path_failure = validate_path(goal_handle, goal);
        if (!path_failure.empty()) {
          failures += " standoff=" + std::to_string(distance) + " " + path_failure + ";";
          ++index;
          continue;
        }
        RCLCPP_INFO(
          get_logger(), "selected standoff candidate=(%.3f, %.3f) distance=%.3fm",
          goal.pose.position.x, goal.pose.position.y, distance);
        result->reached_pose = goal;
        if (request.dry_run) {
          result->error.code = CapabilityError::NONE;
          result->error.message = "dry-run standoff path valid; no navigation goal sent";
          publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
          goal_handle->succeed(result);
          return;
        }

        const double remaining_s = std::chrono::duration<double>(
          navigation_deadline - std::chrono::steady_clock::now()).count();
        if (remaining_s <= 0.0) {
          break;
        }
        require_motion_gate();
        publish_feedback(
          goal_handle, "navigate", 0.50F + 0.35F * static_cast<float>(index) /
          static_cast<float>(distances.size()),
          "navigating to person standoff candidate");
        try {
          navigate(goal_handle, goal, std::min(nav_attempt_timeout_s_, remaining_s));
          if (const auto reached = try_current_robot_pose()) {
            result->reached_pose = make_pose(*reached);
          }
          result->error.code = CapabilityError::NONE;
          result->error.message = "arrived at target standoff";
          publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
          goal_handle->succeed(result);
          return;
        } catch (const ApproachFailure & failure) {
          if (failure.canceled || failure.code == CapabilityError::CANCELED ||
            failure.code == CapabilityError::SAFETY_REJECTED ||
            failure.code == CapabilityError::UNAVAILABLE)
          {
            throw;
          }
          failures += " standoff=" + std::to_string(distance) + " " + failure.what() + ";";
          RCLCPP_WARN(
            get_logger(), "standoff %.3fm failed at runtime; trying next candidate: %s",
            distance, failure.what());
          if (!backup_used) {
            backup_used = true;
            publish_feedback(
              goal_handle, "recover", 0.45F,
              "backing up once before replanning the requested standoff");
            if (try_backup(goal_handle)) {
              RCLCPP_INFO(
                get_logger(),
                "backup complete; refreshing robot pose and retrying standoff %.3fm",
                distance);
              continue;
            } else {
              RCLCPP_WARN(
                get_logger(), "backup recovery unavailable; continuing with farther standoff");
            }
          }
          ++index;
        }
      }
      throw ApproachFailure(
              CapabilityError::BACKEND_FAILURE,
              "no standoff candidate completed:" + failures);
    } catch (const ApproachFailure & failure) {
      result->error.code = failure.code;
      result->error.message = failure.what();
      if (failure.canceled || goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      result->error.code = CapabilityError::INTERNAL_ERROR;
      result->error.message = error.what();
      goal_handle->abort(result);
    }
  }

  Pose2D target_in_map(const geometry_msgs::msg::PointStamped & target)
  {
    geometry_msgs::msg::PointStamped transformed = target;
    if (target.header.frame_id != map_frame_) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          map_frame_, target.header.frame_id, rclcpp::Time(target.header.stamp),
          tf2::durationFromSec(tf_timeout_s_));
        tf2::doTransform(target, transformed, transform);
      } catch (const std::exception & error) {
        throw ApproachFailure(
                CapabilityError::UNAVAILABLE,
                "target transform failed: " + std::string(error.what()));
      }
    }
    return {transformed.point.x, transformed.point.y, 0.0};
  }

  Pose2D current_robot_pose()
  {
    const auto pose = try_current_robot_pose();
    if (!pose) {
      throw ApproachFailure(
              CapabilityError::UNAVAILABLE, "map-to-base TF unavailable for approach");
    }
    return *pose;
  }

  std::optional<Pose2D> try_current_robot_pose()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_s_));
      const auto & translation = transform.transform.translation;
      tf2::Quaternion orientation;
      tf2::fromMsg(transform.transform.rotation, orientation);
      return Pose2D{translation.x, translation.y, tf2::getYaw(orientation)};
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::vector<double> standoff_candidates(double requested_standoff) const
  {
    std::vector<double> distances{requested_standoff};
    for (double distance : fallback_standoffs_) {
      if (std::none_of(
          distances.begin(), distances.end(),
          [distance](double existing) {return std::abs(existing - distance) <= 1.0e-6;}))
      {
        distances.push_back(distance);
      }
    }
    return distances;
  }

  std::string validate_path(
    const std::shared_ptr<GoalHandleApproach> & outer,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!wait_for_action<ComputePath>(path_client_, server_timeout_s_, outer)) {
      return "compute_path_to_pose unavailable";
    }
    ComputePath::Goal goal;
    goal.goal = pose;
    goal.use_start = false;
    auto send_future = path_client_->async_send_goal(goal);
    if (!wait_future(send_future, server_timeout_s_, outer, false)) {
      return "path goal response timed out";
    }
    const auto handle = send_future.get();
    if (!handle) {
      return "path goal rejected";
    }
    set_active_child([this, handle]() {path_client_->async_cancel_goal(handle);});
    auto result_future = path_client_->async_get_result(handle);
    if (!wait_future(result_future, path_timeout_s_, outer, false)) {
      clear_active_child();
      return "path result timed out";
    }
    clear_active_child();
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != ComputePath::Result::NONE ||
      wrapped.result->path.poses.empty())
    {
      return "planner found no valid path";
    }
    return "";
  }

  void navigate(
    const std::shared_ptr<GoalHandleApproach> & outer,
    const geometry_msgs::msg::PoseStamped & pose,
    double timeout_s)
  {
    if (!wait_for_action<NavigatePose>(nav_client_, server_timeout_s_, outer)) {
      throw ApproachFailure(CapabilityError::UNAVAILABLE, "navigate_to_pose unavailable");
    }
    require_motion_gate();
    NavigatePose::Goal goal;
    goal.pose = pose;
    auto send_future = nav_client_->async_send_goal(goal);
    const auto acceptance_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(server_timeout_s_);
    while (rclcpp::ok() && !shutting_down_ &&
      send_future.wait_for(20ms) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() >= acceptance_deadline) {
        throw ApproachFailure(CapabilityError::TIMEOUT, "Nav2 goal acceptance timed out");
      }
    }
    if (!rclcpp::ok() || shutting_down_) {
      throw ApproachFailure(CapabilityError::CANCELED, "approach interrupted", true);
    }
    const auto handle = send_future.get();
    if (!handle) {
      throw ApproachFailure(CapabilityError::BACKEND_FAILURE, "Nav2 rejected standoff goal");
    }
    set_active_child([this, handle]() {nav_client_->async_cancel_goal(handle);});
    if (outer->is_canceling()) {
      cancel_active_child();
      throw ApproachFailure(CapabilityError::CANCELED, "approach canceled", true);
    }
    require_motion_gate_or_cancel();
    auto result_future = nav_client_->async_get_result(handle);
    if (!wait_future(result_future, timeout_s, outer, true)) {
      auto cancel_future = nav_client_->async_cancel_goal(handle);
      const auto stop_deadline = std::chrono::steady_clock::now() + 2s;
      while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_deadline &&
        cancel_future.wait_for(20ms) != std::future_status::ready)
      {
        check_parent(outer);
      }
      while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_deadline &&
        result_future.wait_for(20ms) != std::future_status::ready)
      {
        check_parent(outer);
      }
      const bool nav_stopped = result_future.wait_for(0ms) == std::future_status::ready;
      clear_active_child();
      if (!nav_stopped) {
        throw ApproachFailure(
                CapabilityError::UNAVAILABLE,
                "Nav2 standoff attempt did not confirm stop after cancellation");
      }
      throw ApproachFailure(CapabilityError::TIMEOUT, "Nav2 standoff attempt timed out");
    }
    clear_active_child();
    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      throw ApproachFailure(CapabilityError::CANCELED, "Nav2 approach canceled", true);
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != NavigatePose::Result::NONE)
    {
      throw ApproachFailure(CapabilityError::BACKEND_FAILURE, "Nav2 approach failed");
    }
  }

  bool try_backup(const std::shared_ptr<GoalHandleApproach> & outer)
  {
    if (!wait_for_action<BackUp>(backup_client_, server_timeout_s_, outer)) {
      return false;
    }
    require_motion_gate();
    BackUp::Goal goal;
    goal.target.x = -backup_distance_m_;
    goal.speed = static_cast<float>(backup_speed_mps_);
    goal.time_allowance = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(backup_timeout_s_));
    auto send_future = backup_client_->async_send_goal(goal);
    if (!wait_future(send_future, server_timeout_s_, outer, true)) {
      return false;
    }
    const auto handle = send_future.get();
    if (!handle) {
      return false;
    }
    set_active_child([this, handle]() {backup_client_->async_cancel_goal(handle);});
    auto result_future = backup_client_->async_get_result(handle);
    if (!wait_future(result_future, backup_timeout_s_ + 1.0, outer, true)) {
      clear_active_child();
      return false;
    }
    clear_active_child();
    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED && outer->is_canceling()) {
      throw ApproachFailure(CapabilityError::CANCELED, "approach canceled", true);
    }
    return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
           wrapped.result->error_code == BackUp::Result::NONE;
  }

  geometry_msgs::msg::PoseStamped make_pose(const Pose2D & pose)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, pose.yaw);
    message.pose.orientation = tf2::toMsg(orientation);
    return message;
  }

  template<typename ActionT>
  bool wait_for_action(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    double timeout_s,
    const std::shared_ptr<GoalHandleApproach> & outer)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(outer);
      if (client->wait_for_action_server(50ms)) {
        return true;
      }
    }
    return false;
  }

  template<typename FutureT>
  bool wait_future(
    FutureT & future,
    double timeout_s,
    const std::shared_ptr<GoalHandleApproach> & outer,
    bool require_gate)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !shutting_down_ &&
      future.wait_for(20ms) != std::future_status::ready)
    {
      check_parent(outer);
      if (require_gate) {
        require_motion_gate_or_cancel();
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        cancel_active_child();
        return false;
      }
    }
    return rclcpp::ok() && !shutting_down_;
  }

  void check_parent(const std::shared_ptr<GoalHandleApproach> & outer)
  {
    if (outer->is_canceling()) {
      cancel_active_child();
      throw ApproachFailure(CapabilityError::CANCELED, "approach canceled", true);
    }
  }

  void require_motion_gate() const
  {
    if (!execution_enabled_) {
      throw ApproachFailure(
              CapabilityError::SAFETY_REJECTED, "execution_enabled is false");
    }
  }

  void require_motion_gate_or_cancel()
  {
    try {
      require_motion_gate();
    } catch (...) {
      cancel_active_child();
      throw;
    }
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleApproach> & goal_handle,
    const std::string & phase,
    float progress,
    const std::string & message)
  {
    auto feedback = std::make_shared<ApproachTarget::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = progress;
    feedback->state.message = message;
    goal_handle->publish_feedback(feedback);
  }

  void set_active_child(std::function<void()> cancel)
  {
    std::lock_guard<std::mutex> lock(child_mutex_);
    cancel_active_ = std::move(cancel);
  }

  void clear_active_child()
  {
    std::lock_guard<std::mutex> lock(child_mutex_);
    cancel_active_ = nullptr;
  }

  void cancel_active_child()
  {
    std::function<void()> cancel;
    {
      std::lock_guard<std::mutex> lock(child_mutex_);
      cancel = cancel_active_;
    }
    if (cancel) {
      cancel();
    }
  }

  bool execution_enabled_{false};
  std::string map_frame_;
  std::string base_frame_;
  double min_standoff_m_{0.4};
  double max_standoff_m_{1.5};
  std::vector<double> fallback_standoffs_;
  double min_target_separation_m_{0.05};
  double max_target_distance_m_{8.0};
  double tf_timeout_s_{1.0};
  double server_timeout_s_{5.0};
  double path_timeout_s_{8.0};
  double nav_attempt_timeout_s_{30.0};
  double backup_distance_m_{0.20};
  double backup_speed_mps_{0.08};
  double backup_timeout_s_{5.0};
  double nav_timeout_s_{125.0};
  std::mutex active_mutex_;
  bool goal_reserved_{false};
  std::mutex child_mutex_;
  std::function<void()> cancel_active_;
  std::atomic<bool> shutting_down_{false};
  std::thread worker_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<ComputePath>::SharedPtr path_client_;
  rclcpp_action::Client<NavigatePose>::SharedPtr nav_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Server<ApproachTarget>::SharedPtr action_server_;
};

}  // namespace xlerobot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xlerobot_navigation::ApproachTargetServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
