#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xlerobot_interfaces/action/navigate_to_named_place.hpp>
#include <xlerobot_interfaces/msg/capability_error.hpp>

#include "xlerobot_navigation/navigation_core.hpp"

namespace xlerobot_navigation
{

namespace
{

using namespace std::chrono_literals;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;
using NavigateNamed = xlerobot_interfaces::action::NavigateToNamedPlace;
using BackUp = nav2_msgs::action::BackUp;
using NavigatePose = nav2_msgs::action::NavigateToPose;
using Spin = nav2_msgs::action::Spin;

struct StageResult
{
  bool success{false};
  bool canceled{false};
  uint16_t error_code{CapabilityError::BACKEND_FAILURE};
  std::string message;
  bool recoverable{false};
};

}  // namespace

class NamedNavigationServer : public rclcpp::Node
{
public:
  using GoalHandleNamed = rclcpp_action::ServerGoalHandle<NavigateNamed>;
  using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigatePose>;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  NamedNavigationServer()
  : Node("named_navigation_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    dock_cmd_vel_topic_ =
      declare_parameter<std::string>("dock_cmd_vel_topic", "cmd_vel_dock");
    server_timeout_s_ = declare_parameter<double>("server_timeout_s", 8.0);
    nav_timeout_s_ = declare_parameter<double>("nav_timeout_s", 120.0);
    nav_attempt_timeout_s_ = declare_parameter<double>("nav_attempt_timeout_s", 35.0);
    nav_cancel_timeout_s_ = declare_parameter<double>("nav_cancel_timeout_s", 2.0);
    backup_distance_m_ = declare_parameter<double>("backup_distance_m", 0.20);
    backup_speed_mps_ = declare_parameter<double>("backup_speed_mps", 0.08);
    backup_timeout_s_ = declare_parameter<double>("backup_timeout_s", 5.0);
    already_reached_position_tolerance_m_ =
      declare_parameter<double>("already_reached_position_tolerance_m", 0.08);
    already_reached_yaw_tolerance_rad_ =
      declare_parameter<double>("already_reached_yaw_tolerance_rad", 0.08);
    spin_timeout_s_ = declare_parameter<double>("spin_timeout_s", 15.0);
    spin_threshold_rad_ = declare_parameter<double>("spin_threshold_rad", 0.15);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 1.0);
    dock_control_rate_hz_ = declare_parameter<double>("dock_control_rate_hz", 20.0);
    dock_timeout_s_ = declare_parameter<double>("dock_timeout_s", 28.0);
    max_initial_distance_m_ = declare_parameter<double>("max_initial_distance_m", 0.60);
    stop_publish_count_ = declare_parameter<int>("stop_publish_count", 5);
    validate_runtime_parameters();

    DockConfig dock_config;
    dock_config.position_tolerance_m =
      declare_parameter<double>("dock.position_tolerance_m", 0.010);
    dock_config.lateral_tolerance_m =
      declare_parameter<double>("dock.lateral_tolerance_m", 0.08);
    dock_lateral_tolerance_m_ = dock_config.lateral_tolerance_m;
    dock_config.yaw_tolerance_rad =
      declare_parameter<double>("dock.yaw_tolerance_rad", 0.025);
    dock_config.max_linear_speed_mps =
      declare_parameter<double>("dock.max_linear_speed_mps", 0.035);
    dock_config.max_angular_speed_radps =
      declare_parameter<double>("dock.max_angular_speed_radps", 0.16);
    dock_config.linear_kp = declare_parameter<double>("dock.linear_kp", 0.45);
    dock_config.angular_kp = declare_parameter<double>("dock.angular_kp", 0.9);
    dock_config.angular_deadband_rad =
      declare_parameter<double>("dock.angular_deadband_rad", 0.006);
    dock_config.yaw_align_release_rad =
      declare_parameter<double>("dock.yaw_align_release_rad", 0.018);
    dock_config.yaw_align_stable_s =
      declare_parameter<double>("dock.yaw_align_stable_s", 0.35);
    dock_config.yaw_realign_threshold_rad =
      declare_parameter<double>("dock.yaw_realign_threshold_rad", 0.025);
    dock_config.angular_sign_change_deadband_rad =
      declare_parameter<double>("dock.angular_sign_change_deadband_rad", 0.04);
    dock_config.max_angular_accel_radps2 =
      declare_parameter<double>("dock.max_angular_accel_radps2", 0.28);
    dock_controller_ = std::make_unique<DockController>(dock_config);
    load_places();

    dock_publisher_ = create_publisher<geometry_msgs::msg::Twist>(dock_cmd_vel_topic_, 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_client_ = rclcpp_action::create_client<NavigatePose>(this, "navigate_to_pose");
    backup_client_ = rclcpp_action::create_client<BackUp>(this, "/backup");
    spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");
    action_server_ = rclcpp_action::create_server<NavigateNamed>(
      this,
      "navigate_to_named_place",
      std::bind(&NamedNavigationServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&NamedNavigationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NamedNavigationServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Named navigation ready with %zu places; execution_enabled=%s",
      places_.size(), execution_enabled_ ? "true" : "false");
  }

  ~NamedNavigationServer() override
  {
    shutting_down_ = true;
    publish_stop();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void validate_runtime_parameters() const
  {
    if (server_timeout_s_ <= 0.0 || nav_timeout_s_ <= 0.0 ||
      nav_attempt_timeout_s_ <= 0.0 || nav_attempt_timeout_s_ > nav_timeout_s_ ||
      nav_cancel_timeout_s_ <= 0.0 || backup_distance_m_ <= 0.0 ||
      backup_speed_mps_ <= 0.0 || backup_timeout_s_ <= 0.0 ||
      already_reached_position_tolerance_m_ <= 0.0 ||
      already_reached_yaw_tolerance_rad_ <= 0.0 || spin_timeout_s_ <= 0.0 ||
      spin_threshold_rad_ < 0.0 || tf_timeout_s_ <= 0.0 || dock_control_rate_hz_ <= 0.0 ||
      dock_timeout_s_ <= 0.0 || max_initial_distance_m_ <= 0.0 || stop_publish_count_ <= 0)
    {
      throw std::invalid_argument("navigation timeouts, rates, and limits are invalid");
    }
  }

  void load_places()
  {
    const auto ids = declare_parameter<std::vector<std::string>>(
      "place_ids", std::vector<std::string>{"table"});
    for (const auto & id : ids) {
      if (id.empty() || places_.count(id) != 0U) {
        throw std::invalid_argument("place_ids must be non-empty and unique");
      }
      const std::string prefix = "places." + id + ".";
      NamedPlace place;
      place.id = id;
      place.frame_id = declare_parameter<std::string>(prefix + "frame_id", map_frame_);
      place.target.x = declare_parameter<double>(prefix + "x", 0.0);
      place.target.y = declare_parameter<double>(prefix + "y", 0.0);
      place.target.yaw = declare_parameter<double>(prefix + "yaw", 0.0);
      place.nav_offset_m = declare_parameter<double>(prefix + "nav_offset_m", 0.0);
      place.dock = declare_parameter<bool>(prefix + "dock", false);
      static_cast<void>(make_navigation_plan(place));
      places_.emplace(id, std::move(place));
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateNamed::Goal>)
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (goal_reserved_) {
      RCLCPP_WARN(get_logger(), "Rejecting concurrent named navigation goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNamed>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNamed> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<GoalHandleNamed> goal_handle)
  {
    struct ReservationGuard
    {
      NamedNavigationServer * server;
      ~ReservationGuard()
      {
        std::lock_guard<std::mutex> lock(server->active_mutex_);
        server->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<NavigateNamed::Result>();
    const std::string place_id = goal_handle->get_goal()->place_id;
    const auto place_item = places_.find(place_id);
    if (place_id.empty() || place_item == places_.end()) {
      abort(goal_handle, result, CapabilityError::INVALID_GOAL, "resolve",
        "unknown named place: " + place_id);
      return;
    }

    const auto plan = make_navigation_plan(place_item->second);
    result->reached_pose = make_pose(plan.place.frame_id, plan.place.target);
    publish_feedback(goal_handle, "resolve", 0.05F, "resolved " + place_id);
    if (goal_handle->get_goal()->dry_run) {
      publish_feedback(goal_handle, "plan_nav", 0.25F, "planned position-only Nav2 predock");
      if (plan.place.dock) {
        publish_feedback(goal_handle, "plan_spin", 0.50F, "planned coarse Spin alignment");
        publish_feedback(goal_handle, "plan_dock", 0.75F, "planned precise local docking");
      }
      result->error.code = CapabilityError::NONE;
      result->error.message = "dry-run navigation plan valid";
      goal_handle->succeed(result);
      return;
    }
    if (!execution_enabled_) {
      abort(goal_handle, result, CapabilityError::SAFETY_REJECTED, "safety_gate",
        "non-dry-run navigation rejected because execution_enabled is false");
      return;
    }

    if (goal_handle->get_goal()->skip_if_already_reached) {
      const auto current = lookup_pose(plan.place.frame_id);
      if (current && pose_already_reached(*current, plan.place.target)) {
        result->reached_pose = make_pose(plan.place.frame_id, *current);
        result->error.code = CapabilityError::NONE;
        result->error.message = "trusted localization confirms robot already at named place";
        publish_feedback(goal_handle, "already_reached", 1.0F, result->error.message);
        goal_handle->succeed(result);
        return;
      }
    }

    publish_feedback(goal_handle, "navigate", 0.15F, "sending position-only Nav2 goal");
    const auto nav_result = navigate_to_predock(goal_handle, plan);
    if (!finish_stage_or_continue(goal_handle, result, nav_result, "navigate")) {
      return;
    }

    if (plan.place.dock) {
      publish_feedback(goal_handle, "spin", 0.50F, "coarse yaw alignment");
      const auto spin_result = spin_to_target(goal_handle, plan);
      if (!finish_stage_or_continue(goal_handle, result, spin_result, "spin")) {
        return;
      }
      const auto lateral_result = correct_lateral_error_after_spin(
        goal_handle, plan, now());
      if (!finish_stage_or_continue(
          goal_handle, result, lateral_result, "predock_correction"))
      {
        return;
      }
      publish_feedback(goal_handle, "dock", 0.70F, "precise low-speed docking");
      const auto dock_result = dock_to_target(goal_handle, plan);
      if (!finish_stage_or_continue(goal_handle, result, dock_result, "dock")) {
        return;
      }
    }

    if (const auto reached = lookup_pose(plan.place.frame_id)) {
      result->reached_pose = make_pose(plan.place.frame_id, *reached);
    }
    result->error.code = CapabilityError::NONE;
    result->error.message = "arrived at named place";
    publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
    goal_handle->succeed(result);
  }

  StageResult navigate_to_predock(
    const std::shared_ptr<GoalHandleNamed> & outer, const NavigationPlan & plan)
  {
    const auto overall_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(nav_timeout_s_);
    StageResult latest;
    for (int attempt = 0; attempt < 2; ++attempt) {
      const double remaining_s = std::chrono::duration<double>(
        overall_deadline - std::chrono::steady_clock::now()).count();
      if (remaining_s <= 0.0) {
        return {false, false, CapabilityError::TIMEOUT, "Nav2 predock timed out"};
      }
      latest = navigate_to_predock_once(
        outer, plan, std::min(nav_attempt_timeout_s_, remaining_s));
      if (latest.success || latest.canceled || !latest.recoverable) {
        return latest;
      }
      if (const auto current = lookup_pose(plan.place.frame_id)) {
        const double target_error_m = std::hypot(
          plan.place.target.x - current->x, plan.place.target.y - current->y);
        if (target_error_m <= already_reached_position_tolerance_m_) {
          return {
            true, false, CapabilityError::NONE,
            "predock stopped but current position is already within table tolerance"};
        }
      }
      if (attempt > 0) {
        return latest;
      }
      publish_feedback(
        outer, "navigate_recover", 0.32F,
        "Nav2 recovery did not finish; backing up once before a fresh plan");
      const auto backup = try_backup(outer);
      if (!backup.success) {
        if (backup.canceled) {
          return backup;
        }
        return {
          false, false, backup.error_code,
          latest.message + "; predock backup recovery failed: " + backup.message};
      }
      publish_feedback(
        outer, "navigate_retry", 0.36F,
        "backup complete; resubmitting predock goal for a fresh plan");
    }
    return latest;
  }

  StageResult navigate_to_predock_once(
    const std::shared_ptr<GoalHandleNamed> & outer, const NavigationPlan & plan,
    double timeout_s)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_s_))) {
      return {false, false, CapabilityError::UNAVAILABLE, "navigate_to_pose unavailable"};
    }
    NavigatePose::Goal nav_goal;
    nav_goal.pose = make_pose(plan.place.frame_id, plan.predock);
    auto send_future = nav_client_->async_send_goal(nav_goal);
    if (!wait_future(send_future, server_timeout_s_, outer)) {
      return outer->is_canceling() ?
             StageResult{false, true, CapabilityError::CANCELED, "canceled before Nav2 accepted"} :
             StageResult{false, false, CapabilityError::TIMEOUT, "Nav2 goal acceptance timed out"};
    }
    const auto nav_handle = send_future.get();
    if (!nav_handle) {
      return {false, false, CapabilityError::BACKEND_FAILURE, "Nav2 rejected predock goal"};
    }
    auto result_future = nav_client_->async_get_result(nav_handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (result_future.wait_for(50ms) != std::future_status::ready) {
      if (shutting_down_ || !rclcpp::ok()) {
        const bool stopped = cancel_nav_and_wait(nav_handle, result_future);
        return stopped ?
               StageResult{false, true, CapabilityError::CANCELED,
                 "navigation server shutting down"} :
               StageResult{false, false, CapabilityError::UNAVAILABLE,
                 "Nav2 predock did not confirm stop during shutdown"};
      }
      if (outer->is_canceling()) {
        const bool stopped = cancel_nav_and_wait(nav_handle, result_future);
        return stopped ?
               StageResult{false, true, CapabilityError::CANCELED, "Nav2 goal canceled"} :
               StageResult{false, false, CapabilityError::UNAVAILABLE,
                 "Nav2 predock did not confirm stop after cancellation"};
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        if (!cancel_nav_and_wait(nav_handle, result_future)) {
          return {false, false, CapabilityError::UNAVAILABLE,
            "Nav2 predock did not confirm stop after timeout"};
        }
        return {false, false, CapabilityError::TIMEOUT, "Nav2 predock attempt timed out", true};
      }
      publish_feedback(outer, "navigate", 0.30F, "Nav2 predock active");
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != NavigatePose::Result::NONE)
    {
      return {false, false, CapabilityError::BACKEND_FAILURE, "Nav2 predock failed", true};
    }
    return {true, false, CapabilityError::NONE, "Nav2 predock complete"};
  }

  template<typename ResultFutureT>
  bool cancel_nav_and_wait(
    const GoalHandleNavigate::SharedPtr & handle, ResultFutureT & result_future)
  {
    auto cancel_future = nav_client_->async_cancel_goal(handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(nav_cancel_timeout_s_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
      cancel_future.wait_for(20ms) != std::future_status::ready)
    {
    }
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
      result_future.wait_for(20ms) != std::future_status::ready)
    {
    }
    return result_future.wait_for(0ms) == std::future_status::ready;
  }

  StageResult try_backup(const std::shared_ptr<GoalHandleNamed> & outer)
  {
    if (!backup_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_s_))) {
      return {false, false, CapabilityError::UNAVAILABLE, "Nav2 BackUp unavailable"};
    }
    BackUp::Goal goal;
    goal.target.x = backup_distance_m_;
    goal.speed = static_cast<float>(backup_speed_mps_);
    goal.time_allowance = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(backup_timeout_s_));
    auto send_future = backup_client_->async_send_goal(goal);
    if (!wait_future(send_future, server_timeout_s_, outer)) {
      return outer->is_canceling() ?
             StageResult{false, true, CapabilityError::CANCELED,
               "predock recovery canceled before BackUp accepted"} :
             StageResult{false, false, CapabilityError::TIMEOUT,
               "Nav2 BackUp acceptance timed out"};
    }
    const auto handle = send_future.get();
    if (!handle) {
      return {false, false, CapabilityError::BACKEND_FAILURE, "Nav2 rejected BackUp"};
    }
    auto result_future = backup_client_->async_get_result(handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(backup_timeout_s_ + 1.0);
    while (result_future.wait_for(20ms) != std::future_status::ready) {
      if (outer->is_canceling() || shutting_down_ || !rclcpp::ok()) {
        static_cast<void>(backup_client_->async_cancel_goal(handle));
        return {false, true, CapabilityError::CANCELED, "predock BackUp canceled"};
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        static_cast<void>(backup_client_->async_cancel_goal(handle));
        return {false, false, CapabilityError::TIMEOUT, "Nav2 BackUp timed out"};
      }
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != BackUp::Result::NONE)
    {
      const uint16_t nav2_code = wrapped.result ? wrapped.result->error_code : 0U;
      const std::string detail = wrapped.result && !wrapped.result->error_msg.empty() ?
        ": " + wrapped.result->error_msg : "";
      return {
        false, false,
        nav2_code == BackUp::Result::COLLISION_AHEAD ?
        CapabilityError::SAFETY_REJECTED : CapabilityError::BACKEND_FAILURE,
        "Nav2 BackUp failed (error_code=" + std::to_string(nav2_code) + ")" + detail,
        nav2_code == BackUp::Result::COLLISION_AHEAD};
    }
    return {true, false, CapabilityError::NONE, "Nav2 BackUp complete"};
  }

  StageResult spin_to_target(
    const std::shared_ptr<GoalHandleNamed> & outer, const NavigationPlan & plan)
  {
    const auto current = lookup_pose(plan.place.frame_id);
    if (!current) {
      return {false, false, CapabilityError::UNAVAILABLE, "map-to-base TF unavailable for spin"};
    }
    const double yaw_delta = normalize_angle(plan.spin_target_yaw - current->yaw);
    if (std::abs(yaw_delta) <= spin_threshold_rad_) {
      return {true, false, CapabilityError::NONE, "coarse spin already within threshold"};
    }
    if (!spin_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_s_))) {
      return {false, false, CapabilityError::UNAVAILABLE, "spin action unavailable"};
    }
    Spin::Goal spin_goal;
    spin_goal.target_yaw = yaw_delta;
    spin_goal.time_allowance.sec = static_cast<int32_t>(spin_timeout_s_);
    spin_goal.time_allowance.nanosec = static_cast<uint32_t>(
      (spin_timeout_s_ - std::floor(spin_timeout_s_)) * 1.0e9);
    auto send_future = spin_client_->async_send_goal(spin_goal);
    if (!wait_future(send_future, server_timeout_s_, outer)) {
      return outer->is_canceling() ?
             StageResult{false, true, CapabilityError::CANCELED, "canceled before Spin accepted"} :
             StageResult{false, false, CapabilityError::TIMEOUT, "Spin acceptance timed out"};
    }
    const auto spin_handle = send_future.get();
    if (!spin_handle) {
      return {false, false, CapabilityError::BACKEND_FAILURE, "Spin goal rejected"};
    }
    auto result_future = spin_client_->async_get_result(spin_handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(spin_timeout_s_ + 1.0);
    while (result_future.wait_for(50ms) != std::future_status::ready) {
      if (shutting_down_ || !rclcpp::ok()) {
        static_cast<void>(spin_client_->async_cancel_goal(spin_handle));
        return {false, true, CapabilityError::CANCELED, "navigation server shutting down"};
      }
      if (outer->is_canceling()) {
        static_cast<void>(spin_client_->async_cancel_goal(spin_handle));
        return {false, true, CapabilityError::CANCELED, "Spin canceled"};
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        static_cast<void>(spin_client_->async_cancel_goal(spin_handle));
        return {false, false, CapabilityError::TIMEOUT, "Spin timed out"};
      }
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != Spin::Result::NONE)
    {
      return {false, false, CapabilityError::BACKEND_FAILURE, "coarse Spin failed"};
    }
    return {true, false, CapabilityError::NONE, "coarse Spin complete"};
  }

  StageResult correct_lateral_error_after_spin(
    const std::shared_ptr<GoalHandleNamed> & outer, const NavigationPlan & plan,
    const rclcpp::Time & spin_completed_at)
  {
    auto current = wait_for_pose_after(plan.place.frame_id, spin_completed_at, outer);
    if (!current) {
      return {
        false, false, CapabilityError::UNAVAILABLE,
        "no fresh map-to-base TF arrived after Spin"};
    }
    double lateral_error_m = target_frame_lateral_error(*current, plan.place.target);
    if (std::abs(lateral_error_m) <= dock_lateral_tolerance_m_) {
      return {
        true, false, CapabilityError::NONE,
        "post-Spin lateral error is within docking tolerance"};
    }

    publish_feedback(
      outer, "predock_correction", 0.58F,
      "post-Spin lateral error " + std::to_string(lateral_error_m) +
      " m exceeds tolerance; correcting predock position once");
    auto correction = navigate_to_predock_once(outer, plan, nav_attempt_timeout_s_);
    if (!correction.success) {
      correction.message = "post-Spin predock correction failed: " + correction.message;
      return correction;
    }

    const auto correction_completed_at = now();
    current = wait_for_pose_after(plan.place.frame_id, correction_completed_at, outer);
    if (!current) {
      return {
        false, false, CapabilityError::UNAVAILABLE,
        "no fresh map-to-base TF arrived after predock correction"};
    }

    publish_feedback(
      outer, "spin_correction", 0.64F,
      "predock position corrected; restoring table heading");
    auto spin = spin_to_target(outer, plan);
    if (!spin.success) {
      spin.message = "post-correction Spin failed: " + spin.message;
      return spin;
    }

    current = wait_for_pose_after(plan.place.frame_id, now(), outer);
    if (!current) {
      return {
        false, false, CapabilityError::UNAVAILABLE,
        "no fresh map-to-base TF arrived after correction Spin"};
    }
    lateral_error_m = target_frame_lateral_error(*current, plan.place.target);
    if (std::abs(lateral_error_m) > dock_lateral_tolerance_m_) {
      return {
        false, false, CapabilityError::SAFETY_REJECTED,
        "lateral error remains " + std::to_string(lateral_error_m) +
        " m after the single predock correction"};
    }
    return {
      true, false, CapabilityError::NONE,
      "predock correction restored lateral docking tolerance"};
  }

  StageResult dock_to_target(
    const std::shared_ptr<GoalHandleNamed> & outer, const NavigationPlan & plan)
  {
    auto current = lookup_pose(plan.place.frame_id);
    if (!current) {
      return {false, false, CapabilityError::UNAVAILABLE, "map-to-base TF unavailable for dock"};
    }
    if (std::hypot(plan.place.target.x - current->x, plan.place.target.y - current->y) >
      max_initial_distance_m_)
    {
      return {false, false, CapabilityError::SAFETY_REJECTED,
        "dock target exceeds initial distance gate"};
    }

    dock_controller_->reset();
    const double period_s = 1.0 / dock_control_rate_hz_;
    const auto period = std::chrono::duration<double>(period_s);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(dock_timeout_s_);
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      if (outer->is_canceling()) {
        publish_stop();
        return {false, true, CapabilityError::CANCELED, "dock canceled"};
      }
      current = lookup_pose(plan.place.frame_id);
      if (!current) {
        publish_stop();
        return {false, false, CapabilityError::UNAVAILABLE, "lost map-to-base TF while docking"};
      }
      const auto step = dock_controller_->update(*current, plan.place.target, period_s);
      publish_dock_feedback(outer, step);
      if (step.phase == DockPhase::kComplete) {
        publish_stop();
        return {true, false, CapabilityError::NONE, "precise docking complete"};
      }
      if (step.phase == DockPhase::kFailed) {
        publish_stop();
        return {false, false, CapabilityError::SAFETY_REJECTED, step.message};
      }
      geometry_msgs::msg::Twist command;
      command.linear.x = step.linear_x;
      command.angular.z = step.angular_z;
      dock_publisher_->publish(command);
      std::this_thread::sleep_for(period);
    }
    publish_stop();
    return {false, false, CapabilityError::TIMEOUT, "precise docking timed out"};
  }

  template<typename FutureT>
  bool wait_future(
    FutureT & future, double timeout_s, const std::shared_ptr<GoalHandleNamed> & outer) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (future.wait_for(20ms) != std::future_status::ready) {
      if (shutting_down_ || !rclcpp::ok() || outer->is_canceling() ||
        std::chrono::steady_clock::now() >= deadline)
      {
        return false;
      }
    }
    return true;
  }

  bool finish_stage_or_continue(
    const std::shared_ptr<GoalHandleNamed> & goal_handle,
    const std::shared_ptr<NavigateNamed::Result> & result,
    const StageResult & stage,
    const std::string & phase)
  {
    if (stage.success) {
      return true;
    }
    if (stage.canceled) {
      result->error.code = CapabilityError::CANCELED;
      result->error.message = phase + ": " + stage.message;
      goal_handle->canceled(result);
    } else {
      abort(goal_handle, result, stage.error_code, phase, stage.message);
    }
    return false;
  }

  bool pose_already_reached(const Pose2D & current, const Pose2D & target) const
  {
    return std::hypot(target.x - current.x, target.y - current.y) <=
           already_reached_position_tolerance_m_ &&
           std::abs(normalize_angle(target.yaw - current.yaw)) <=
           already_reached_yaw_tolerance_rad_;
  }

  std::optional<Pose2D> lookup_pose(const std::string & frame) const
  {
    const auto sample = lookup_pose_sample(frame);
    return sample ? std::optional<Pose2D>{sample->pose} : std::nullopt;
  }

  struct PoseSample
  {
    Pose2D pose;
    rclcpp::Time stamp;
  };

  std::optional<PoseSample> lookup_pose_sample(const std::string & frame) const
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        frame, base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_s_));
      return PoseSample{
        Pose2D{
          transform.transform.translation.x,
          transform.transform.translation.y,
          tf2::getYaw(transform.transform.rotation)},
        rclcpp::Time(transform.header.stamp)};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "TF lookup %s <- %s failed: %s",
        frame.c_str(), base_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  std::optional<Pose2D> wait_for_pose_after(
    const std::string & frame, const rclcpp::Time & completed_at,
    const std::shared_ptr<GoalHandleNamed> & outer) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(tf_timeout_s_);
    while (rclcpp::ok() && !shutting_down_ && !outer->is_canceling() &&
      std::chrono::steady_clock::now() < deadline)
    {
      const auto sample = lookup_pose_sample(frame);
      if (sample && sample->stamp > completed_at) {
        return sample->pose;
      }
      std::this_thread::sleep_for(20ms);
    }
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped make_pose(
    const std::string & frame, const Pose2D & pose) const
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = frame;
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, pose.yaw);
    message.pose.orientation = tf2::toMsg(quaternion);
    return message;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleNamed> & goal_handle,
    const std::string & phase, float progress, const std::string & message) const
  {
    auto feedback = std::make_shared<NavigateNamed::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = progress;
    feedback->state.message = message;
    goal_handle->publish_feedback(feedback);
  }

  void publish_dock_feedback(
    const std::shared_ptr<GoalHandleNamed> & goal_handle, const DockStep & step) const
  {
    const std::string message = step.message + " forward=" +
      std::to_string(step.forward_error_m) + " lateral=" +
      std::to_string(step.lateral_error_m) + " yaw=" +
      std::to_string(step.yaw_error_rad);
    publish_feedback(goal_handle, "dock_" + to_string(step.phase), 0.85F, message);
  }

  void publish_stop() const
  {
    if (!dock_publisher_) {
      return;
    }
    geometry_msgs::msg::Twist stop;
    for (int index = 0; index < std::max(1, stop_publish_count_); ++index) {
      dock_publisher_->publish(stop);
    }
  }

  void abort(
    const std::shared_ptr<GoalHandleNamed> & goal_handle,
    const std::shared_ptr<NavigateNamed::Result> & result,
    uint16_t code, const std::string & phase, const std::string & message) const
  {
    result->error.code = code;
    result->error.message = phase + ": " + message;
    goal_handle->abort(result);
  }

  bool execution_enabled_{false};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string dock_cmd_vel_topic_{"cmd_vel_dock"};
  double server_timeout_s_{8.0};
  double nav_timeout_s_{120.0};
  double nav_attempt_timeout_s_{35.0};
  double nav_cancel_timeout_s_{2.0};
  double backup_distance_m_{0.20};
  double backup_speed_mps_{0.08};
  double backup_timeout_s_{5.0};
  double already_reached_position_tolerance_m_{0.08};
  double already_reached_yaw_tolerance_rad_{0.08};
  double spin_timeout_s_{15.0};
  double spin_threshold_rad_{0.15};
  double tf_timeout_s_{1.0};
  double dock_control_rate_hz_{20.0};
  double dock_timeout_s_{28.0};
  double dock_lateral_tolerance_m_{0.08};
  double max_initial_distance_m_{0.60};
  int stop_publish_count_{5};
  std::map<std::string, NamedPlace> places_;
  std::unique_ptr<DockController> dock_controller_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dock_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigatePose>::SharedPtr nav_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Server<NavigateNamed>::SharedPtr action_server_;
  mutable std::mutex active_mutex_;
  bool goal_reserved_{false};
  std::atomic<bool> shutting_down_{false};
  std::thread worker_;
};

}  // namespace xlerobot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xlerobot_navigation::NamedNavigationServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
