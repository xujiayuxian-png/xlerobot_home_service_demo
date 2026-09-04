#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <xlerobot_interfaces/action/auto_localize.hpp>
#include <xlerobot_interfaces/msg/capability_error.hpp>

#include "xlerobot_navigation/localization_core.hpp"

namespace xlerobot_navigation
{

namespace
{

using namespace std::chrono_literals;
using AutoLocalize = xlerobot_interfaces::action::AutoLocalize;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
using Spin = nav2_msgs::action::Spin;
constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace

class AutoLocalizerServer : public rclcpp::Node
{
public:
  using GoalHandleAutoLocalize = rclcpp_action::ServerGoalHandle<AutoLocalize>;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  AutoLocalizerServer()
  : Node("auto_localizer_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    max_revolutions_ = declare_parameter<double>("max_revolutions", 2.5);
    max_duration_s_ = declare_parameter<double>("max_duration_s", 60.0);
    spin_direction_ = declare_parameter<double>("spin_direction", 1.0);
    service_timeout_s_ = declare_parameter<double>("service_timeout_s", 3.0);
    server_timeout_s_ = declare_parameter<double>("server_timeout_s", 8.0);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    settle_timeout_s_ = declare_parameter<double>("settle_timeout_s", 5.0);
    settle_hold_s_ = declare_parameter<double>("settle_hold_s", 0.75);
    settle_angular_speed_radps_ =
      declare_parameter<double>("settle_angular_speed_radps", 0.03);
    amcl_pose_timeout_s_ = declare_parameter<double>("amcl_pose_timeout_s", 1.0);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "amcl_pose");
    global_localization_service_ = declare_parameter<std::string>(
      "global_localization_service", "/reinitialize_global_localization");
    spin_action_name_ = declare_parameter<std::string>("spin_action_name", "spin");
    action_name_ = declare_parameter<std::string>("action_name", "auto_localize");
    activate_navigation_after_localization_ =
      declare_parameter<bool>("activate_navigation_after_localization", false);
    navigation_lifecycle_manager_ = declare_parameter<std::string>(
      "navigation_lifecycle_manager", "/lifecycle_manager_navigation");
    navigation_startup_timeout_s_ =
      declare_parameter<double>("navigation_startup_timeout_s", 15.0);

    ConvergenceConfig convergence_config;
    convergence_config.min_rotation_rad =
      declare_parameter<double>("convergence.min_rotation_rad", 3.14);
    convergence_config.position_stddev_m =
      declare_parameter<double>("convergence.position_stddev_m", 0.15);
    convergence_config.yaw_stddev_rad =
      declare_parameter<double>("convergence.yaw_stddev_rad", 0.10);
    convergence_config.hold_s = declare_parameter<double>("convergence.hold_s", 1.0);
    convergence_tracker_ = std::make_unique<ConvergenceTracker>(convergence_config);
    validate_parameters();

    amcl_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_pose_ = message;
        latest_quality_ = quality_from_covariance(message.pose.covariance);
        have_pose_ = true;
        latest_pose_received_at_ = std::chrono::steady_clock::now();
        ++pose_sequence_;
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      [this](const nav_msgs::msg::Odometry & message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_angular_speed_radps_ = std::abs(message.twist.twist.angular.z);
        have_odom_ = true;
        ++odom_sequence_;
      });
    global_localization_client_ =
      create_client<std_srvs::srv::Empty>(global_localization_service_);
    navigation_is_active_client_ = create_client<std_srvs::srv::Trigger>(
      navigation_lifecycle_manager_ + "/is_active");
    navigation_manage_client_ = create_client<ManageLifecycleNodes>(
      navigation_lifecycle_manager_ + "/manage_nodes");
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_name_);
    action_server_ = rclcpp_action::create_server<AutoLocalize>(
      this, action_name_,
      std::bind(&AutoLocalizerServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&AutoLocalizerServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&AutoLocalizerServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Auto localization ready; execution_enabled=%s motion_backend=Nav2 Spin",
      execution_enabled_ ? "true" : "false");
  }

  ~AutoLocalizerServer() override
  {
    shutting_down_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct StateSnapshot
  {
    LocalizationQuality quality;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    double angular_speed_radps{std::numeric_limits<double>::infinity()};
    uint64_t odom_sequence{0};
    uint64_t pose_sequence{0};
    std::chrono::steady_clock::time_point pose_received_at{};
    bool have_pose{false};
    bool have_odom{false};
  };

  void validate_parameters() const
  {
    if (max_revolutions_ <= 0.0 || max_revolutions_ > 3.0 || max_duration_s_ <= 0.0 ||
      std::abs(spin_direction_) < 1.0e-9 || service_timeout_s_ <= 0.0 ||
      server_timeout_s_ <= 0.0 || control_rate_hz_ <= 0.0 || settle_timeout_s_ <= 0.0 ||
      settle_hold_s_ <= 0.0 || settle_hold_s_ > settle_timeout_s_ ||
      settle_angular_speed_radps_ <= 0.0 || amcl_pose_timeout_s_ <= 0.0 ||
      odom_topic_.empty() || amcl_pose_topic_.empty() || global_localization_service_.empty() ||
      spin_action_name_.empty() || action_name_.empty() ||
      navigation_lifecycle_manager_.empty() || navigation_startup_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("auto localization parameters are invalid");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const AutoLocalize::Goal>)
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (goal_reserved_ || shutting_down_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAutoLocalize>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAutoLocalize> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<GoalHandleAutoLocalize> goal_handle)
  {
    struct ReservationGuard
    {
      AutoLocalizerServer * server;
      ~ReservationGuard()
      {
        std::lock_guard<std::mutex> lock(server->active_mutex_);
        server->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<AutoLocalize::Result>();
    fill_result_state(*result);
    publish_feedback(goal_handle, "plan", 0.05F, "validated AMCL scatter and Nav2 Spin plan");
    if (goal_handle->get_goal()->dry_run) {
      result->error.code = CapabilityError::NONE;
      result->error.message = "dry-run auto localization plan valid";
      goal_handle->succeed(result);
      return;
    }
    if (!execution_enabled_) {
      abort(goal_handle, result, CapabilityError::SAFETY_REJECTED,
        "execution_enabled is false");
      return;
    }
    reset_localization_sample();
    if (!scatter_particles(goal_handle, result)) {
      return;
    }
    run_spin_until_converged(goal_handle, result);
  }

  bool scatter_particles(
    const std::shared_ptr<GoalHandleAutoLocalize> & goal_handle,
    const std::shared_ptr<AutoLocalize::Result> & result)
  {
    publish_feedback(goal_handle, "scatter", 0.10F, "requesting AMCL global localization");
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_s_);
    while (!global_localization_client_->service_is_ready()) {
      if (goal_handle->is_canceling()) {
        cancel_outer(goal_handle, result, "canceled while waiting for AMCL service");
        return false;
      }
      if (shutting_down_ || !rclcpp::ok() ||
        std::chrono::steady_clock::now() >= service_deadline)
      {
        abort(goal_handle, result, CapabilityError::UNAVAILABLE,
          global_localization_service_ + " unavailable");
        return false;
      }
      static_cast<void>(global_localization_client_->wait_for_service(50ms));
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = global_localization_client_->async_send_request(request);
    if (!wait_future(future, service_timeout_s_, goal_handle)) {
      if (goal_handle->is_canceling()) {
        cancel_outer(goal_handle, result, "canceled while scattering AMCL particles");
      } else {
        abort(goal_handle, result, CapabilityError::TIMEOUT,
          "AMCL global localization request timed out");
      }
      return false;
    }
    try {
      static_cast<void>(future.get());
    } catch (const std::exception & error) {
      abort(goal_handle, result, CapabilityError::BACKEND_FAILURE,
        std::string("AMCL global localization failed: ") + error.what());
      return false;
    }
    // Ignore any AMCL sample that arrived before the scatter service completed.
    reset_localization_sample();
    return true;
  }

  void run_spin_until_converged(
    const std::shared_ptr<GoalHandleAutoLocalize> & outer,
    const std::shared_ptr<AutoLocalize::Result> & result)
  {
    const auto server_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(server_timeout_s_);
    while (!spin_client_->action_server_is_ready()) {
      if (outer->is_canceling()) {
        cancel_outer(outer, result, "canceled while waiting for Nav2 Spin");
        return;
      }
      if (shutting_down_ || !rclcpp::ok() ||
        std::chrono::steady_clock::now() >= server_deadline)
      {
        abort(outer, result, CapabilityError::UNAVAILABLE, "Nav2 Spin action unavailable");
        return;
      }
      static_cast<void>(spin_client_->wait_for_action_server(50ms));
    }
    {
      std::lock_guard<std::mutex> lock(spin_feedback_mutex_);
      spin_rotated_rad_ = 0.0;
    }
    convergence_tracker_->reset();

    Spin::Goal spin_goal;
    spin_goal.target_yaw = std::copysign(max_revolutions_ * kTwoPi, spin_direction_);
    spin_goal.time_allowance.sec = static_cast<int32_t>(std::floor(max_duration_s_));
    spin_goal.time_allowance.nanosec = static_cast<uint32_t>(
      (max_duration_s_ - std::floor(max_duration_s_)) * 1.0e9);
    rclcpp_action::Client<Spin>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandleSpin::SharedPtr, const std::shared_ptr<const Spin::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(spin_feedback_mutex_);
        spin_rotated_rad_ = std::abs(feedback->angular_distance_traveled);
      };
    GoalHandleSpin::SharedPtr spin_handle;
    while (std::chrono::steady_clock::now() < server_deadline && !spin_handle) {
      auto send_future = spin_client_->async_send_goal(spin_goal, options);
      const double remaining_s = std::chrono::duration<double>(
        server_deadline - std::chrono::steady_clock::now()).count();
      if (!wait_future(send_future, std::max(0.05, remaining_s), outer)) {
        if (outer->is_canceling()) {
          cancel_outer(outer, result, "canceled before Nav2 Spin accepted");
        } else {
          abort(outer, result, CapabilityError::TIMEOUT, "Nav2 Spin acceptance timed out");
        }
        return;
      }
      spin_handle = send_future.get();
      if (!spin_handle) {
        publish_feedback(
          outer, "waiting_nav2", 0.12F,
          "Nav2 Spin is present but not active; waiting for lifecycle bringup");
        std::this_thread::sleep_for(100ms);
      }
    }
    if (!spin_handle) {
      abort(
        outer, result, CapabilityError::UNAVAILABLE,
        "Nav2 Spin remained inactive throughout the readiness window");
      return;
    }

    auto spin_result_future = spin_client_->async_get_result(spin_handle);
    const auto started_at = std::chrono::steady_clock::now();
    auto previous_pose_received_at = started_at;
    uint64_t last_pose_sequence = snapshot().pose_sequence;
    bool have_previous_pose_sample = false;
    const auto deadline = started_at + std::chrono::duration<double>(max_duration_s_ + 2.0);
    while (rclcpp::ok() && !shutting_down_) {
      if (outer->is_canceling()) {
        static_cast<void>(cancel_spin_and_wait(spin_handle, spin_result_future, 2.0));
        cancel_outer(outer, result, "auto localization canceled");
        return;
      }
      const auto current = std::chrono::steady_clock::now();
      const double rotated_rad = spin_rotation();
      const auto state = snapshot();
      bool converged = false;
      if (state.have_pose && state.pose_sequence != last_pose_sequence) {
        const double pose_interval_s = have_previous_pose_sample ?
          std::chrono::duration<double>(
          state.pose_received_at - previous_pose_received_at).count() : 0.0;
        if (have_previous_pose_sample && pose_interval_s > amcl_pose_timeout_s_) {
          convergence_tracker_->reset();
        }
        last_pose_sequence = state.pose_sequence;
        previous_pose_received_at = state.pose_received_at;
        have_previous_pose_sample = true;
        converged = convergence_tracker_->update(
          state.quality, rotated_rad, std::max(0.0, pose_interval_s));
      } else if (state.have_pose &&
        std::chrono::duration<double>(current - state.pose_received_at).count() >
        amcl_pose_timeout_s_)
      {
        convergence_tracker_->reset();
      }
      if (converged) {
        if (!cancel_spin_and_wait(spin_handle, spin_result_future, 2.0)) {
          abort(outer, result, CapabilityError::TIMEOUT,
            "Nav2 Spin did not stop after convergence cancellation");
          return;
        }
        const auto settle_status = wait_for_settle(outer, rotated_rad);
        if (settle_status == SettleStatus::kCanceled) {
          cancel_outer(outer, result, "canceled while waiting for rotation to settle");
          return;
        }
        if (settle_status == SettleStatus::kTimeout) {
          abort(outer, result, CapabilityError::TIMEOUT,
            "odom did not confirm a stable stop after Nav2 Spin cancellation");
          return;
        }
        const auto final_state = snapshot();
        fill_result_state(*result, final_state);
        if (!pose_is_fresh(final_state) ||
          !convergence_tracker_->is_within_threshold(final_state.quality, rotated_rad))
        {
          abort(outer, result, CapabilityError::BACKEND_FAILURE,
            "AMCL convergence was lost while rotation settled");
          return;
        }
        if (!ensure_navigation_active(outer, result)) {
          return;
        }
        // Lifecycle activation does not move the robot. AMCL may therefore suppress
        // redundant pose publications while Nav2 configures, so do not require a new
        // pose here. The converged, settled pose was validated immediately above;
        // the task layer subsequently requires a fresh scan-map consistency result.
        result->error.code = CapabilityError::NONE;
        result->error.message = activate_navigation_after_localization_ ?
          "AMCL localization converged; Nav2 navigation active" :
          "AMCL localization converged";
        publish_feedback(outer, "complete", 1.0F, result->error.message);
        outer->succeed(result);
        return;
      }
      publish_quality_feedback(outer, convergence_tracker_->phase(), rotated_rad, state.quality);

      if (spin_result_future.wait_for(0ms) == std::future_status::ready) {
        const auto wrapped = spin_result_future.get();
        fill_result_state(*result);
        if (wrapped.result && wrapped.result->error_code == Spin::Result::TIMEOUT) {
          abort(outer, result, CapabilityError::TIMEOUT,
            "Nav2 Spin timed out before AMCL converged");
        } else {
          abort(outer, result, CapabilityError::BACKEND_FAILURE,
            "maximum localization rotation reached without convergence");
        }
        return;
      }
      if (current >= deadline) {
        static_cast<void>(cancel_spin_and_wait(spin_handle, spin_result_future, 2.0));
        abort(outer, result, CapabilityError::TIMEOUT,
          "auto localization exceeded its deadline");
        return;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / control_rate_hz_));
    }
    static_cast<void>(cancel_spin_and_wait(spin_handle, spin_result_future, 2.0));
    cancel_outer(outer, result, "auto localization interrupted by shutdown");
  }

  bool ensure_navigation_active(
    const std::shared_ptr<GoalHandleAutoLocalize> & outer,
    const std::shared_ptr<AutoLocalize::Result> & result)
  {
    if (!activate_navigation_after_localization_) {
      return true;
    }
    publish_feedback(
      outer, "activate_navigation", 0.95F,
      "activating map-dependent Nav2 planner and navigator");
    const auto wait_for_service = [this, &outer](const auto & client) {
        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(navigation_startup_timeout_s_);
        while (!client->service_is_ready()) {
          if (shutting_down_ || !rclcpp::ok() || outer->is_canceling() ||
            std::chrono::steady_clock::now() >= deadline)
          {
            return false;
          }
          static_cast<void>(client->wait_for_service(50ms));
        }
        return true;
      };

    if (!wait_for_service(navigation_is_active_client_)) {
      if (outer->is_canceling()) {
        cancel_outer(outer, result, "canceled while checking Nav2 lifecycle state");
      } else {
        abort(outer, result, CapabilityError::UNAVAILABLE,
          "Nav2 lifecycle status service unavailable");
      }
      return false;
    }
    auto status_future = navigation_is_active_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!wait_future(status_future, service_timeout_s_, outer)) {
      if (outer->is_canceling()) {
        cancel_outer(outer, result, "canceled while checking Nav2 lifecycle state");
      } else {
        abort(outer, result, CapabilityError::TIMEOUT,
          "Nav2 lifecycle status request timed out");
      }
      return false;
    }
    try {
      if (status_future.get()->success) {
        return true;
      }
    } catch (const std::exception & error) {
      abort(outer, result, CapabilityError::BACKEND_FAILURE,
        std::string("Nav2 lifecycle status request failed: ") + error.what());
      return false;
    }

    if (!wait_for_service(navigation_manage_client_)) {
      if (outer->is_canceling()) {
        cancel_outer(outer, result, "canceled before Nav2 lifecycle startup");
      } else {
        abort(outer, result, CapabilityError::UNAVAILABLE,
          "Nav2 lifecycle management service unavailable");
      }
      return false;
    }
    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = ManageLifecycleNodes::Request::STARTUP;
    auto startup_future = navigation_manage_client_->async_send_request(request);
    if (!wait_future(startup_future, navigation_startup_timeout_s_, outer)) {
      if (outer->is_canceling()) {
        cancel_outer(outer, result, "canceled during Nav2 lifecycle startup");
      } else {
        abort(outer, result, CapabilityError::TIMEOUT,
          "Nav2 lifecycle startup timed out");
      }
      return false;
    }
    try {
      if (!startup_future.get()->success) {
        abort(outer, result, CapabilityError::BACKEND_FAILURE,
          "Nav2 lifecycle manager rejected navigation startup");
        return false;
      }
    } catch (const std::exception & error) {
      abort(outer, result, CapabilityError::BACKEND_FAILURE,
        std::string("Nav2 lifecycle startup failed: ") + error.what());
      return false;
    }
    return true;
  }

  template<typename ResultFutureT>
  bool cancel_spin_and_wait(
    const GoalHandleSpin::SharedPtr & spin_handle, ResultFutureT & result_future,
    double timeout_s)
  {
    auto cancel_future = spin_client_->async_cancel_goal(spin_handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (cancel_future.wait_for(20ms) != std::future_status::ready) {
      if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
    }
    while (result_future.wait_for(20ms) != std::future_status::ready) {
      if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
    }
    return true;
  }

  enum class SettleStatus
  {
    kSettled,
    kCanceled,
    kTimeout,
  };

  SettleStatus wait_for_settle(
    const std::shared_ptr<GoalHandleAutoLocalize> & outer, double rotated_rad)
  {
    double stable_s = 0.0;
    auto previous = std::chrono::steady_clock::now();
    const auto deadline = previous + std::chrono::duration<double>(settle_timeout_s_);
    uint64_t last_odom_sequence = snapshot().odom_sequence;
    while (rclcpp::ok() && !shutting_down_) {
      if (outer->is_canceling()) {
        return SettleStatus::kCanceled;
      }
      const auto current = std::chrono::steady_clock::now();
      const double dt_s = std::chrono::duration<double>(current - previous).count();
      previous = current;
      const auto state = snapshot();
      if (state.odom_sequence != last_odom_sequence) {
        last_odom_sequence = state.odom_sequence;
        if (state.have_odom && state.angular_speed_radps <= settle_angular_speed_radps_) {
          stable_s += std::max(0.0, dt_s);
        } else {
          stable_s = 0.0;
        }
      } else if (state.have_odom && state.angular_speed_radps > settle_angular_speed_radps_) {
        stable_s = 0.0;
      }
      publish_quality_feedback(outer, "settling", rotated_rad, state.quality);
      if (stable_s >= settle_hold_s_) {
        return SettleStatus::kSettled;
      }
      if (current >= deadline) {
        return SettleStatus::kTimeout;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / control_rate_hz_));
    }
    return SettleStatus::kCanceled;
  }

  template<typename FutureT>
  bool wait_future(
    FutureT & future, double timeout_s,
    const std::shared_ptr<GoalHandleAutoLocalize> & outer) const
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

  double spin_rotation() const
  {
    std::lock_guard<std::mutex> lock(spin_feedback_mutex_);
    return spin_rotated_rad_;
  }

  void reset_localization_sample()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_quality_ = LocalizationQuality{};
    latest_pose_ = geometry_msgs::msg::PoseWithCovarianceStamped{};
    have_pose_ = false;
  }

  StateSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return {
      latest_quality_, latest_pose_, latest_angular_speed_radps_, odom_sequence_,
      pose_sequence_, latest_pose_received_at_, have_pose_, have_odom_};
  }

  bool pose_is_fresh(const StateSnapshot & state) const
  {
    return state.have_pose &&
           std::chrono::duration<double>(
      std::chrono::steady_clock::now() - state.pose_received_at).count() <=
           amcl_pose_timeout_s_;
  }

  void fill_result_state(AutoLocalize::Result & result) const
  {
    fill_result_state(result, snapshot());
  }

  void fill_result_state(AutoLocalize::Result & result, const StateSnapshot & state) const
  {
    if (state.have_pose) {
      result.pose = state.pose;
    }
    result.position_stddev_m = state.quality.position_stddev_m;
    result.yaw_stddev_rad = state.quality.yaw_stddev_rad;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleAutoLocalize> & goal_handle,
    const std::string & phase, float progress, const std::string & message) const
  {
    auto feedback = std::make_shared<AutoLocalize::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = progress;
    feedback->state.message = message;
    const auto state = snapshot();
    feedback->rotated_rad = spin_rotation();
    feedback->position_stddev_m = state.quality.position_stddev_m;
    feedback->yaw_stddev_rad = state.quality.yaw_stddev_rad;
    goal_handle->publish_feedback(feedback);
  }

  void publish_quality_feedback(
    const std::shared_ptr<GoalHandleAutoLocalize> & goal_handle,
    const std::string & phase, double rotated_rad, const LocalizationQuality & quality) const
  {
    auto feedback = std::make_shared<AutoLocalize::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = static_cast<float>(
      std::clamp(rotated_rad / (max_revolutions_ * kTwoPi), 0.0, 0.95));
    feedback->state.message = "evaluating AMCL covariance while Nav2 Spin is active";
    feedback->rotated_rad = rotated_rad;
    feedback->position_stddev_m = quality.position_stddev_m;
    feedback->yaw_stddev_rad = quality.yaw_stddev_rad;
    goal_handle->publish_feedback(feedback);
  }

  void abort(
    const std::shared_ptr<GoalHandleAutoLocalize> & goal_handle,
    const std::shared_ptr<AutoLocalize::Result> & result,
    uint16_t code, const std::string & message) const
  {
    fill_result_state(*result);
    result->error.code = code;
    result->error.message = message;
    goal_handle->abort(result);
  }

  void cancel_outer(
    const std::shared_ptr<GoalHandleAutoLocalize> & goal_handle,
    const std::shared_ptr<AutoLocalize::Result> & result,
    const std::string & message) const
  {
    fill_result_state(*result);
    result->error.code = CapabilityError::CANCELED;
    result->error.message = message;
    goal_handle->canceled(result);
  }

  bool execution_enabled_{false};
  bool activate_navigation_after_localization_{false};
  double max_revolutions_{2.5};
  double max_duration_s_{60.0};
  double spin_direction_{1.0};
  double service_timeout_s_{3.0};
  double server_timeout_s_{8.0};
  double control_rate_hz_{20.0};
  double settle_timeout_s_{5.0};
  double settle_hold_s_{0.75};
  double settle_angular_speed_radps_{0.03};
  double amcl_pose_timeout_s_{1.0};
  std::string odom_topic_;
  std::string amcl_pose_topic_;
  std::string global_localization_service_;
  std::string spin_action_name_;
  std::string action_name_;
  std::string navigation_lifecycle_manager_;
  double navigation_startup_timeout_s_{15.0};

  std::unique_ptr<ConvergenceTracker> convergence_tracker_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_localization_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr navigation_is_active_client_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr navigation_manage_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Server<AutoLocalize>::SharedPtr action_server_;

  mutable std::mutex state_mutex_;
  LocalizationQuality latest_quality_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  double latest_angular_speed_radps_{std::numeric_limits<double>::infinity()};
  uint64_t odom_sequence_{0};
  uint64_t pose_sequence_{0};
  std::chrono::steady_clock::time_point latest_pose_received_at_{};
  bool have_pose_{false};
  bool have_odom_{false};
  mutable std::mutex spin_feedback_mutex_;
  double spin_rotated_rad_{0.0};
  mutable std::mutex active_mutex_;
  bool goal_reserved_{false};
  std::atomic<bool> shutting_down_{false};
  std::thread worker_;
};

}  // namespace xlerobot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xlerobot_navigation::AutoLocalizerServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
