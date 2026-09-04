#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <xlerobot_interfaces/action/execute_policy_stream.hpp>
#include <xlerobot_interfaces/msg/capability_error.hpp>
#include <xlerobot_interfaces/msg/policy_joint_chunk.hpp>
#include <xlerobot_interfaces/msg/policy_start_context.hpp>

#include "xlerobot_manipulation/controller_mode_manager.hpp"
#include "xlerobot_manipulation/streaming_validator.hpp"

namespace xlerobot_manipulation
{

namespace
{

using namespace std::chrono_literals;
using ExecutePolicyStream = xlerobot_interfaces::action::ExecutePolicyStream;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecutePolicyStream>;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Duration duration_message(double seconds)
{
  builtin_interfaces::msg::Duration message;
  message.sec = static_cast<int32_t>(std::floor(seconds));
  message.nanosec = static_cast<uint32_t>((seconds - std::floor(seconds)) * 1.0e9);
  return message;
}

}  // namespace

class StreamingJointExecutor : public rclcpp::Node
{
public:
  StreamingJointExecutor()
  : Node("streaming_joint_executor")
  {
    load_parameters();
    validator_config_ = make_validator_config();
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&StreamingJointExecutor::on_joint_state, this, std::placeholders::_1));
    chunk_subscription_ = create_subscription<xlerobot_interfaces::msg::PolicyJointChunk>(
      chunk_topic_, rclcpp::QoS(10),
      std::bind(&StreamingJointExecutor::on_chunk, this, std::placeholders::_1));
    command_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      command_topic_, rclcpp::QoS(10));
    controller_modes_ = std::make_unique<ControllerModeManager>(
      *this,
      ControllerModeManager::Config{
        controller_manager_, policy_controller_, normal_controllers_,
        controller_switch_timeout_s_});
    action_server_ = rclcpp_action::create_server<ExecutePolicyStream>(
      this, action_name_,
      std::bind(&StreamingJointExecutor::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&StreamingJointExecutor::handle_cancel, this, std::placeholders::_1),
      std::bind(&StreamingJointExecutor::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Streaming executor ready; execution_enabled=%s policy_controller=%s",
      execution_enabled_ ? "true" : "false", policy_controller_.c_str());
  }

  ~StreamingJointExecutor() override
  {
    shutting_down_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct MeasuredState
  {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::chrono::steady_clock::time_point received_at{};
    bool complete{false};
  };

  struct QueuedSample
  {
    std::vector<double> positions;
    double period_s{0.0};
  };

  struct Session
  {
    std::string id;
    std::string source_id;
    std::unique_ptr<StreamingValidator> validator;
    std::deque<QueuedSample> queue;
    std::chrono::steady_clock::time_point last_chunk_at{};
    uint64_t accepted_chunks{0};
    uint64_t executed_samples{0};
    bool final_received{false};
    bool faulted{false};
    uint16_t fault_code{CapabilityError::BACKEND_FAILURE};
    std::string fault_message;
  };

  void load_parameters()
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    action_name_ = declare_parameter<std::string>("action_name", "execute_policy_stream");
    chunk_topic_ = declare_parameter<std::string>("chunk_topic", "policy_joint_chunks");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/right_policy_controller/joint_trajectory");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    controller_manager_ = declare_parameter<std::string>(
      "controller_manager", "/controller_manager");
    policy_controller_ = declare_parameter<std::string>(
      "policy_controller", "right_policy_controller");
    normal_controllers_ = declare_parameter<std::vector<std::string>>(
      "normal_controllers", {"right_arm_controller", "right_gripper_controller"});
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"});
    lower_positions_ = declare_parameter<std::vector<double>>(
      "lower_positions", {-2.05, -1.40, -1.65, -1.75, -3.09, 0.0});
    upper_positions_ = declare_parameter<std::vector<double>>(
      "upper_positions", {2.05, 1.85, 1.70, 1.75, 3.09, 1.65});
    position_setpoint_joints_ = declare_parameter<std::vector<std::string>>(
      "position_setpoint_joints", {"right_arm_gripper"});
    min_sample_period_s_ = declare_parameter<double>("min_sample_period_s", 0.02);
    max_sample_period_s_ = declare_parameter<double>("max_sample_period_s", 0.10);
    max_command_age_s_ = declare_parameter<double>("max_command_age_s", 0.25);
    max_future_skew_s_ = declare_parameter<double>("max_future_skew_s", 0.05);
    max_queue_horizon_s_ = declare_parameter<double>("max_queue_horizon_s", 2.0);
    input_watchdog_s_ = declare_parameter<double>("input_watchdog_s", 0.50);
    joint_state_timeout_s_ = declare_parameter<double>("joint_state_timeout_s", 0.25);
    controller_switch_timeout_s_ = declare_parameter<double>(
      "controller_switch_timeout_s", 2.0);
    controller_check_period_s_ = declare_parameter<double>("controller_check_period_s", 0.25);
    max_session_duration_s_ = declare_parameter<double>("max_session_duration_s", 30.0);
    max_start_context_age_s_ = declare_parameter<double>("max_start_context_age_s", 5.0);
    start_context_future_tolerance_s_ =
      declare_parameter<double>("start_context_future_tolerance_s", 0.10);
    start_context_tolerance_rad_ =
      declare_parameter<double>("start_context_tolerance_rad", 0.05);
    start_context_velocity_tolerance_radps_ =
      declare_parameter<double>("start_context_velocity_tolerance_radps", 0.05);
    start_context_wait_s_ = declare_parameter<double>("start_context_wait_s", 0.25);
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", 200.0);
    validate_parameters();
  }

  void validate_parameters() const
  {
    const size_t count = joint_names_.size();
    if (count == 0 || lower_positions_.size() != count || upper_positions_.size() != count ||
      normal_controllers_.empty() ||
      action_name_.empty() || chunk_topic_.empty() || command_topic_.empty() ||
      policy_controller_.empty() || min_sample_period_s_ <= 0.0 ||
      max_sample_period_s_ < min_sample_period_s_ || max_command_age_s_ <= 0.0 ||
      max_queue_horizon_s_ <= 0.0 || input_watchdog_s_ <= 0.0 ||
      joint_state_timeout_s_ <= 0.0 ||
      controller_switch_timeout_s_ <= 0.0 || max_session_duration_s_ <= 0.0 ||
      controller_check_period_s_ <= 0.0 ||
      !std::isfinite(max_start_context_age_s_) || max_start_context_age_s_ <= 0.0 ||
      !std::isfinite(start_context_future_tolerance_s_) ||
      start_context_future_tolerance_s_ < 0.0 ||
      !std::isfinite(start_context_tolerance_rad_) || start_context_tolerance_rad_ <= 0.0 ||
      !std::isfinite(start_context_velocity_tolerance_radps_) ||
      start_context_velocity_tolerance_radps_ <= 0.0 ||
      !std::isfinite(start_context_wait_s_) || start_context_wait_s_ <= 0.0 ||
      loop_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("streaming executor parameters are invalid");
    }
    std::vector<std::string> seen;
    for (const auto & name : position_setpoint_joints_) {
      if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end() ||
        std::find(seen.begin(), seen.end(), name) != seen.end())
      {
        throw std::invalid_argument("position_setpoint_joints must be a unique joint subset");
      }
      seen.push_back(name);
    }
  }

  StreamSafetyConfig make_validator_config() const
  {
    StreamSafetyConfig config;
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      const bool position_setpoint =
        std::find(
        position_setpoint_joints_.begin(), position_setpoint_joints_.end(),
        joint_names_[index]) != position_setpoint_joints_.end();
      config.joints.push_back({
          joint_names_[index], lower_positions_[index], upper_positions_[index],
          position_setpoint ? JointCommandSemantics::kPositionSetpoint :
          JointCommandSemantics::kTrajectory});
    }
    config.min_sample_period_s = min_sample_period_s_;
    config.max_sample_period_s = max_sample_period_s_;
    config.max_command_age_s = max_command_age_s_;
    config.max_future_skew_s = max_future_skew_s_;
    config.max_queue_horizon_s = max_queue_horizon_s_;
    return config;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecutePolicyStream::Goal>)
  {
    std::lock_guard<std::mutex> lock(reservation_mutex_);
    if (goal_reserved_ || shutting_down_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    struct ReservationGuard
    {
      StreamingJointExecutor * node;
      ~ReservationGuard()
      {
        std::lock_guard<std::mutex> lock(node->reservation_mutex_);
        node->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<ExecutePolicyStream::Result>();
    const auto & goal = *goal_handle->get_goal();
    const double requested_duration_s = duration_seconds(goal.max_duration);
    if (goal.session_id.empty() || goal.source_id.empty() || goal.joint_names != joint_names_ ||
      requested_duration_s <= 0.0 || requested_duration_s > max_session_duration_s_)
    {
      abort(goal_handle, result, CapabilityError::INVALID_GOAL,
          "invalid session, source, joints, or duration");
      return;
    }
    publish_feedback(goal_handle, nullptr, "plan", 0.05F, "validated exclusive stream plan");
    if (goal.capture_only) {
      abort(
        goal_handle, result, CapabilityError::SAFETY_REJECTED,
        "capture-only policy goals are forbidden on the motor executor");
      return;
    }
    if (goal.dry_run) {
      result->error.code = CapabilityError::NONE;
      result->error.message = "dry-run streaming plan valid";
      result->session_id = goal.session_id;
      goal_handle->succeed(result);
      return;
    }
    if (!execution_enabled_) {
      abort(goal_handle, result, CapabilityError::SAFETY_REJECTED, "execution_enabled is false");
      return;
    }
    MeasuredState measured;
    std::string gate_error;
    const auto context_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(start_context_wait_s_));
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->error.code = CapabilityError::CANCELED;
        result->error.message = "policy stream canceled before arming";
        result->session_id = goal.session_id;
        goal_handle->canceled(result);
        return;
      }
      measured = measured_state();
      if (measured_fresh(measured) &&
        start_context_valid(goal.start_context, measured, gate_error))
      {
        break;
      }
      if (std::chrono::steady_clock::now() >= context_deadline) {
        abort(
          goal_handle, result, CapabilityError::SAFETY_REJECTED,
          gate_error.empty() ? "joint state is missing or stale" : gate_error);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!controller_modes_->enter_policy_mode(gate_error)) {
      abort(goal_handle, result, CapabilityError::BACKEND_FAILURE, gate_error);
      return;
    }
    if (!wait_for_command_subscriber(controller_switch_timeout_s_)) {
      static_cast<void>(controller_modes_->restore_normal_mode(gate_error));
      abort(goal_handle, result, CapabilityError::UNAVAILABLE,
          "policy controller command input unavailable");
      return;
    }

    auto session = std::make_shared<Session>();
    session->id = goal.session_id;
    session->source_id = goal.source_id;
    session->validator = std::make_unique<StreamingValidator>(validator_config_);
    const auto begin_result = session->validator->begin(
      session->id, session->source_id, measured.positions, measured.velocities);
    if (!begin_result) {
      static_cast<void>(controller_modes_->restore_normal_mode(gate_error));
      abort(goal_handle, result, CapabilityError::SAFETY_REJECTED, begin_result.message);
      return;
    }
    session->last_chunk_at = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      active_session_ = session;
    }
    publish_feedback(goal_handle, session, "armed", 0.10F, "session ready for chunk zero");
    run_session(goal_handle, result, session, requested_duration_s);
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (active_session_ == session) {
        active_session_.reset();
      }
    }
  }

  void run_session(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<ExecutePolicyStream::Result> & result,
    const std::shared_ptr<Session> & session,
    double max_duration_s)
  {
    const auto started = std::chrono::steady_clock::now();
    auto next_send = started;
    bool final_sent = false;
    auto final_sent_at = started;
    auto next_controller_check = started;
    auto next_armed_feedback = started;
    const auto loop_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    uint16_t terminal_code = CapabilityError::NONE;
    std::string terminal_message;
    bool canceled = false;

    while (rclcpp::ok() && !shutting_down_) {
      const auto now = std::chrono::steady_clock::now();
      if (goal_handle->is_canceling()) {
        canceled = true;
        terminal_code = CapabilityError::CANCELED;
        terminal_message = "policy stream canceled";
        break;
      }
      const auto measured = measured_state();
      if (!measured_fresh(measured)) {
        terminal_code = CapabilityError::TIMEOUT;
        terminal_message = "joint state became stale during policy stream";
        break;
      }
      if (command_publisher_->get_subscription_count() == 0) {
        terminal_code = CapabilityError::UNAVAILABLE;
        terminal_message = "policy controller command input disappeared";
        break;
      }
      if (now >= next_controller_check) {
        std::string controller_error;
        if (!controller_modes_->policy_mode_intact(controller_error)) {
          terminal_code = CapabilityError::UNAVAILABLE;
          terminal_message = controller_error.empty() ?
            "exclusive policy controller mode was lost" : controller_error;
          break;
        }
        next_controller_check = now +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(controller_check_period_s_));
      }

      QueuedSample sample;
      bool have_sample = false;
      bool final_received = false;
      bool awaiting_first_chunk = false;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (session->faulted) {
          terminal_code = session->fault_code;
          terminal_message = session->fault_message;
          break;
        }
        if (!session->queue.empty() && now >= next_send) {
          sample = std::move(session->queue.front());
          session->queue.pop_front();
          have_sample = true;
          ++session->executed_samples;
        }
        final_received = session->final_received;
        awaiting_first_chunk = session->accepted_chunks == 0;
        if (!final_received && session->queue.empty() &&
          std::chrono::duration<double>(now - session->last_chunk_at).count() > input_watchdog_s_)
        {
          terminal_code = CapabilityError::TIMEOUT;
          terminal_message = "policy input watchdog expired";
          break;
        }
      }
      if (awaiting_first_chunk && now >= next_armed_feedback) {
        publish_feedback(goal_handle, session, "armed", 0.10F, "session ready for chunk zero");
        next_armed_feedback = now + std::chrono::milliseconds(50);
      }
      if (have_sample) {
        publish_command(sample.positions, sample.period_s);
        next_send = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(sample.period_s));
        publish_feedback(goal_handle, session, "running", 0.50F, "executing validated samples");
      }

      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (final_received && session->queue.empty() && !final_sent) {
          final_sent = true;
          final_sent_at = next_send;
        }
      }
      if (final_sent && now >= final_sent_at) {
        terminal_code = CapabilityError::NONE;
        terminal_message = "final policy sample executed";
        break;
      }
      if (std::chrono::duration<double>(now - started).count() > max_duration_s) {
        terminal_code = CapabilityError::TIMEOUT;
        terminal_message = "policy stream exceeded session duration";
        break;
      }
      std::this_thread::sleep_for(loop_period);
    }

    if (terminal_message.empty()) {
      canceled = true;
      terminal_code = CapabilityError::CANCELED;
      terminal_message = "policy stream interrupted by shutdown";
    }

    publish_hold(session);
    std::string switch_error;
    const bool restored = controller_modes_->restore_normal_mode(switch_error);
    fill_result(*result, session);
    if (!restored) {
      terminal_code = CapabilityError::BACKEND_FAILURE;
      terminal_message = "failed to restore normal controllers: " + switch_error;
      canceled = false;
    }
    result->error.code = terminal_code;
    result->error.message = terminal_message;
    if (canceled) {
      goal_handle->canceled(result);
    } else if (terminal_code == CapabilityError::NONE) {
      publish_feedback(goal_handle, session, "complete", 1.0F, terminal_message);
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void on_chunk(const xlerobot_interfaces::msg::PolicyJointChunk::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    const auto session = active_session_;
    if (!session || session->faulted) {
      return;
    }
    PolicyChunk chunk;
    chunk.session_id = message->session_id;
    chunk.source_id = message->source_id;
    chunk.sequence = message->sequence;
    chunk.age_s = (now() - rclcpp::Time(message->generated_at)).seconds();
    chunk.sample_period_s = duration_seconds(message->sample_period);
    chunk.final_chunk = message->final_chunk;
    chunk.joint_names = message->joint_names;
    chunk.sample_count = message->sample_count;
    chunk.positions = message->positions;
    double horizon_s = 0.0;
    for (const auto & queued : session->queue) {
      horizon_s += queued.period_s;
    }
    const auto validation = session->validator->validate_and_accept(chunk, horizon_s);
    if (!validation) {
      session->faulted = true;
      session->fault_code = CapabilityError::SAFETY_REJECTED;
      session->fault_message = "rejected policy chunk: " + validation.message;
      session->queue.clear();
      return;
    }
    const size_t joint_count = joint_names_.size();
    for (uint32_t sample_index = 0; sample_index < message->sample_count; ++sample_index) {
      const size_t offset = static_cast<size_t>(sample_index) * joint_count;
      session->queue.push_back({
          std::vector<double>(
          message->positions.begin() + static_cast<std::ptrdiff_t>(offset),
          message->positions.begin() + static_cast<std::ptrdiff_t>(offset + joint_count)),
          chunk.sample_period_s});
    }
    ++session->accepted_chunks;
    session->final_received = message->final_chunk;
    session->last_chunk_at = std::chrono::steady_clock::now();
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::unordered_map<std::string, size_t> indices;
    for (size_t index = 0; index < message->name.size(); ++index) {
      indices[message->name[index]] = index;
    }
    MeasuredState state;
    for (const auto & name : joint_names_) {
      const auto item = indices.find(name);
      if (item == indices.end() || item->second >= message->position.size() ||
        item->second >= message->velocity.size())
      {
        return;
      }
      state.positions.push_back(message->position[item->second]);
      state.velocities.push_back(message->velocity[item->second]);
    }
    state.received_at = std::chrono::steady_clock::now();
    state.complete = true;
    std::lock_guard<std::mutex> lock(state_mutex_);
    measured_state_ = std::move(state);
  }

  MeasuredState measured_state() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return measured_state_;
  }

  bool measured_fresh(const MeasuredState & state) const
  {
    return state.complete &&
           std::chrono::duration<double>(
      std::chrono::steady_clock::now() - state.received_at).count() <=
           joint_state_timeout_s_;
  }

  bool start_context_valid(
    const xlerobot_interfaces::msg::PolicyStartContext & context,
    const MeasuredState & measured,
    std::string & error) const
  {
    if (context.context_id.empty()) {
      error = "live policy stream requires a pregrasp start context";
      return false;
    }
    if (context.joint_names != joint_names_ || context.positions.size() != joint_names_.size()) {
      error = "pregrasp start context has the wrong joint layout";
      return false;
    }
    const rclcpp::Time established(context.established_at);
    if (established.nanoseconds() <= 0) {
      error = "pregrasp start context timestamp is invalid";
      return false;
    }
    const double age_s = (now() - established).seconds();
    if (!std::isfinite(age_s) || age_s < -start_context_future_tolerance_s_ ||
      age_s > max_start_context_age_s_)
    {
      error = "pregrasp start context is stale or future-dated";
      return false;
    }
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      const double expected = context.positions[index];
      if (!std::isfinite(expected) || expected < lower_positions_[index] ||
        expected > upper_positions_[index])
      {
        error = "pregrasp start context exceeds the joint envelope";
        return false;
      }
      if (std::abs(measured.positions[index] - expected) > start_context_tolerance_rad_) {
        error = "measured state left pregrasp context at joint " + joint_names_[index];
        return false;
      }
      if (std::abs(measured.velocities[index]) > start_context_velocity_tolerance_radps_) {
        error = "measured state is still moving at policy start for joint " + joint_names_[index];
        return false;
      }
    }
    return true;
  }

  bool wait_for_command_subscriber(double timeout_s) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      if (command_publisher_->get_subscription_count() > 0) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  void publish_command(const std::vector<double> & positions, double period_s)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = duration_message(period_s);
    trajectory.points.push_back(std::move(point));
    command_publisher_->publish(trajectory);
  }

  void publish_hold(const std::shared_ptr<Session> & session)
  {
    const auto state = measured_state();
    if (measured_fresh(state)) {
      publish_command(state.positions, 0.10);
    } else if (session && session->validator) {
      std::vector<double> target;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        target = session->validator->last_positions();
      }
      publish_command(target, 0.10);
    }
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<Session> & session,
    const std::string & phase, float progress, const std::string & message)
  {
    auto feedback = std::make_shared<ExecutePolicyStream::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = progress;
    feedback->state.message = message;
    if (session) {
      std::lock_guard<std::mutex> lock(session_mutex_);
      feedback->session_id = session->id;
      feedback->accepted_chunks = session->accepted_chunks;
      feedback->executed_samples = session->executed_samples;
      for (const auto & queued : session->queue) {
        feedback->queued_horizon_s += queued.period_s;
      }
    }
    goal_handle->publish_feedback(feedback);
  }

  void fill_result(
    ExecutePolicyStream::Result & result,
    const std::shared_ptr<Session> & session) const
  {
    if (!session) {
      return;
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    result.session_id = session->id;
    result.accepted_chunks = session->accepted_chunks;
    result.executed_samples = session->executed_samples;
  }

  void abort(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<ExecutePolicyStream::Result> & result,
    uint16_t code, const std::string & message) const
  {
    result->error.code = code;
    result->error.message = message;
    goal_handle->abort(result);
  }

  bool execution_enabled_{false};
  std::string action_name_;
  std::string chunk_topic_;
  std::string command_topic_;
  std::string joint_state_topic_;
  std::string controller_manager_;
  std::string policy_controller_;
  std::vector<std::string> normal_controllers_;
  std::vector<std::string> joint_names_;
  std::vector<double> lower_positions_;
  std::vector<double> upper_positions_;
  std::vector<std::string> position_setpoint_joints_;
  double min_sample_period_s_{0.02};
  double max_sample_period_s_{0.10};
  double max_command_age_s_{0.25};
  double max_future_skew_s_{0.05};
  double max_queue_horizon_s_{2.0};
  double input_watchdog_s_{0.50};
  double joint_state_timeout_s_{0.25};
  double controller_switch_timeout_s_{2.0};
  double controller_check_period_s_{0.25};
  double max_session_duration_s_{30.0};
  double max_start_context_age_s_{5.0};
  double start_context_future_tolerance_s_{0.10};
  double start_context_tolerance_rad_{0.05};
  double start_context_velocity_tolerance_radps_{0.05};
  double start_context_wait_s_{0.25};
  double loop_rate_hz_{200.0};
  StreamSafetyConfig validator_config_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<xlerobot_interfaces::msg::PolicyJointChunk>::SharedPtr chunk_subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_publisher_;
  std::unique_ptr<ControllerModeManager> controller_modes_;
  rclcpp_action::Server<ExecutePolicyStream>::SharedPtr action_server_;

  mutable std::mutex state_mutex_;
  MeasuredState measured_state_;
  mutable std::mutex session_mutex_;
  std::shared_ptr<Session> active_session_;
  std::mutex reservation_mutex_;
  bool goal_reserved_{false};
  std::atomic<bool> shutting_down_{false};
  std::thread worker_;
};

}  // namespace xlerobot_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xlerobot_manipulation::StreamingJointExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
