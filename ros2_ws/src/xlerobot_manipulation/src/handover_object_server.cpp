#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <xlerobot_interfaces/action/handover_object.hpp>
#include <xlerobot_interfaces/msg/capability_error.hpp>

namespace xlerobot_manipulation
{

namespace
{

using namespace std::chrono_literals;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;
using HandoverObject = xlerobot_interfaces::action::HandoverObject;
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;

struct HandoverFailure : public std::runtime_error
{
  HandoverFailure(
    uint16_t code_value, const std::string & message_value,
    bool canceled_value = false)
  : std::runtime_error(message_value), code(code_value), canceled(canceled_value) {}

  uint16_t code;
  bool canceled;
};

bool valid_identifier(const std::string & value)
{
  return !value.empty() && value.size() <= 128U &&
         std::none_of(value.begin(), value.end(), [](unsigned char character) {
             return character < 32U;
    });
}

}  // namespace

class HandoverObjectServer : public rclcpp::Node
{
public:
  using GoalHandleHandover = rclcpp_action::ServerGoalHandle<HandoverObject>;

  HandoverObjectServer()
  : Node("handover_object_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll"});
    handover_positions_ = declare_parameter<std::vector<double>>(
      "handover_positions", {-0.3344, -0.0952, -0.4458, 0.5291, 0.0});
    ready_positions_ = declare_parameter<std::vector<double>>(
      "ready_positions", {-0.11505, 1.62142, 1.62602, 0.50621, 0.00153});
    lower_positions_ = declare_parameter<std::vector<double>>(
      "lower_positions", {-2.05, -1.40, -1.65, -1.75, -3.09});
    upper_positions_ = declare_parameter<std::vector<double>>(
      "upper_positions", {2.05, 1.85, 1.70, 1.75, 3.09});
    gripper_joint_ = declare_parameter<std::string>(
      "gripper_joint", "right_arm_gripper");
    gripper_lower_ = declare_parameter<double>("gripper_lower", 0.0);
    gripper_upper_ = declare_parameter<double>("gripper_upper", 1.65);
    gripper_open_position_ = declare_parameter<double>("gripper_open_position", 1.35);
    gripper_closed_position_ = declare_parameter<double>("gripper_closed_position", 0.0);
    held_max_position_ = declare_parameter<double>("held_max_position", 0.15);
    handover_duration_s_ = declare_parameter<double>("handover_duration_s", 2.5);
    release_wait_s_ = declare_parameter<double>("release_wait_s", 0.5);
    gripper_open_duration_s_ = declare_parameter<double>("gripper_open_duration_s", 1.5);
    ready_duration_s_ = declare_parameter<double>("ready_duration_s", 4.0);
    gripper_close_duration_s_ = declare_parameter<double>("gripper_close_duration_s", 1.0);
    service_timeout_s_ = declare_parameter<double>("service_timeout_s", 5.0);
    controller_timeout_margin_s_ =
      declare_parameter<double>("controller_timeout_margin_s", 5.0);
    joint_state_timeout_s_ = declare_parameter<double>("joint_state_timeout_s", 0.50);
    validate_parameters();

    const auto arm_action = declare_parameter<std::string>(
      "arm_action", "/right_arm_controller/follow_joint_trajectory");
    const auto gripper_action = declare_parameter<std::string>(
      "gripper_action", "/right_gripper_controller/follow_joint_trajectory");
    const auto joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");

    arm_client_ = rclcpp_action::create_client<FollowTrajectory>(this, arm_action);
    gripper_client_ = rclcpp_action::create_client<FollowTrajectory>(this, gripper_action);
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_joint_state_ = *message;
        joint_state_received_at_ = std::chrono::steady_clock::now();
      });
    action_server_ = rclcpp_action::create_server<HandoverObject>(
      this, "handover_object",
      std::bind(&HandoverObjectServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&HandoverObjectServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&HandoverObjectServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "HandoverObject ready; execution_enabled=%s",
      execution_enabled_ ? "true" : "false");
  }

  ~HandoverObjectServer() override
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
    const size_t count = joint_names_.size();
    const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
    if (count == 0U || handover_positions_.size() != count ||
      ready_positions_.size() != count || lower_positions_.size() != count ||
      upper_positions_.size() != count || gripper_joint_.empty() ||
      !std::isfinite(gripper_lower_) || !std::isfinite(gripper_upper_) ||
      gripper_lower_ >= gripper_upper_ || !std::isfinite(gripper_open_position_) ||
      !std::isfinite(gripper_closed_position_) || !std::isfinite(held_max_position_) ||
      gripper_open_position_ < gripper_lower_ || gripper_open_position_ > gripper_upper_ ||
      gripper_closed_position_ < gripper_lower_ ||
      gripper_closed_position_ > gripper_upper_ || held_max_position_ < gripper_lower_ ||
      held_max_position_ >= gripper_open_position_ ||
      gripper_closed_position_ > held_max_position_ || !positive(handover_duration_s_) ||
      !positive(release_wait_s_) || !positive(gripper_open_duration_s_) ||
      !positive(ready_duration_s_) || !positive(gripper_close_duration_s_) ||
      !positive(service_timeout_s_) || !positive(controller_timeout_margin_s_) ||
      !positive(joint_state_timeout_s_))
    {
      throw std::invalid_argument("handover parameters are invalid");
    }
    std::vector<std::string> unique_names = joint_names_;
    std::sort(unique_names.begin(), unique_names.end());
    if (std::adjacent_find(unique_names.begin(), unique_names.end()) != unique_names.end() ||
      std::any_of(unique_names.begin(), unique_names.end(), [](const auto & name) {
        return name.empty();
      }))
    {
      throw std::invalid_argument("handover joint names must be unique and nonempty");
    }
    for (size_t index = 0; index < count; ++index) {
      if (!std::isfinite(lower_positions_[index]) || !std::isfinite(upper_positions_[index]) ||
        lower_positions_[index] >= upper_positions_[index] ||
        !inside(handover_positions_[index], lower_positions_[index], upper_positions_[index]) ||
        !inside(ready_positions_[index], lower_positions_[index], upper_positions_[index]))
      {
        throw std::invalid_argument("handover arm pose exceeds a joint limit");
      }
    }
  }

  static bool inside(double value, double lower, double upper)
  {
    return std::isfinite(value) && value >= lower && value <= upper;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const HandoverObject::Goal> goal)
  {
    if (!valid_identifier(goal->object_id) || !valid_identifier(goal->recipient_id)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (goal_reserved_ || shutting_down_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleHandover>)
  {
    cancel_active_child();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHandover> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<GoalHandleHandover> goal_handle)
  {
    struct ReservationGuard
    {
      HandoverObjectServer * server;
      ~ReservationGuard()
      {
        server->clear_active_child();
        std::lock_guard<std::mutex> lock(server->active_mutex_);
        server->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<HandoverObject::Result>();
    try {
      if (goal_handle->get_goal()->dry_run) {
        result->error.code = CapabilityError::NONE;
        result->error.message = "dry-run handover sequence and pose limits valid";
        publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
        goal_handle->succeed(result);
        return;
      }
      require_motion_gate();
      require_held_gripper(goal_handle);
      publish_feedback(goal_handle, "handover_pose", 0.15F, "moving arm to handover pose");
      send_named_trajectory(
        goal_handle, arm_client_, joint_names_, handover_positions_,
        handover_duration_s_, "handover pose");
      publish_feedback(goal_handle, "release_wait", 0.40F, "waiting before release");
      gated_wait(goal_handle, release_wait_s_);
      publish_feedback(goal_handle, "release", 0.55F, "opening right gripper");
      send_named_trajectory(
        goal_handle, gripper_client_, {gripper_joint_}, {gripper_open_position_},
        gripper_open_duration_s_, "open gripper");
      publish_feedback(goal_handle, "return_ready", 0.70F, "returning arm to ready pose");
      send_named_trajectory(
        goal_handle, arm_client_, joint_names_, ready_positions_,
        ready_duration_s_, "return ready");
      publish_feedback(goal_handle, "ready_gripper", 0.90F, "closing empty gripper");
      send_named_trajectory(
        goal_handle, gripper_client_, {gripper_joint_}, {gripper_closed_position_},
        gripper_close_duration_s_, "close gripper");
      result->error.code = CapabilityError::NONE;
      result->error.message = "handover sequence completed";
      publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
      goal_handle->succeed(result);
    } catch (const HandoverFailure & failure) {
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

  void require_held_gripper(const std::shared_ptr<GoalHandleHandover> & goal_handle)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_s_);
    while (rclcpp::ok() && !shutting_down_) {
      check_parent(goal_handle);
      require_motion_gate();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double age_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - joint_state_received_at_).count();
        const auto item = std::find(
          latest_joint_state_.name.begin(), latest_joint_state_.name.end(), gripper_joint_);
        if (age_s <= joint_state_timeout_s_ && item != latest_joint_state_.name.end()) {
          const size_t index = std::distance(latest_joint_state_.name.begin(), item);
          if (index >= latest_joint_state_.position.size() ||
            !std::isfinite(latest_joint_state_.position[index]))
          {
            throw HandoverFailure(
                    CapabilityError::SAFETY_REJECTED, "gripper state is nonfinite or incomplete");
          }
          if (latest_joint_state_.position[index] > held_max_position_) {
            throw HandoverFailure(
                    CapabilityError::SAFETY_REJECTED,
                    "gripper is too open to claim an object for handover");
          }
          return;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw HandoverFailure(
                CapabilityError::TIMEOUT, "fresh right-gripper state is unavailable");
      }
      std::this_thread::sleep_for(20ms);
    }
    throw HandoverFailure(CapabilityError::CANCELED, "handover interrupted", true);
  }

  void send_named_trajectory(
    const std::shared_ptr<GoalHandleHandover> & parent,
    const rclcpp_action::Client<FollowTrajectory>::SharedPtr & client,
    const std::vector<std::string> & names,
    const std::vector<double> & positions,
    double duration_s,
    const std::string & label)
  {
    require_motion_gate();
    wait_for_controller(client, parent, label);
    FollowTrajectory::Goal goal;
    goal.trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(duration_s));
    goal.trajectory.points.push_back(point);
    auto send_future = client->async_send_goal(goal);
    const auto handle = wait_for_goal_handle(send_future, client, parent, label);
    if (!handle) {
      throw HandoverFailure(CapabilityError::BACKEND_FAILURE, label + " goal rejected");
    }
    set_active_child([client, handle]() {client->async_cancel_goal(handle);});
    require_motion_gate_or_cancel();
    auto result_future = client->async_get_result(handle);
    const auto wrapped = wait_for_result(
      result_future, parent, duration_s + controller_timeout_margin_s_, label);
    clear_active_child();
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      throw HandoverFailure(CapabilityError::CANCELED, label + " canceled", true);
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != FollowTrajectory::Result::SUCCESSFUL)
    {
      throw HandoverFailure(CapabilityError::BACKEND_FAILURE, label + " controller failed");
    }
  }

  void wait_for_controller(
    const rclcpp_action::Client<FollowTrajectory>::SharedPtr & client,
    const std::shared_ptr<GoalHandleHandover> & parent,
    const std::string & label)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_s_);
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(parent);
      require_motion_gate();
      if (client->wait_for_action_server(50ms)) {
        return;
      }
    }
    throw HandoverFailure(CapabilityError::UNAVAILABLE, label + " controller unavailable");
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowTrajectory>> wait_for_goal_handle(
    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowTrajectory>>> & future,
    const rclcpp_action::Client<FollowTrajectory>::SharedPtr & client,
    const std::shared_ptr<GoalHandleHandover> & parent,
    const std::string & label)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_s_);
    while (rclcpp::ok() && !shutting_down_ &&
      future.wait_for(20ms) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw HandoverFailure(CapabilityError::TIMEOUT, label + " acceptance timed out");
      }
    }
    if (!rclcpp::ok() || shutting_down_) {
      throw HandoverFailure(CapabilityError::CANCELED, label + " interrupted", true);
    }
    const auto handle = future.get();
    if (parent->is_canceling()) {
      if (handle) {
        static_cast<void>(client->async_cancel_goal(handle));
      }
      throw HandoverFailure(CapabilityError::CANCELED, "handover canceled", true);
    }
    return handle;
  }

  rclcpp_action::ClientGoalHandle<FollowTrajectory>::WrappedResult wait_for_result(
    std::shared_future<rclcpp_action::ClientGoalHandle<FollowTrajectory>::WrappedResult> & future,
    const std::shared_ptr<GoalHandleHandover> & parent,
    double timeout_s,
    const std::string & label)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !shutting_down_ &&
      future.wait_for(20ms) != std::future_status::ready)
    {
      check_parent(parent);
      require_motion_gate_or_cancel();
      if (std::chrono::steady_clock::now() >= deadline) {
        cancel_active_child();
        throw HandoverFailure(CapabilityError::TIMEOUT, label + " timed out");
      }
    }
    if (!rclcpp::ok() || shutting_down_) {
      throw HandoverFailure(CapabilityError::CANCELED, label + " interrupted", true);
    }
    return future.get();
  }

  void gated_wait(
    const std::shared_ptr<GoalHandleHandover> & parent,
    double duration_s)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_s);
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(parent);
      require_motion_gate();
      std::this_thread::sleep_for(20ms);
    }
    if (!rclcpp::ok() || shutting_down_) {
      throw HandoverFailure(CapabilityError::CANCELED, "handover wait interrupted", true);
    }
  }

  void check_parent(const std::shared_ptr<GoalHandleHandover> & parent)
  {
    if (parent->is_canceling()) {
      cancel_active_child();
      throw HandoverFailure(CapabilityError::CANCELED, "handover canceled", true);
    }
  }

  void require_motion_gate() const
  {
    if (!execution_enabled_) {
      throw HandoverFailure(
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
    const std::shared_ptr<GoalHandleHandover> & goal_handle,
    const std::string & phase,
    float progress,
    const std::string & message)
  {
    auto feedback = std::make_shared<HandoverObject::Feedback>();
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
  std::vector<std::string> joint_names_;
  std::vector<double> handover_positions_;
  std::vector<double> ready_positions_;
  std::vector<double> lower_positions_;
  std::vector<double> upper_positions_;
  std::string gripper_joint_;
  double gripper_lower_{0.0};
  double gripper_upper_{1.65};
  double gripper_open_position_{1.35};
  double gripper_closed_position_{0.0};
  double held_max_position_{0.15};
  double handover_duration_s_{2.5};
  double release_wait_s_{0.5};
  double gripper_open_duration_s_{1.5};
  double ready_duration_s_{4.0};
  double gripper_close_duration_s_{1.0};
  double service_timeout_s_{5.0};
  double controller_timeout_margin_s_{5.0};
  double joint_state_timeout_s_{0.50};

  mutable std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::chrono::steady_clock::time_point joint_state_received_at_{};
  std::mutex active_mutex_;
  bool goal_reserved_{false};
  std::mutex child_mutex_;
  std::function<void()> cancel_active_;
  std::atomic<bool> shutting_down_{false};
  std::thread worker_;

  rclcpp_action::Client<FollowTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr gripper_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp_action::Server<HandoverObject>::SharedPtr action_server_;
};

}  // namespace xlerobot_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xlerobot_manipulation::HandoverObjectServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
