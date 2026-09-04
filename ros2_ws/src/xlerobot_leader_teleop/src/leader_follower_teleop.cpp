#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

namespace xlerobot_leader_teleop
{

class LeaderFollowerTeleop final : public rclcpp::Node
{
public:
  LeaderFollowerTeleop()
  : Node("leader_follower_teleop")
  {
    timeout_ = declare_parameter("leader_timeout_s", 0.5);
    enable_lease_s_ = declare_parameter("enable_lease_s", 1.0);
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
      throw std::invalid_argument("leader_timeout_s must be finite and positive");
    }
    if (!std::isfinite(enable_lease_s_) || enable_lease_s_ <= 0.0 ||
      enable_lease_s_ > 2.0)
    {
      throw std::invalid_argument(
              "enable_lease_s must be finite, positive, and no greater than "
              "2 seconds");
    }
    source_names_ = declare_parameter<std::vector<std::string>>("source_joint_names", {
        "leader_shoulder_pan", "leader_shoulder_lift", "leader_elbow_flex",
        "leader_wrist_flex", "leader_wrist_roll", "leader_gripper"});
    target_names_ = declare_parameter<std::vector<std::string>>("target_joint_names", {
        "right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"});
    lower_ = declare_parameter<std::vector<double>>(
      "lower_limits", {-2.05, -1.4, -1.65, -1.75, -3.09, 0.05});
    upper_ = declare_parameter<std::vector<double>>(
      "upper_limits", {2.05, 1.7, 1.4, 1.75, 3.09, 1.65});
    if (source_names_.size() != 6 || target_names_.size() != 6 ||
      lower_.size() != 6 || upper_.size() != 6)
    {
      throw std::invalid_argument("Leader/Follower mapping must contain six joints");
    }
    for (std::size_t i = 0; i < lower_.size(); ++i) {
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) {
        throw std::invalid_argument("Leader joint limits are invalid");
      }
    }
    arm_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/right_arm_controller/joint_trajectory", 10);
    gripper_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/right_gripper_controller/joint_trajectory", 10);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&LeaderFollowerTeleop::on_state, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::SetBool>(
      "/leader_follower_teleop/set_enabled",
      std::bind(
        &LeaderFollowerTeleop::set_enabled, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(33ms, std::bind(&LeaderFollowerTeleop::tick, this));
  }

private:
  void on_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->name.size() != message->position.size()) {
      return;
    }
    std::unordered_map<std::string, double> values;
    for (std::size_t i = 0; i < message->name.size(); ++i) {
      values[message->name[i]] = message->position[i];
    }
    std::array<double, 6> candidate{};
    for (std::size_t i = 0; i < source_names_.size(); ++i) {
      const auto found = values.find(source_names_[i]);
      if (found == values.end() || !std::isfinite(found->second) ||
        found->second < lower_[i] || found->second > upper_[i])
      {
        return;
      }
      candidate[i] = found->second;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    positions_ = candidate;
    received_at_ = std::chrono::steady_clock::now();
    complete_ = true;
  }

  bool fresh_locked() const
  {
    const auto age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_at_).count();
    return complete_ && age_s <= timeout_;
  }

  bool lease_fresh_locked() const
  {
    return enabled_ && std::chrono::steady_clock::now() < lease_deadline_;
  }

  void set_enabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->data && !fresh_locked()) {
      enabled_ = false;
      response->success = false;
      response->message = "leader state is incomplete or stale";
      return;
    }
    if (request->data) {
      enabled_ = true;
      lease_deadline_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(enable_lease_s_));
    } else {
      enabled_ = false;
      lease_deadline_ = std::chrono::steady_clock::time_point{};
    }
    response->success = true;
    response->message = enabled_ ?
      "teleoperation lease enabled or refreshed" : "teleoperation disabled";
  }

  void tick()
  {
    std::array<double, 6> positions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!lease_fresh_locked() || !fresh_locked()) {
        enabled_ = false;
        lease_deadline_ = std::chrono::steady_clock::time_point{};
        return;
      }
      positions = positions_;
    }
    publish(
      std::vector<std::string>(target_names_.begin(), target_names_.begin() + 5),
      std::vector<double>(positions.begin(), positions.begin() + 5), arm_publisher_);
    publish({target_names_[5]}, {positions[5]}, gripper_publisher_);
  }

  void publish(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher)
  {
    trajectory_msgs::msg::JointTrajectory command;
    command.header.stamp = now();
    command.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration(0, 80'000'000);
    command.points.push_back(point);
    publisher->publish(command);
  }

  double timeout_{0.5};
  double enable_lease_s_{1.0};
  bool enabled_{false};
  bool complete_{false};
  std::chrono::steady_clock::time_point received_at_{};
  std::chrono::steady_clock::time_point lease_deadline_{};
  std::mutex mutex_;
  std::array<double, 6> positions_{};
  std::vector<std::string> source_names_;
  std::vector<std::string> target_names_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xlerobot_leader_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xlerobot_leader_teleop::LeaderFollowerTeleop>());
  rclcpp::shutdown();
  return 0;
}
