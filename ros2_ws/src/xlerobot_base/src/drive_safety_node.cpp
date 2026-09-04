#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "xlerobot_base/drive_arbiter.hpp"

namespace xlerobot_base
{

class DriveSafetyNode : public rclcpp::Node
{
public:
  DriveSafetyNode()
  : Node("drive_safety"), arbiter_(load_config())
  {
    navigation_topic_ =
      declare_parameter<std::string>("navigation_topic", "cmd_vel_nav");
    docking_topic_ =
      declare_parameter<std::string>("docking_topic", "cmd_vel_dock");
    teleop_topic_ =
      declare_parameter<std::string>("teleop_topic", "cmd_vel_teleop");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/base_controller/cmd_vel");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and positive");
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
    diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    stop_state_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drive_safety/stop_latched",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    stop_service_ = create_service<std_srvs::srv::SetBool>(
      "/drive_safety/set_stop",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        const bool clearing_stop = !request->data && arbiter_.stopped();
        arbiter_.set_stopped(request->data);
        last_selection_ = {
          geometry_msgs::msg::Twist(), DriveSource::kNone, false};
        if (request->data) {
          // A software stop bypasses acceleration limiting and is published
          // synchronously rather than waiting for the periodic arbiter tick.
          publish_zero();
        }
        if (clearing_stop) {
          // Replace the DDS readers so commands queued while stopped cannot be
          // delivered as if they were published after the stop was cleared.
          reset_command_subscriptions();
        }
        publish_stop_state();
        response->success = true;
        response->message =
          request->data ? "base software stop latched" : "base software stop cleared";
      });
    reset_command_subscriptions();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publish_command();});
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostic();});

    publish_stop_state();
    RCLCPP_INFO(get_logger(), "Drive command arbiter ready");
  }

  ~DriveSafetyNode() override
  {
    publish_zero();
  }

private:
  DriveArbiterConfig load_config()
  {
    DriveArbiterConfig config;
    config.navigation_timeout_s = declare_parameter<double>("navigation_timeout_s", 0.5);
    config.docking_timeout_s = declare_parameter<double>("docking_timeout_s", 0.35);
    config.teleop_timeout_s = declare_parameter<double>("teleop_timeout_s", 0.35);
    config.max_linear_mps = declare_parameter<double>("max_linear_mps", 0.15);
    config.max_angular_radps = declare_parameter<double>("max_angular_radps", 0.6);
    config.max_linear_accel_mps2 =
      declare_parameter<double>("max_linear_accel_mps2", 0.35);
    config.max_angular_accel_radps2 =
      declare_parameter<double>("max_angular_accel_radps2", 1.2);
    config.navigation_has_priority = declare_parameter<bool>("navigation_has_priority", true);
    return config;
  }

  void publish_command()
  {
    const double now_s = steady_seconds();
    const double dt_s = last_publish_s_ > 0.0 ? now_s - last_publish_s_ : 0.0;
    last_publish_s_ = now_s;
    last_selection_ = arbiter_.select(now_s, dt_s);
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    output.twist = last_selection_.command;
    command_publisher_->publish(output);
  }

  void publish_zero()
  {
    if (!command_publisher_) {
      return;
    }
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    command_publisher_->publish(output);
  }

  void publish_stop_state()
  {
    std_msgs::msg::Bool message;
    message.data = arbiter_.stopped();
    stop_state_publisher_->publish(message);
  }

  void reset_command_subscriptions()
  {
    navigation_subscription_.reset();
    docking_subscription_.reset();
    teleop_subscription_.reset();
    const uint64_t generation = ++command_subscription_generation_;
    navigation_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      navigation_topic_, 10,
      [this, generation](const geometry_msgs::msg::Twist & message) {
        if (generation == command_subscription_generation_) {
          arbiter_.update_navigation(message, steady_seconds());
        }
      });
    docking_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      docking_topic_, 10,
      [this, generation](const geometry_msgs::msg::Twist & message) {
        if (generation == command_subscription_generation_) {
          arbiter_.update_docking(message, steady_seconds());
        }
      });
    teleop_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      teleop_topic_, 10,
      [this, generation](const geometry_msgs::msg::Twist & message) {
        if (generation == command_subscription_generation_) {
          arbiter_.update_teleop(message, steady_seconds());
        }
      });
  }

  void publish_diagnostic()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "xlerobot/drive_safety";
    status.hardware_id = "two_wheel_reference";
    status.level = arbiter_.stopped() ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = arbiter_.stopped() ?
      "base software stop latched" :
      "drive command path ready";
    diagnostic_msgs::msg::KeyValue stopped;
    stopped.key = "stop_latched";
    stopped.value = arbiter_.stopped() ? "true" : "false";
    status.values.push_back(stopped);
    diagnostic_msgs::msg::KeyValue source;
    source.key = "selected_source";
    source.value = to_string(last_selection_.source);
    status.values.push_back(source);
    diagnostic_msgs::msg::KeyValue limited;
    limited.key = "command_limited";
    limited.value = last_selection_.clamped ? "true" : "false";
    status.values.push_back(limited);
    array.status.push_back(status);
    diagnostic_publisher_->publish(array);
  }

  static double steady_seconds()
  {
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
  }

  DriveArbiter arbiter_;
  double publish_rate_hz_{30.0};
  double last_publish_s_{0.0};
  std::string frame_id_{"base_link"};
  std::string navigation_topic_;
  std::string docking_topic_;
  std::string teleop_topic_;
  uint64_t command_subscription_generation_{0};
  DriveSelection last_selection_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_state_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr docking_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace xlerobot_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xlerobot_base::DriveSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
