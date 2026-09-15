#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xlerobot_interfaces/srv/lookup_robot_transform.hpp>

// Only data interfaces: this process never opens or commands a motor.
class RobotStateGateway : public rclcpp::Node
{
public:
  RobotStateGateway() : Node("x1_robot_state")
  {
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Dedicated TF receive thread keeps bounded lookups from blocking updates.
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    lookup_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lookup_ = create_service<xlerobot_interfaces::srv::LookupRobotTransform>(
      "/x1/lookup_transform", [this](
        const xlerobot_interfaces::srv::LookupRobotTransform::Request::SharedPtr req,
        xlerobot_interfaces::srv::LookupRobotTransform::Response::SharedPtr res) {
        if (req->target_frame.empty() || req->source_frame.empty() ||
          !std::isfinite(req->timeout_s) || req->timeout_s < 0 || req->timeout_s > 5.0 ||
          req->stamp.sec < 0 || req->stamp.nanosec >= 1000000000u)
        {
          res->error = "invalid frame, timestamp or lookup timeout (expected 0..5 seconds)";
          return;
        }
        try {
          res->transform = buffer_->lookupTransform(
            req->target_frame, req->source_frame, rclcpp::Time(req->stamp),
            rclcpp::Duration::from_seconds(req->timeout_s));
          res->success = true;
        } catch (const tf2::TransformException & error) {
          res->error = error.what();
        }
      }, rmw_qos_profile_services_default, lookup_group_);
    summary_ = create_publisher<sensor_msgs::msg::JointState>("/x1/joint_state_summary", 1);
    // No re-stamping or repeat publication of stale measurements.
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {latest_ = msg;});
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (latest_) {
        summary_->publish(*latest_);
        latest_.reset();
      }
    });
  }

private:
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::CallbackGroup::SharedPtr lookup_group_;
  rclcpp::Service<xlerobot_interfaces::srv::LookupRobotTransform>::SharedPtr lookup_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr summary_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<RobotStateGateway>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
