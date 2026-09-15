#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

// Only headers are retained. Large Python consumers need not deserialize Image
// messages just to establish readiness. This does not claim DDS zero-copy.
class CameraHealth : public rclcpp::Node
{
public:
  CameraHealth() : Node("camera_health")
  {
    external_wrist_ = declare_parameter("external_wrist_health", false);
    const std::array<std::string, 4> defaults = {
      "/xlerobot/d455/color/image_raw", "/xlerobot/d455/aligned_depth_to_color/image_raw",
      "/xlerobot/d455/color/camera_info", "/right_wrist_camera/image_raw"};
    for (size_t i = 0; i < names_.size(); ++i) {
      if (i == 3 && external_wrist_) {continue;}
      const auto topic = declare_parameter(names_[i] + "_topic", defaults[i]);
      if (i == 2) {
        subscriptions_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
          topic, rclcpp::SensorDataQoS().keep_last(1),
          [this, i](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            if (msg->width && msg->height && msg->k[0] > 0 && msg->k[4] > 0) {
              receive(i, msg->header.stamp);
            }
          }));
      } else {
        subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
          topic, rclcpp::SensorDataQoS().keep_last(1),
          [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            if (msg->width && msg->height && msg->step &&
              msg->data.size() >= static_cast<size_t>(msg->height) * msg->step)
            {
              receive(i, msg->header.stamp);
            }
          }));
      }
    }
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/camera/health", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publish(); });
  }

private:
  using Steady = std::chrono::steady_clock;
  struct Sample {
    int64_t stamp_ns{0};
    Steady::time_point received{};
    uint64_t count{0};
  };
  void receive(size_t i, const builtin_interfaces::msg::Time & stamp)
  {
    samples_[i].stamp_ns = rclcpp::Time(stamp).nanoseconds();
    samples_[i].received = Steady::now();
    ++samples_[i].count;
  }
  void publish()
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    const auto ros_now = rclcpp::Time(message.header.stamp).nanoseconds();
    for (size_t i = 0; i < names_.size(); ++i) {
      if (i == 3 && external_wrist_) {continue;}
      const auto & sample = samples_[i];
      const double stamp_age = (ros_now - sample.stamp_ns) * 1e-9;
      const double receive_age = std::chrono::duration<double>(Steady::now() - sample.received).count();
      const double age = sample.stamp_ns > 0 && stamp_age >= -0.1 ?
        std::max(receive_age, stamp_age) : std::numeric_limits<double>::infinity();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "xlerobot/camera/" + names_[i];
      status.hardware_id = names_[i];
      status.level = age <= 1.0 ? status.OK : status.STALE;
      status.message = age <= 1.0 ? "fresh" : "missing or stale frames";
      diagnostic_msgs::msg::KeyValue value;
      value.key = "age_s";
      value.value = std::to_string(age);
      status.values.push_back(value);
      value.key = "frames_received";
      value.value = std::to_string(sample.count);
      status.values.push_back(value);
      message.status.push_back(status);
    }
    publisher_->publish(message);
  }
  const std::array<std::string, 4> names_ = {"head_color", "head_depth", "head_camera_info", "wrist"};
  std::array<Sample, 4> samples_;
  bool external_wrist_{false};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraHealth>());
  rclcpp::shutdown();
  return 0;
}
