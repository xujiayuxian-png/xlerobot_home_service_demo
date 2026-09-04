#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xlerobot_interfaces/msg/scan_map_consistency.hpp>

#include "xlerobot_navigation/localization_core.hpp"

namespace xlerobot_navigation
{

class ScanMapConsistencyNode : public rclcpp::Node
{
public:
  ScanMapConsistencyNode()
  : Node("scan_map_consistency"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localization/scan_map_consistency");
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    match_radius_m_ = declare_parameter<double>("match_radius_m", 0.18);
    score_threshold_ = declare_parameter<double>("score_threshold", 0.35);
    min_evaluated_beams_ = declare_parameter<int>("min_evaluated_beams", 20);
    beam_stride_ = declare_parameter<int>("beam_stride", 3);
    smoothing_window_ = declare_parameter<int>("smoothing_window", 3);
    max_range_margin_m_ = declare_parameter<double>("max_range_margin_m", 0.03);
    evaluation_period_s_ = declare_parameter<double>("evaluation_period_s", 0.20);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.15);

    if (map_topic_.empty() || scan_topic_.empty() || output_topic_.empty() ||
      occupied_threshold_ < 1 || occupied_threshold_ > 100 || match_radius_m_ < 0.0 ||
      score_threshold_ <= 0.0 || score_threshold_ > 1.0 || min_evaluated_beams_ < 1 ||
      beam_stride_ < 1 || smoothing_window_ < 1 || max_range_margin_m_ < 0.0 ||
      evaluation_period_s_ <= 0.0 || tf_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("scan-map consistency parameters are invalid");
    }

    consistency_publisher_ = create_publisher<xlerobot_interfaces::msg::ScanMapConsistency>(
      output_topic_, rclcpp::QoS(5).reliable());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        latest_map_ = message;
        score_history_.clear();
      });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanMapConsistencyNode::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "scan-map consistency ready: threshold=%.2f radius=%.2fm window=%d",
      score_threshold_, match_radius_m_, smoothing_window_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (last_evaluation_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now_steady - last_evaluation_).count() < evaluation_period_s_)
    {
      return;
    }
    last_evaluation_ = now_steady;

    if (!latest_map_) {
      publish_unavailable(*scan, "waiting for occupancy map");
      return;
    }
    if (scan->header.frame_id.empty() || latest_map_->header.frame_id.empty()) {
      publish_unavailable(*scan, "scan or map frame is empty");
      return;
    }

    geometry_msgs::msg::TransformStamped transform_message;
    try {
      transform_message = tf_buffer_.lookupTransform(
        latest_map_->header.frame_id, scan->header.frame_id, rclcpp::Time(scan->header.stamp),
        tf2::durationFromSec(tf_timeout_s_));
    } catch (const tf2::TransformException & error) {
      publish_unavailable(*scan,
          std::string("missing timestamped laser-to-map TF: ") + error.what());
      return;
    }

    tf2::Transform laser_to_map;
    tf2::fromMsg(transform_message.transform, laser_to_map);
    std::vector<std::array<double, 2>> endpoints;
    endpoints.reserve(scan->ranges.size() / static_cast<std::size_t>(beam_stride_) + 1U);
    double angle = static_cast<double>(scan->angle_min);
    const double usable_max = static_cast<double>(scan->range_max) - max_range_margin_m_;
    for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
      const double range = static_cast<double>(scan->ranges[index]);
      if (index % static_cast<std::size_t>(beam_stride_) == 0U && std::isfinite(range) &&
        range >= static_cast<double>(scan->range_min) && range <= usable_max)
      {
        const tf2::Vector3 endpoint_laser(
          range * std::cos(angle), range * std::sin(angle), 0.0);
        const tf2::Vector3 endpoint_map = laser_to_map * endpoint_laser;
        endpoints.push_back({endpoint_map.x(), endpoint_map.y()});
      }
      angle += static_cast<double>(scan->angle_increment);
    }

    const auto & map = *latest_map_;
    const auto score = score_scan_endpoints(
      map.data, map.info.width, map.info.height, map.info.resolution,
      map.info.origin.position.x, map.info.origin.position.y,
      tf2::getYaw(map.info.origin.orientation), endpoints,
      static_cast<std::int8_t>(occupied_threshold_), match_radius_m_);
    if (score.evaluated_beams < static_cast<std::size_t>(min_evaluated_beams_)) {
      score_history_.clear();
      std::ostringstream reason;
      reason << "too few usable map endpoints: " << score.evaluated_beams << "/" <<
        min_evaluated_beams_;
      publish_unavailable(*scan, reason.str(), score);
      return;
    }

    score_history_.push_back(score.score());
    while (score_history_.size() > static_cast<std::size_t>(smoothing_window_)) {
      score_history_.pop_front();
    }
    if (score_history_.size() < static_cast<std::size_t>(smoothing_window_)) {
      publish_unavailable(*scan, "warming scan-map score window", score);
      return;
    }

    double smoothed_score = 0.0;
    for (const double value : score_history_) {
      smoothed_score += value;
    }
    smoothed_score /= static_cast<double>(score_history_.size());
    const bool consistent = smoothed_score >= score_threshold_;
    std::ostringstream message;
    message.precision(3);
    message << std::fixed << "scan-map score=" << smoothed_score <<
      " (minimum=" << score_threshold_ << ")";
    publish_result(*scan, true, consistent, smoothed_score, score, message.str());
  }

  void publish_unavailable(
    const sensor_msgs::msg::LaserScan & scan, const std::string & reason,
    const ScanMapScore & score = {})
  {
    publish_result(scan, false, false, score.score(), score, reason);
  }

  void publish_result(
    const sensor_msgs::msg::LaserScan & scan, bool valid, bool consistent, double score_value,
    const ScanMapScore & score, const std::string & message)
  {
    xlerobot_interfaces::msg::ScanMapConsistency result;
    result.header = scan.header;
    if (latest_map_) {
      result.header.frame_id = latest_map_->header.frame_id;
    }
    result.valid = valid;
    result.consistent = consistent;
    result.score = static_cast<float>(score_value);
    result.matched_beams = static_cast<std::uint32_t>(score.matched_beams);
    result.evaluated_beams = static_cast<std::uint32_t>(score.evaluated_beams);
    result.message = message;
    consistency_publisher_->publish(result);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "localization/scan_map_consistency";
    status.hardware_id = "two_wheel_reference";
    // A score is temporarily unavailable before the first localization and while the
    // smoothing window warms up.  That is an expected precondition, not a system
    // blocker.  A valid low score remains ERROR and is handled as a real mismatch.
    status.level = !valid ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      (consistent ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.message = message;
    const auto add_value = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    add_value("score", std::to_string(score_value));
    add_value("threshold", std::to_string(score_threshold_));
    add_value("matched_beams", std::to_string(score.matched_beams));
    add_value("evaluated_beams", std::to_string(score.evaluated_beams));
    diagnostics.status.push_back(std::move(status));
    diagnostic_publisher_->publish(diagnostics);
  }

  std::string map_topic_;
  std::string scan_topic_;
  std::string output_topic_;
  int occupied_threshold_{65};
  double match_radius_m_{0.18};
  double score_threshold_{0.35};
  int min_evaluated_beams_{20};
  int beam_stride_{3};
  int smoothing_window_{3};
  double max_range_margin_m_{0.03};
  double evaluation_period_s_{0.20};
  double tf_timeout_s_{0.15};
  std::chrono::steady_clock::time_point last_evaluation_{};
  std::deque<double> score_history_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<xlerobot_interfaces::msg::ScanMapConsistency>::SharedPtr
    consistency_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace xlerobot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xlerobot_navigation::ScanMapConsistencyNode>());
  rclcpp::shutdown();
  return 0;
}
