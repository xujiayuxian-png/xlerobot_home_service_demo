#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "xlerobot_lidar_driver/lidar_packet_parser.hpp"
#include "xlerobot_lidar_driver/scan_accumulator.hpp"
#include "xlerobot_lidar_driver/scan_builder.hpp"
#include "xlerobot_lidar_driver/scan_filter.hpp"
#include "xlerobot_lidar_driver/serial_port.hpp"

namespace xlerobot_lidar_driver
{

namespace
{
constexpr int kLidarBaudrate = 150000;
}  // namespace

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar_node")
  {
    declareParameters();
    loadParameters();

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    parser_.setChecksumEnabled(checksum_enabled_);

    if (mock_hardware_) {
      const auto period = std::chrono::duration<double>(1.0 / mock_scan_rate_hz_);
      read_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {publishMockScan();});
      RCLCPP_INFO(
        get_logger(),
        "Lidar started in deterministic mock mode; no serial device will be opened");
      return;
    }
    if (!hardware_enabled_) {
      RCLCPP_FATAL(
        get_logger(),
        "Refusing to open %s: real lidar requires mock_hardware=false and hardware_enabled=true",
        port_name_.c_str());
      throw std::runtime_error("lidar hardware gate is closed");
    }

    if (!serial_.openPort(port_name_, kLidarBaudrate)) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", port_name_.c_str());
      throw std::runtime_error("failed to open lidar serial port");
    }

    serial_.writeBytes({0xA5, 0x60});
    RCLCPP_INFO(get_logger(), "Lidar started. Port: %s", port_name_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Filter Config -> Enabled: %s, Rad: %.2fm, Neighbors: %d",
      filter_config_.enabled ? "true" : "false",
      filter_config_.radius_m,
      filter_config_.min_neighbors);
    RCLCPP_INFO(
      get_logger(),
      "Mask Config -> Enabled: %s, Ranges: %zu values, MaxRange: %.2fm",
      fixed_mask_config_.enabled ? "true" : "false",
      fixed_mask_config_.angle_ranges_deg.size(),
      fixed_mask_config_.max_range_m);
    RCLCPP_INFO(
      get_logger(),
      "Auto Body Posts Mask -> Enabled: %s, Range: %.2f-%.2fm, Padding: %.1fdeg, MaxClusters: %d",
      auto_body_posts_config_.enabled ? "true" : "false",
      auto_body_posts_config_.min_range_m,
      auto_body_posts_config_.max_range_m,
      auto_body_posts_config_.padding_deg,
      auto_body_posts_config_.max_clusters);
    RCLCPP_INFO(
      get_logger(),
      "Checksum Validation -> Enabled: %s",
      checksum_enabled_ ? "true" : "false");

    if (fixed_mask_config_.angle_ranges_deg.size() % 2 != 0) {
      RCLCPP_WARN(get_logger(), "mask.angle_ranges_deg should contain start/end degree pairs.");
    }
    read_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {pollSerial();});
  }

  ~LidarNode() override
  {
    shutdown();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    declare_parameter<std::string>("frame_id", "laser_frame");
    declare_parameter<bool>("mock_hardware", true);
    declare_parameter<bool>("hardware_enabled", false);
    declare_parameter<double>("mock.scan_rate", 7.0);
    declare_parameter<double>("mock.range", 2.0);
    declare_parameter<int>("scan.size", 720);
    declare_parameter<double>("scan.range_min", 0.05);
    declare_parameter<double>("scan.range_max", 8.0);
    declare_parameter<double>("scan.scan_time", 1.0 / 7.0);
    declare_parameter<bool>("filter.enabled", true);
    declare_parameter<double>("filter.radius", 0.10);
    declare_parameter<int>("filter.min_neighbors", 2);
    declare_parameter<bool>("mask.enabled", false);
    declare_parameter<std::vector<double>>("mask.angle_ranges_deg", std::vector<double>{});
    declare_parameter<double>("mask.max_range", 0.0);
    declare_parameter<bool>("mask.auto_body_posts.enabled", false);
    declare_parameter<double>("mask.auto_body_posts.min_range", 0.12);
    declare_parameter<double>("mask.auto_body_posts.max_range", 0.35);
    declare_parameter<double>("mask.auto_body_posts.padding_deg", 3.0);
    declare_parameter<double>("mask.auto_body_posts.cluster_gap_deg", 4.0);
    declare_parameter<int>("mask.auto_body_posts.max_clusters", 4);
    declare_parameter<int>("mask.auto_body_posts.min_cluster_points", 1);
    declare_parameter<bool>("checksum.enabled", true);
    declare_parameter<int>("serial.max_bytes_per_poll", 4096);
  }

  void loadParameters()
  {
    get_parameter("port_name", port_name_);
    get_parameter("frame_id", frame_id_);
    get_parameter("mock_hardware", mock_hardware_);
    get_parameter("hardware_enabled", hardware_enabled_);
    get_parameter("mock.scan_rate", mock_scan_rate_hz_);
    get_parameter("mock.range", mock_range_m_);
    if (mock_scan_rate_hz_ <= 0.0) {
      throw std::invalid_argument("mock.scan_rate must be positive");
    }
    int scan_size = 720;
    get_parameter("scan.size", scan_size);

    ScanBuilderOptions options;
    options.frame_id = frame_id_;
    get_parameter("scan.range_min", options.range_min_m);
    get_parameter("scan.range_max", options.range_max_m);
    get_parameter("scan.scan_time", options.scan_time_s);
    validateScanOptions(scan_size, options);
    scan_size_ = options.scan_size;
    range_min_m_ = options.range_min_m;
    range_max_m_ = options.range_max_m;
    fallback_scan_time_s_ = options.scan_time_s;

    get_parameter("filter.enabled", filter_config_.enabled);
    get_parameter("filter.radius", filter_config_.radius_m);
    get_parameter("filter.min_neighbors", filter_config_.min_neighbors);
    get_parameter("mask.enabled", fixed_mask_config_.enabled);
    get_parameter("mask.angle_ranges_deg", fixed_mask_config_.angle_ranges_deg);
    get_parameter("mask.max_range", fixed_mask_config_.max_range_m);
    get_parameter("mask.auto_body_posts.enabled", auto_body_posts_config_.enabled);
    get_parameter("mask.auto_body_posts.min_range", auto_body_posts_config_.min_range_m);
    get_parameter("mask.auto_body_posts.max_range", auto_body_posts_config_.max_range_m);
    get_parameter("mask.auto_body_posts.padding_deg", auto_body_posts_config_.padding_deg);
    get_parameter("mask.auto_body_posts.cluster_gap_deg", auto_body_posts_config_.cluster_gap_deg);
    get_parameter("mask.auto_body_posts.max_clusters", auto_body_posts_config_.max_clusters);
    get_parameter("mask.auto_body_posts.min_cluster_points",
        auto_body_posts_config_.min_cluster_points);
    get_parameter("checksum.enabled", checksum_enabled_);
    get_parameter("serial.max_bytes_per_poll", max_bytes_per_poll_);
    max_bytes_per_poll_ = std::max(256, max_bytes_per_poll_);

    scan_builder_ = std::make_unique<ScanBuilder>(
      options,
      ScanFilter(filter_config_, fixed_mask_config_, auto_body_posts_config_));
  }

  void validateScanOptions(int scan_size, ScanBuilderOptions & options)
  {
    if (scan_size <= 0) {
      RCLCPP_WARN(get_logger(), "scan.size must be positive; using 720.");
      options.scan_size = 720;
    } else {
      options.scan_size = static_cast<size_t>(scan_size);
    }
    if (options.range_min_m < 0.0) {
      RCLCPP_WARN(get_logger(), "scan.range_min must be non-negative; using 0.05.");
      options.range_min_m = 0.05;
    }
    if (options.range_max_m <= options.range_min_m) {
      RCLCPP_WARN(get_logger(), "scan.range_max must be greater than scan.range_min; using 8.0.");
      options.range_max_m = 8.0;
    }
    if (options.scan_time_s <= 0.0) {
      RCLCPP_WARN(get_logger(), "scan.scan_time must be positive; using 1/7.");
      options.scan_time_s = 1.0 / 7.0;
    }
  }

  void shutdown()
  {
    if (is_shutdown_.exchange(true)) {
      return;
    }
    serial_.sendStopCommand();
    serial_.closePort();
  }

  void publishMockScan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = frame_id_;
    scan.angle_min = 0.0;
    scan.angle_max = 2.0 * M_PI;
    scan.angle_increment = 2.0 * M_PI / static_cast<double>(scan_size_);
    scan.scan_time = 1.0 / mock_scan_rate_hz_;
    scan.time_increment = scan.scan_time / static_cast<double>(scan_size_);
    scan.range_min = range_min_m_;
    scan.range_max = range_max_m_;
    const float mock_range = static_cast<float>(
      std::clamp(mock_range_m_, range_min_m_, range_max_m_));
    scan.ranges.assign(scan_size_, mock_range);
    scan.intensities.assign(scan_size_, 100.0F);
    scan_pub_->publish(scan);
  }

  void pollSerial()
  {
    uint8_t buffer[1024];
    int bytes_processed = 0;
    while (bytes_processed < max_bytes_per_poll_) {
      const int n = serial_.readBytes(buffer, sizeof(buffer));
      if (n > 0) {
        for (int i = 0; i < n; ++i) {
          processByte(buffer[i]);
        }
        bytes_processed += n;
        continue;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_FATAL(get_logger(), "Lidar serial read error: %s", std::strerror(errno));
        shutdown();
        rclcpp::shutdown();
      }
      break;
    }
    if (bytes_processed >= max_bytes_per_poll_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Lidar read capped at %d bytes this timer tick.", max_bytes_per_poll_);
    }
  }

  void processByte(uint8_t byte)
  {
    const uint64_t invalid_before = parser_.invalidChecksumCount();
    const auto packet = parser_.processByte(byte);
    if (parser_.invalidChecksumCount() != invalid_before) {
      logInvalidChecksum();
    }
    if (packet) {
      processPacket(*packet);
    }
  }

  void logInvalidChecksum()
  {
    const rclcpp::Time now = this->now();
    if (last_checksum_warn_time_.nanoseconds() != 0 &&
      (now - last_checksum_warn_time_).seconds() <= 2.0)
    {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Dropped lidar packet with invalid checksum: expected=0x%04X actual=0x%04X total_invalid=%llu",
      parser_.lastExpectedChecksum(),
      parser_.lastActualChecksum(),
      static_cast<unsigned long long>(parser_.invalidChecksumCount()));
    last_checksum_warn_time_ = now;
  }

  void processPacket(const LidarPacket & packet)
  {
    if (scan_start_time_.nanoseconds() == 0) {
      scan_start_time_ = now();
    }

    const auto completed_scans = scan_accumulator_.addPacket(packet);
    for (const auto & completed_scan : completed_scans) {
      finishCurrentScan(completed_scan.points);
    }
  }

  void finishCurrentScan(const std::vector<LidarPoint> & points)
  {
    const rclcpp::Time boundary_time = now();
    if (!points.empty()) {
      if (scan_count_ > 0) {
        scan_builder_->clear();
        scan_builder_->addPoints(points);
        const rclcpp::Time stamp =
          (scan_start_time_.nanoseconds() == 0) ? boundary_time : scan_start_time_;
        const double measured_scan_time_s =
          (scan_start_time_.nanoseconds() == 0) ?
          fallback_scan_time_s_ :
          (boundary_time - scan_start_time_).seconds();
        const auto result = scan_builder_->build(
          static_cast<builtin_interfaces::msg::Time>(stamp),
          measured_scan_time_s);
        maybeLogAutoBodyPosts(result.auto_mask_clusters, result.scan);
        scan_pub_->publish(result.scan);
      } else {
        RCLCPP_INFO(get_logger(), "Skipping first partial scan...");
      }
    }

    scan_count_++;
    scan_start_time_ = boundary_time;
  }

  void maybeLogAutoBodyPosts(
    const std::vector<AutoMaskCluster> & clusters,
    const sensor_msgs::msg::LaserScan & scan)
  {
    if (clusters.empty()) {
      return;
    }

    const rclcpp::Time current_time = now();
    if (last_auto_body_posts_log_time_.nanoseconds() != 0 &&
      (current_time - last_auto_body_posts_log_time_).seconds() <= 2.0)
    {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Auto body posts masked %zu cluster(s):%s",
      clusters.size(),
      summarizeAutoMaskClusters(clusters, scan).c_str());
    last_auto_body_posts_log_time_ = current_time;
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::string port_name_;
  std::string frame_id_;
  bool mock_hardware_{true};
  bool hardware_enabled_{false};
  double mock_scan_rate_hz_{7.0};
  double mock_range_m_{2.0};
  size_t scan_size_{720};
  double range_min_m_{0.05};
  double range_max_m_{8.0};
  RadiusFilterConfig filter_config_;
  FixedMaskConfig fixed_mask_config_;
  AutoBodyPostsMaskConfig auto_body_posts_config_;
  bool checksum_enabled_ = true;
  double fallback_scan_time_s_ = 1.0 / 7.0;
  int max_bytes_per_poll_ = 4096;

  SerialPort serial_;
  LidarPacketParser parser_;
  ScanAccumulator scan_accumulator_;
  std::unique_ptr<ScanBuilder> scan_builder_;

  std::atomic_bool is_shutdown_{false};
  int scan_count_ = 0;
  rclcpp::Time scan_start_time_;
  rclcpp::Time last_checksum_warn_time_;
  rclcpp::Time last_auto_body_posts_log_time_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

}  // namespace xlerobot_lidar_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<xlerobot_lidar_driver::LidarNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_node"), "Exception: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
