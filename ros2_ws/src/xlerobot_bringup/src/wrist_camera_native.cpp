#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV's V4L2 grab dequeues MJPEG; retrieve decodes only when requested.
// Keep draining the device even at idle, so a new action never drains old frames.
class WristCameraNative : public rclcpp::Node
{
public:
  WristCameraNative() : Node("right_wrist_camera")
  {
    device_ = declare_parameter<std::string>("video_device", "");
    frame_id_ = declare_parameter<std::string>(
      "camera_frame_id", "right_arm_wrist_camera_rgb_optical_frame");
    width_ = declare_parameter("image_width", 640);
    height_ = declare_parameter("image_height", 480);
    fps_ = declare_parameter("fps", 30.0);
    if (device_.rfind("/dev/", 0) != 0 || device_.rfind("/dev/video", 0) == 0 ||
      width_ <= 0 || height_ <= 0 || !std::isfinite(fps_) || fps_ <= 0)
    {
      throw std::runtime_error("valid stable /dev camera link and positive profile required");
    }
    images_ = create_publisher<sensor_msgs::msg::Image>(
      "/right_wrist_camera/image_raw", rclcpp::SensorDataQoS().keep_last(1));
    health_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/camera/health", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {publish_health();});
    worker_ = std::thread([this]() {capture();});
  }
  ~WristCameraNative() override {stopping_ = true; if (worker_.joinable()) {worker_.join();}}

private:
  bool open(cv::VideoCapture & camera)
  {
    if (!camera.open(device_, cv::CAP_V4L2)) {return false;}
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FPS, fps_);
    if (!camera.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
      RCLCPP_ERROR(get_logger(), "camera rejected the single-frame capture queue");
      camera.release();
      return false;
    }
    if (static_cast<int>(camera.get(cv::CAP_PROP_FOURCC)) != cv::VideoWriter::fourcc('M', 'J', 'P', 'G') ||
      std::lround(camera.get(cv::CAP_PROP_FRAME_WIDTH)) != width_ ||
      std::lround(camera.get(cv::CAP_PROP_FRAME_HEIGHT)) != height_)
    {
      camera.release();
      RCLCPP_ERROR(get_logger(), "camera rejected the required MJPG resolution");
      return false;
    }
    return true;
  }
  void capture()
  {
    cv::VideoCapture camera;
    auto last_decode = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && !stopping_) {
      try {
        if (!camera.isOpened()) {
          if (!open(camera)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
          }
          last_decode = std::chrono::steady_clock::time_point{};
        }
        const auto stamp = now();  // Before acquisition, never after decoding.
        if (!camera.grab()) {
          camera.release();
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
          continue;
        }
        const bool requested = images_->get_subscription_count() > 0;
        const auto steady = std::chrono::steady_clock::now();
        // Low-rate decode validation also detects corrupt idle camera output.
        if (requested || steady - last_decode >= std::chrono::seconds(1)) {
          cv::Mat bgr;
          if (!camera.retrieve(bgr) || bgr.empty() || bgr.type() != CV_8UC3) {
            camera.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
          }
          last_decode = steady;
          if (requested) {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id_;
            msg.height = bgr.rows;
            msg.width = bgr.cols;
            msg.encoding = "bgr8";
            msg.step = bgr.cols * 3;
            msg.data.resize(msg.height * msg.step);
            for (int row = 0; row < bgr.rows; ++row) {
              std::copy_n(bgr.ptr<uint8_t>(row), msg.step, msg.data.data() + row * msg.step);
            }
            images_->publish(std::move(msg));
          }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_stamp_ = stamp.nanoseconds();
        last_received_ = steady;
        ++frames_;
      } catch (const cv::Exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "camera capture: %s", e.what());
        camera.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  }
  void publish_health()
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "xlerobot/camera/wrist";
    status.hardware_id = "wrist";
    std::lock_guard<std::mutex> lock(mutex_);
    double age = last_stamp_ == 0 ? INFINITY : std::max(
      (now().nanoseconds() - last_stamp_) / 1e9,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_received_).count());
    status.level = age >= 0 && age <= 1 ? status.OK : status.STALE;
    status.message = status.level == status.OK ? "fresh capture" : "missing or stale frames";
    diagnostic_msgs::msg::KeyValue value;
    value.key = "age_s"; value.value = std::to_string(age); status.values.push_back(value);
    value.key = "frames_received"; value.value = std::to_string(frames_); status.values.push_back(value);
    msg.status.push_back(status);
    health_->publish(msg);
  }
  std::string device_, frame_id_;
  int width_, height_;
  double fps_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  std::mutex mutex_;
  int64_t last_stamp_{0};
  uint64_t frames_{0};
  std::chrono::steady_clock::time_point last_received_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<WristCameraNative>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("wrist_camera"), "%s", e.what()); return 1;}
  rclcpp::shutdown();
  return 0;
}
