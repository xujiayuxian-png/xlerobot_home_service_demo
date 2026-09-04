// Copyright 2026 Lisa

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "xlerobot_feetech/scservo_feetech_bus.hpp"
#include "xlerobot_interfaces/msg/capability_error.hpp"
#include "xlerobot_interfaces/srv/servo_calibration_step.hpp"

namespace
{
using Service = xlerobot_interfaces::srv::ServoCalibrationStep;
using Error = xlerobot_interfaces::msg::CapabilityError;
using Bus = xlerobot_feetech::ScServoFeetechBus;
using namespace std::chrono_literals;

struct JointSpec
{
  std::string group;
  std::string name;
  uint8_t id;
  int direction;
  double lower;
  double upper;
};

struct JointCapture
{
  std::optional<int> offset;
  std::optional<int> raw_min;
  std::optional<int> raw_max;
};

const std::vector<JointSpec> kJoints = {
  {"right_arm", "shoulder_pan", 1, 1, -2.05, 2.05},
  {"right_arm", "shoulder_lift", 2, -1, -1.40, 1.85},
  {"right_arm", "elbow_flex", 3, 1, -1.65, 1.70},
  {"right_arm", "wrist_flex", 4, 1, -1.75, 1.75},
  {"right_arm", "wrist_roll", 5, -1, -3.09, 3.09},
  {"right_arm", "gripper", 6, 1, 0.0, 1.65},
  {"left_arm", "shoulder_pan", 1, 1, -2.05, 2.05},
  {"left_arm", "shoulder_lift", 2, -1, -1.45, 1.78},
  {"left_arm", "elbow_flex", 3, 1, -1.50, 1.70},
  {"left_arm", "wrist_flex", 4, 1, -1.75, 1.75},
  {"left_arm", "wrist_roll", 5, -1, -3.09, 3.09},
  {"left_arm", "gripper", 6, 1, 0.0, 1.65},
  {"head", "pan", 7, -1, -1.57, 1.57},
  {"head", "tilt", 8, 1, -0.76, 1.45},
};

class ServoCalibrationServer : public rclcpp::Node
{
public:
  ServoCalibrationServer()
  : Node("servo_calibration_server")
  {
    const auto right_port = declare_parameter<std::string>("right_port", "/dev/right_arm");
    const auto left_port = declare_parameter<std::string>("left_port", "/dev/left_arm");
    output_ = declare_parameter<std::string>(
      "result_file", "/var/lib/xlerobot/calibration_work/servo/result.yaml");
    const auto baudrate = declare_parameter<int>("baudrate", 1000000);
    if (!right_.connect(right_port, baudrate)) {
      throw std::runtime_error("failed to open right servo bus: " + right_port);
    }
    if (!left_.connect(left_port, baudrate)) {
      throw std::runtime_error("failed to open left servo bus: " + left_port);
    }
    service_ = create_service<Service>(
      "/calibration/servo_step",
      std::bind(
        &ServoCalibrationServer::step, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ = create_wall_timer(20ms, std::bind(&ServoCalibrationServer::sample_range, this));
    RCLCPP_INFO(get_logger(), "Exclusive servo calibration buses are ready");
  }

private:
  Bus & bus(const std::string & group)
  {
    return group == "right_arm" ? right_ : left_;
  }

  std::vector<const JointSpec *> group(const std::string & name) const
  {
    std::vector<const JointSpec *> result;
    for (const auto & joint : kJoints) {
      if (joint.group == name) {
        result.push_back(&joint);
      }
    }
    return result;
  }

  const JointSpec * joint(const std::string & group_name, const std::string & name) const
  {
    const auto found = std::find_if(kJoints.begin(), kJoints.end(), [&](const auto & row) {
          return row.group == group_name && row.name == name;
      });
    return found == kJoints.end() ? nullptr : &*found;
  }

  static std::string key(const JointSpec & joint)
  {
    return joint.group + "." + joint.name;
  }

  static void fail(Service::Response & response, const std::string & message)
  {
    response.error.code = Error::INVALID_GOAL;
    response.error.message = message;
  }

  void scan(
    const std::vector<const JointSpec *> & joints, Service::Response & response)
  {
    for (const auto * item : joints) {
      const auto position = bus(item->group).readPosition(item->id);
      if (!position) {
        throw std::runtime_error(
                item->group + "." + item->name + " did not respond");
      }
      response.joint_names.push_back(key(*item));
      response.raw_positions.push_back(*position);
    }
    response.phase = "SCANNED";
  }

  void release(
    const std::vector<const JointSpec *> & joints, Service::Response & response)
  {
    for (const auto * item : joints) {
      if (!bus(item->group).enableTorque(item->id, false)) {
        throw std::runtime_error("failed to release torque: " + key(*item));
      }
    }
    response.phase = "TORQUE_RELEASED";
  }

  void capture_zero(
    const std::vector<const JointSpec *> & joints, Service::Response & response)
  {
    std::vector<std::pair<std::string, int>> measured;
    for (const auto * item : joints) {
      const auto position = bus(item->group).readPosition(item->id);
      if (!position) {
        throw std::runtime_error("failed to read zero: " + key(*item));
      }
      measured.emplace_back(key(*item), *position);
      response.joint_names.push_back(key(*item));
      response.raw_positions.push_back(*position);
    }
    for (const auto & [name, position] : measured) {
      captures_[name].offset = position;
    }
    response.phase = "ZERO_CAPTURED";
  }

  void start_range(const JointSpec & item, Service::Response & response)
  {
    if (active_) {
      throw std::runtime_error("finish the active range before starting another");
    }
    if (!captures_[key(item)].offset) {
      throw std::runtime_error("capture the group zero pose first");
    }
    const auto position = bus(item.group).readPosition(item.id);
    if (!position) {
      throw std::runtime_error("failed to read range start: " + key(item));
    }
    active_ = &item;
    active_min_ = *position;
    active_max_ = *position;
    response.phase = "RANGE_RECORDING";
    response.joint_names.push_back(key(item));
    response.raw_positions.push_back(*position);
  }

  void finish_range(const JointSpec & item, Service::Response & response)
  {
    if (active_ != &item) {
      throw std::runtime_error("the requested joint is not recording a range");
    }
    const auto expected = (item.upper - item.lower) * 4096.0 / (2.0 * M_PI);
    const auto measured = active_max_ - active_min_;
    if (measured < expected * 0.6) {
      throw std::runtime_error("recorded range covers less than 60% of the ROS range");
    }
    auto & capture = captures_[key(item)];
    capture.raw_min = active_min_;
    capture.raw_max = active_max_;
    active_ = nullptr;
    response.phase = "RANGE_CAPTURED";
    response.joint_names = {key(item) + ".min", key(item) + ".max"};
    response.raw_positions = {active_min_, active_max_};
  }

  void sample_range()
  {
    if (!active_) {
      return;
    }
    const auto position = bus(active_->group).readPosition(active_->id);
    if (position) {
      active_min_ = std::min(active_min_, *position);
      active_max_ = std::max(active_max_, *position);
    }
  }

  void finalize(Service::Response & response)
  {
    if (active_) {
      throw std::runtime_error("finish the active range before finalizing");
    }
    for (const auto & item : kJoints) {
      const auto found = captures_.find(key(item));
      if (found == captures_.end() || !found->second.offset ||
        !found->second.raw_min || !found->second.raw_max)
      {
        throw std::runtime_error("calibration is incomplete: " + key(item));
      }
    }
    const std::filesystem::path target(output_);
    std::filesystem::create_directories(target.parent_path());
    const auto temporary = target.string() + ".tmp";
    std::ofstream stream(temporary, std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("cannot create servo result file");
    }
    stream << "{\n  \"schema\": \"xlerobot_servo_calibration/v1\"";
    for (const auto & group_name : {"right_arm", "left_arm", "head"}) {
      stream << ",\n  \"" << group_name << "\": {\"joints\": {";
      bool first = true;
      for (const auto * item : group(group_name)) {
        const auto & value = captures_.at(key(*item));
        stream << (first ? "\n" : ",\n") << "      \"" << item->name << "\": {"
               << "\"servo_id\": " << static_cast<int>(item->id)
               << ", \"direction\": " << item->direction
               << ", \"offset\": " << *value.offset
               << ", \"raw_min\": " << *value.raw_min
               << ", \"raw_max\": " << *value.raw_max
               << ", \"limit_min\": " << item->lower
               << ", \"limit_max\": " << item->upper << "}";
        first = false;
      }
      stream << "\n    }}";
    }
    stream << "\n}\n";
    stream.close();
    if (!stream) {
      throw std::runtime_error("failed to finish servo result file");
    }
    std::error_code error;
    std::filesystem::rename(temporary, target, error);
    if (error) {
      std::filesystem::remove(target, error);
      error.clear();
      std::filesystem::rename(temporary, target, error);
    }
    if (error) {
      throw std::runtime_error("failed to publish servo result file: " + error.message());
    }
    response.phase = "FINALIZED";
    response.result_uri = "file://" + std::filesystem::absolute(target).string();
  }

  void step(
    const std::shared_ptr<Service::Request> request,
    std::shared_ptr<Service::Response> response)
  {
    try {
      const auto joints = group(request->group);
      if (request->command != Service::Request::FINALIZE && joints.empty()) {
        throw std::runtime_error("group must be right_arm, left_arm, or head");
      }
      if (request->command == Service::Request::SCAN) {
        scan(joints, *response);
      } else if (request->command == Service::Request::RELEASE_TORQUE) {
        release(joints, *response);
      } else if (request->command == Service::Request::CAPTURE_ZERO) {
        capture_zero(joints, *response);
      } else if (request->command == Service::Request::START_RANGE ||  // NOLINT
        request->command == Service::Request::FINISH_RANGE)
      {
        const auto * selected = joint(request->group, request->joint);
        if (!selected) {
          throw std::runtime_error("joint does not belong to the selected group");
        }
        if (request->command == Service::Request::START_RANGE) {
          start_range(*selected, *response);
        } else {
          finish_range(*selected, *response);
        }
      } else if (request->command == Service::Request::FINALIZE) {
        finalize(*response);
      } else {
        throw std::runtime_error("unknown servo calibration command");
      }
      response->error.code = Error::NONE;
      response->error.message = "servo calibration step complete";
    } catch (const std::exception & error) {
      fail(*response, error.what());
    }
  }

  Bus right_;
  Bus left_;
  std::string output_;
  std::unordered_map<std::string, JointCapture> captures_;
  const JointSpec * active_{nullptr};
  int active_min_{std::numeric_limits<int>::max()};
  int active_max_{std::numeric_limits<int>::min()};
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoCalibrationServer>());
  rclcpp::shutdown();
  return 0;
}
