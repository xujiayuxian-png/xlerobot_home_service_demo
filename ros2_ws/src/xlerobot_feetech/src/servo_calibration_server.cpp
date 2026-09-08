// Copyright 2026 Lisa
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "xlerobot_feetech/scservo_feetech_bus.hpp"
#include "xlerobot_feetech/servo_calibration_session.hpp"
#include "xlerobot_interfaces/msg/capability_error.hpp"
#include "xlerobot_interfaces/msg/servo_calibration_joint.hpp"
#include "xlerobot_interfaces/srv/servo_calibration_step.hpp"

namespace
{
using Service = xlerobot_interfaces::srv::ServoCalibrationStep;
using Error = xlerobot_interfaces::msg::CapabilityError;
using Bus = xlerobot_feetech::ScServoFeetechBus;
using Session = xlerobot_feetech::ServoCalibrationSession;
using namespace std::chrono_literals;

class ServoCalibrationServer : public rclcpp::Node
{
public:
  ServoCalibrationServer()
  : Node("servo_calibration_server")
  {
    const auto right_port = declare_parameter<std::string>("right_port", "/dev/right_arm");
    const auto left_port = declare_parameter<std::string>("left_port", "/dev/left_arm");
    const auto unit_id = declare_parameter<std::string>("unit_id", "");
    output_ = declare_parameter<std::string>(
      "result_file", ".xlerobot/calibration_work/servo/result.yaml");
    session_file_ = std::filesystem::path(output_).parent_path() / "session.yaml";
    session_ = std::make_unique<Session>(unit_id, right_port + "|" + left_port);
    const auto reference = declare_parameter<std::string>("existing_servo_file", "");
    const auto version = declare_parameter<std::string>("existing_servo_version", "");
    if (!reference.empty()) {session_->load_reference(reference, version);}
    if (std::filesystem::exists(session_file_)) {session_->restore(session_file_);}
    if (std::filesystem::exists(output_)) {session_->recover_finalized_result(output_);}
    if (session_->finalized() && !std::filesystem::exists(output_)) {
      throw std::runtime_error("finalized session result is missing; restore it or use --fresh");
    }
    const auto baudrate = declare_parameter<int>("baudrate", 1000000);
    if (!right_.connect(right_port, baudrate)) {
      throw std::runtime_error("failed to open right servo bus: " + right_port);
    }
    if (!left_.connect(left_port, baudrate)) {
      throw std::runtime_error("failed to open left servo bus: " + left_port);
    }
    service_ = create_service<Service>(
      "/calibration/servo_step",
      std::bind(&ServoCalibrationServer::step, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(50ms, std::bind(&ServoCalibrationServer::sample_range, this));
    persist();
    RCLCPP_INFO(get_logger(), "Servo calibration ready: %s; no torque/motion enabled", session_->phase().c_str());
  }

  ~ServoCalibrationServer() override
  {
    if (!session_) {return;}
    try {session_->save(session_file_);} catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot save servo session on shutdown: %s", error.what());
    }
  }

private:
  Bus & bus(const std::string & group) {return group == "right_arm" ? right_ : left_;}

  Session::Positions read_positions(const std::string & name)
  {
    const auto indices = session_->group(name);
    std::vector<uint8_t> ids;
    for (const auto i : indices) {ids.push_back(Session::specs()[i].id);}
    std::vector<int> positions;
    // One bus transaction per whole group. Valid replies survive another joint's
    // timeout; missing rows are visibly offline and cannot pass Finish.
    bus(name).syncReadPositions(ids, positions);
    Session::Positions sample;
    for (size_t i = 0; i < indices.size(); ++i) {
      sample.push_back(i < positions.size() && positions[i] != -1 ?
        std::optional<int>(positions[i]) : std::nullopt);
    }
    return sample;
  }

  void refresh(const std::string & name) {session_->observe(name, read_positions(name));}

  void persist()
  {
    session_->save(session_file_);
    last_save_ = std::chrono::steady_clock::now();
    persistence_error_.clear();
  }

  void sample_range()
  {
    if (!session_->recording()) {return;}
    try {
      refresh(session_->active_group());
      if (std::chrono::steady_clock::now() - last_save_ >= 1s) {persist();}
    } catch (const std::exception & error) {
      const auto group = session_->active_group();
      session_->pause_range(group);
      persistence_error_ = std::string("recording paused; session save/read failed: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", persistence_error_.c_str());
    }
  }

  void release(const std::string & name)
  {
    session_->check_group_change(name);
    if (session_->recording()) {throw std::runtime_error("pause recording before releasing a group");}
    bool success = true;
    std::string failed;
    for (const auto i : session_->group(name)) {
      const auto & item = Session::specs()[i];
      // Explicit release only: arm IDs 1..6, head IDs 7..8, never wheels 9/10.
      // Do not initialize mode, write homing offsets, or enable torque here.
      if (!bus(name).enableTorque(item.id, false)) {
        success = false;
        failed += (failed.empty() ? "" : ", ") + item.key();
      }
    }
    if (!success) {
      released_groups_.erase(name);
      throw std::runtime_error("torque release incomplete; support the group and retry: " + failed);
    }
    released_groups_.insert(name);
    refresh(name);
  }

  void require_release(const std::string & name) const
  {
    if (!released_groups_.count(name)) {
      throw std::runtime_error("support this group and explicitly release its torque first");
    }
  }

  void response_state(Service::Response & response)
  {
    response.phase = session_->phase();
    response.active_group = session_->active_group();
    response.completed_groups = session_->completed_groups();
    response.released_groups.assign(released_groups_.begin(), released_groups_.end());
    response.session_uri = "file://" + std::filesystem::absolute(session_file_).string();
    if (session_->finalized()) {
      response.result_uri = "file://" + std::filesystem::absolute(output_).string();
    }
    for (size_t i = 0; i < Session::specs().size(); ++i) {
      const auto & item = Session::specs()[i];
      const auto & captured = session_->captures()[i];
      xlerobot_interfaces::msg::ServoCalibrationJoint row;
      row.name = item.key();
      row.servo_id = item.id;
      row.position = captured.position.value_or(-1);
      row.zero = captured.zero.value_or(-1);
      row.reference_zero = captured.reference_zero.value_or(-1);
      row.zero_source = captured.zero_source;
      row.raw_min = captured.raw_min.value_or(-1);
      row.raw_max = captured.raw_max.value_or(-1);
      row.coverage = session_->coverage(i);
      row.zero_captured = captured.zero.has_value();
      row.range_captured = captured.range_captured;
      row.online = captured.online;
      row.message = session_->message(i);
      response.joint_names.push_back(row.name);
      response.raw_positions.push_back(row.position);
      response.joints.push_back(std::move(row));
    }
  }

  void step(
    const std::shared_ptr<Service::Request> request,
    std::shared_ptr<Service::Response> response)
  {
    std::string message = "servo calibration step complete";
    try {
      if (request->command == Service::Request::STATUS) {
        // Read-only metadata. Web polling never starts sampling or motor writes.
        if (!persistence_error_.empty()) {message = persistence_error_;}
      } else if (request->command == Service::Request::SCAN) {
        refresh(request->group);
        message = "group positions refreshed";
      } else if (request->command == Service::Request::RELEASE_TORQUE) {
        release(request->group);
        message = "group torque released; support the arm/head while moving it";
      } else if (request->command == Service::Request::CAPTURE_ZERO) {
        session_->check_group_change(request->group);
        if (session_->recording()) {
          throw std::runtime_error("pause range recording before changing the zero pose");
        }
        require_release(request->group);
        const auto positions = read_positions(request->group);
        session_->observe(request->group, positions);
        session_->capture_zero(request->group, positions);
        persist();
      } else if (request->command == Service::Request::USE_EXISTING_ZERO) {
        session_->check_group_change(request->group);
        require_release(request->group);
        session_->use_existing_zero(request->group);
        persist();
        message = "existing software zeros reused; joint ranges still need recording";
      } else if (request->command == Service::Request::START_RANGE) {
        session_->check_group_change(request->group);
        require_release(request->group);
        refresh(request->group);
        session_->start_range(request->group);
        persist();
      } else if (request->command == Service::Request::FINISH_RANGE) {
        if (!session_->recording() || session_->active_group() != request->group) {
          throw std::runtime_error("this group is not recording; resume before finishing");
        }
        refresh(request->group);  // Never finish from a stale successful read.
        session_->finish_range(request->group);
        persist();
      } else if (request->command == Service::Request::PAUSE_RANGE) {
        session_->pause_range(request->group);
        persist();
      } else if (request->command == Service::Request::RESET_GROUP) {
        session_->reset_group(request->group);
        persist();
      } else if (request->command == Service::Request::FINALIZE) {
        session_->finalize(output_);
        persist();
        message = "all fourteen joints saved; the active calibration has not changed";
      } else {
        throw std::runtime_error("unknown servo calibration command");
      }
      if (request->command != Service::Request::STATUS && !persistence_error_.empty()) {
        throw std::runtime_error(persistence_error_);
      }
      response->error.code = Error::NONE;
      response->error.message = message;
    } catch (const std::exception & error) {
      response->error.code = Error::INVALID_GOAL;
      response->error.message = error.what();
      // A rejected finish keeps sampling and preserves newly collected ranges.
      if (request->command != Service::Request::STATUS && session_->recording()) {
        try {persist();} catch (const std::exception & save_error) {
          session_->pause_range(session_->active_group());
          persistence_error_ = std::string("session save failed: ") + save_error.what();
          response->error.message += "; " + persistence_error_;
        }
      }
    }
    response_state(*response);
  }

  Bus right_;
  Bus left_;
  std::string output_;
  std::filesystem::path session_file_;
  std::unique_ptr<Session> session_;
  std::set<std::string> released_groups_;
  std::chrono::steady_clock::time_point last_save_{};
  std::string persistence_error_;
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
