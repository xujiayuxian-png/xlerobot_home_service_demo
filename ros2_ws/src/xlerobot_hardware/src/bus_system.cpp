#include "xlerobot_hardware/bus_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "xlerobot_feetech/scservo_feetech_bus.hpp"

namespace xlerobot_hardware
{
namespace
{

bool parse_bool(
  const hardware_interface::HardwareInfo & info, const std::string & key, bool fallback,
  bool & valid)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  if (it->second == "true" || it->second == "1" || it->second == "yes") {
    return true;
  }
  if (it->second == "false" || it->second == "0" || it->second == "no") {
    return false;
  }
  valid = false;
  return fallback;
}

int parse_int(
  const hardware_interface::HardwareInfo & info, const std::string & key, int fallback,
  bool & valid)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    valid = false;
    return fallback;
  }
}

double parse_double(const std::string & value, double fallback)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

int joint_int(
  const hardware_interface::ComponentInfo & joint, const std::string & key,
  int fallback)
{
  const auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    return fallback;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    return fallback;
  }
}

double joint_double(
  const hardware_interface::ComponentInfo & joint, const std::string & key, double fallback)
{
  const auto it = joint.parameters.find(key);
  return it == joint.parameters.end() ? fallback : parse_double(it->second, fallback);
}

double initial_position(const hardware_interface::ComponentInfo & joint)
{
  for (const auto & state : joint.state_interfaces) {
    if (state.name != hardware_interface::HW_IF_POSITION) {
      continue;
    }
    const auto it = state.parameters.find("initial_value");
    if (it != state.parameters.end()) {
      return parse_double(it->second, 0.0);
    }
  }
  return 0.0;
}

bool has_joint_state(const hardware_interface::ComponentInfo & joint, const std::string & name)
{
  return std::any_of(
    joint.state_interfaces.begin(), joint.state_interfaces.end(),
    [&name](const auto & interface) {return interface.name == name;});
}

}  // namespace

BusSystemBase::BusSystemBase(std::string bus_name, std::vector<std::string> expected_joints)
: bus_name_(std::move(bus_name)), expected_joints_(std::move(expected_joints))
{
  port_ = bus_name_ == "right" ? "/dev/right_arm" : "/dev/left_arm";
}

BusSystemBase::~BusSystemBase()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (bus_ && bus_->isConnected()) {
    if (!read_only_) {
      stop_wheels();
      disable_all_torque();
    }
    disconnect();
  }
}

hardware_interface::CallbackReturn BusSystemBase::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & info = params.hardware_info;
  bool parameters_valid = true;
  mock_hardware_ = parse_bool(info, "mock_hardware", true, parameters_valid);
  hardware_enabled_ = parse_bool(info, "hardware_enabled", false, parameters_valid);
  torque_enabled_ = parse_bool(info, "torque_enabled", false, parameters_valid);
  runtime_torque_control_ = parse_bool(
    info, "runtime_torque_control", false, parameters_valid);
  torque_command_ = torque_enabled_ ? 1.0 : 0.0;
  applied_torque_enabled_ = false;
  read_only_ = parse_bool(info, "read_only", false, parameters_valid);
  head_only_control_ = parse_bool(info, "head_only_control", false, parameters_valid);
  arm_only_control_ = parse_bool(info, "arm_only_control", false, parameters_valid);
  const auto port = info.hardware_parameters.find("port");
  if (port != info.hardware_parameters.end() && !port->second.empty()) {
    port_ = port->second;
  }
  baudrate_ = parse_int(info, "baudrate", baudrate_, parameters_valid);
  wheel_acceleration_ = static_cast<uint8_t>(std::clamp(
      parse_int(info, "wheel_acceleration", wheel_acceleration_, parameters_valid), 0, 254));
  position_speed_ = static_cast<uint16_t>(std::clamp(
      parse_int(info, "position_speed", position_speed_, parameters_valid), 1, 32767));
  position_acceleration_ = static_cast<uint8_t>(std::clamp(
      parse_int(info, "position_acceleration", position_acceleration_, parameters_valid), 0, 254));
  mock_position_step_ = std::max(
    0.001,
    static_cast<double>(
      parse_int(info, "mock_position_step_millirad", 50, parameters_valid)) / 1000.0);
  wheel_codec_ = std::make_unique<WheelServoCodec>(
    parse_bool(info, "invert_left_wheel", true, parameters_valid),
    parse_bool(info, "invert_right_wheel", false, parameters_valid),
    parse_bool(info, "feedback_invert_left_wheel", true, parameters_valid),
    parse_bool(info, "feedback_invert_right_wheel", false, parameters_valid),
    parse_int(info, "max_raw_velocity", 3000, parameters_valid));

  if (!parameters_valid || baudrate_ <= 0 || (read_only_ && torque_enabled_) ||
    (head_only_control_ && bus_name_ != "left") ||
    (arm_only_control_ && bus_name_ != "right") ||
    !parse_and_validate_joints())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("xlerobot_hardware"),
      "%s bus contains an invalid hardware parameter", bus_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("xlerobot_hardware"),
    "Initialized %s bus owner with %zu joints "
    "(mock=%s hardware_enabled=%s torque=%s read_only=%s)",
    bus_name_.c_str(), joints_.size(), mock_hardware_ ? "true" : "false",
    hardware_enabled_ ? "true" : "false", torque_enabled_ ? "true" : "false",
    read_only_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BusSystemBase::on_configure(const rclcpp_lifecycle::State &)
{
  // Recovery must not inherit an earlier torque lease or controller NaN.
  if (runtime_torque_control_) {torque_command_ = 0.0;}
  applied_torque_enabled_ = false;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    positions_[i] = joints_[i].initial_position;
    previous_positions_[i] = positions_[i];
    velocities_[i] = 0.0;
    commands_[i] = joints_[i].position_codec ? positions_[i] : 0.0;
  }
  if (mock_hardware_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  if (!hardware_enabled_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("xlerobot_hardware"),
      "Refusing to open %s: real mode also requires hardware_enabled=true", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (!connect_and_probe() ||
    (runtime_torque_control_ && !disable_all_torque()) || !read_position_joints(0.01))
  {
    disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  reset_commands_to_hold();
  previous_positions_ = positions_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BusSystemBase::on_activate(const rclcpp_lifecycle::State &)
{
  if (runtime_torque_control_) {torque_command_ = 0.0;}
  reset_commands_to_hold();
  startup_position_pending_ = !validate_commands();
  if (startup_position_pending_) {
    RCLCPP_WARN(rclcpp::get_logger("xlerobot_hardware"),
      "%s bus started in observation-only mode: initial pose is outside command limits; "
      "torque stays off until measured joints return within limits", bus_name_.c_str());
  }
  if (mock_hardware_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (read_only_) {
    return bus_ && bus_->isConnected() ? hardware_interface::CallbackReturn::SUCCESS :
           hardware_interface::CallbackReturn::ERROR;
  }
  const bool prepared = bus_ && bus_->isConnected() && initialize_motors() &&
    stop_wheels() && (startup_position_pending_ || write_position_commands(positions_));
  const bool torque_ready = prepared &&
    (startup_position_pending_ || !torque_enabled_ || set_all_torque(true));
  if (!torque_ready) {
    stop_wheels();
    disable_all_torque();
    return hardware_interface::CallbackReturn::ERROR;
  }
  applied_torque_enabled_ = torque_enabled_ && !startup_position_pending_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BusSystemBase::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (runtime_torque_control_) {torque_command_ = 0.0;}
  applied_torque_enabled_ = false;
  std::fill(velocities_.begin(), velocities_.end(), 0.0);
  const auto left = find_joint("left_wheel_joint");
  const auto right = find_joint("right_wheel_joint");
  if (left < commands_.size()) {commands_[left] = 0.0;}
  if (right < commands_.size()) {commands_[right] = 0.0;}
  if (mock_hardware_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (read_only_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  const bool stopped = stop_wheels();
  const bool torque_off = disable_all_torque();
  return stopped && torque_off ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn BusSystemBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (bus_ && bus_->isConnected()) {
    if (!read_only_) {
      stop_wheels();
      disable_all_torque();
    }
  }
  disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BusSystemBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  result.reserve(joints_.size() * 2);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    result.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]);
    result.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> BusSystemBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  result.reserve(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    result.emplace_back(joints_[i].name, joints_[i].command_interface, &commands_[i]);
  }
  if (runtime_torque_control_) {
    result.emplace_back(bus_name_ + "_bus", "torque_enable", &torque_command_);
  }
  return result;
}

hardware_interface::return_type BusSystemBase::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = std::max(1.0e-6, period.seconds());
  previous_positions_ = positions_;
  if (mock_hardware_) {
    if (read_only_ || startup_position_pending_) {
      std::fill(velocities_.begin(), velocities_.end(), 0.0);
      return hardware_interface::return_type::OK;
    }
    if (runtime_torque_control_ && torque_command_ < 0.5) {
      std::fill(velocities_.begin(), velocities_.end(), 0.0);
      return hardware_interface::return_type::OK;
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i].command_interface == hardware_interface::HW_IF_VELOCITY) {
        velocities_[i] = commands_[i];
        positions_[i] += velocities_[i] * dt;
      } else {
        const double step = std::clamp(
          commands_[i] - positions_[i], -mock_position_step_, mock_position_step_);
        positions_[i] += step;
        velocities_[i] = step / dt;
      }
    }
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (!bus_ || !bus_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }
  const auto left = find_joint("left_wheel_joint");
  const auto right = find_joint("right_wheel_joint");
  if (left < joints_.size() && right < joints_.size() && wheel_feedback_available_) {
    const auto feedback = bus_->syncReadVelocity(joints_[left].servo_id, joints_[right].servo_id);
    if (!feedback) {
      if (!read_only_) {stop_wheels();}
      return hardware_interface::return_type::ERROR;
    }
    velocities_[left] = wheel_codec_->decode_left(feedback->left);
    velocities_[right] = wheel_codec_->decode_right(feedback->right);
    positions_[left] += velocities_[left] * dt;
    positions_[right] += velocities_[right] * dt;
  }
  if (!read_position_joints(dt)) {
    if (!read_only_) {stop_wheels();}
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BusSystemBase::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (read_only_) {
    return hardware_interface::return_type::OK;
  }
  if (runtime_torque_control_ && !std::isfinite(torque_command_)) {
    return hardware_interface::return_type::ERROR;
  }
  if (startup_position_pending_) {
    // Never turn an out-of-range observation into a clamped motor target.
    // Discard queued commands while waiting, including wheel commands.
    reset_commands_to_hold();
    if (!validate_commands()) {return hardware_interface::return_type::OK;}
    if (!mock_hardware_) {
      std::lock_guard<std::mutex> lock(bus_mutex_);
      if (!write_position_commands(positions_) ||
        (torque_enabled_ && !set_all_torque(true)))
      {
        return hardware_interface::return_type::ERROR;
      }
    }
    applied_torque_enabled_ = torque_enabled_;
    startup_position_pending_ = false;
    RCLCPP_INFO(rclcpp::get_logger("xlerobot_hardware"),
      "%s bus initial pose is now within limits; holding measured pose", bus_name_.c_str());
    return hardware_interface::return_type::OK;
  }
  // Passive Leader observations are not motor commands. Release torque before
  // considering any stale position target left by the trajectory controller.
  if (runtime_torque_control_ && std::isfinite(torque_command_) && torque_command_ < 0.5) {
    if (!mock_hardware_ && applied_torque_enabled_) {
      std::lock_guard<std::mutex> lock(bus_mutex_);
      if (!set_all_torque(false)) {return hardware_interface::return_type::ERROR;}
    }
    applied_torque_enabled_ = false;
    reset_commands_to_hold();
    return hardware_interface::return_type::OK;
  }
  if (runtime_torque_control_ && !applied_torque_enabled_) {
    reset_commands_to_hold();
  }
  if (!validate_commands()) {
    if (!mock_hardware_) {
      std::lock_guard<std::mutex> lock(bus_mutex_);
      stop_wheels();
    }
    RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"), "%s bus rejected unsafe command",
        bus_name_.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (mock_hardware_) {
    applied_torque_enabled_ = !runtime_torque_control_ || torque_command_ >= 0.5;
    return hardware_interface::return_type::OK;
  }
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (!bus_ || !bus_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }
  if (runtime_torque_control_) {
    const bool requested = torque_command_ >= 0.5;
    if (requested != applied_torque_enabled_) {
      if (requested && !write_position_commands(positions_)) {
        return hardware_interface::return_type::ERROR;
      }
      if (!set_all_torque(requested)) {
        return hardware_interface::return_type::ERROR;
      }
      applied_torque_enabled_ = requested;
      reset_commands_to_hold();
    }
    if (!applied_torque_enabled_) {
      return hardware_interface::return_type::OK;
    }
  } else if (!torque_enabled_) {
    bool movement_requested = false;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      movement_requested |= joints_[i].command_interface == hardware_interface::HW_IF_VELOCITY ?
        std::abs(commands_[i]) > 1.0e-9 : std::abs(commands_[i] - positions_[i]) > 1.0e-6;
    }
    if (movement_requested) {
      stop_wheels();
      RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"),
          "Command rejected: torque_enabled=false");
      return hardware_interface::return_type::ERROR;
    }
    return stop_wheels() ? hardware_interface::return_type::OK :
           hardware_interface::return_type::ERROR;
  }

  const auto left = find_joint("left_wheel_joint");
  const auto right = find_joint("right_wheel_joint");
  if (left < joints_.size() && right < joints_.size()) {
    const auto encoded = wheel_codec_->encode(commands_[left], commands_[right]);
    if (!bus_->syncWriteVelocity(
        joints_[left].servo_id, encoded.left_steps, joints_[right].servo_id,
        encoded.right_steps, wheel_acceleration_))
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!write_position_commands(commands_)) {
    stop_wheels();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool BusSystemBase::parse_and_validate_joints()
{
  // Narrow selections of the same bus owners, not a second hardware path.
  // Exact joints and IDs prevent either selection from addressing other motors.
  const std::vector<std::string> head_joints{"head_pan_joint", "head_tilt_joint"};
  const std::vector<std::string> arm_joints{
    "right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
    "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"};
  const auto & expected = head_only_control_ ? head_joints :
    (arm_only_control_ ? arm_joints : expected_joints_);
  if (info_.joints.size() != expected.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"), "%s bus expected %zu joints, got %zu",
      bus_name_.c_str(), expected.size(), info_.joints.size());
    return false;
  }
  std::set<uint8_t> ids;
  joints_.clear();
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    if (joint.name != expected[i] || joint.command_interfaces.size() != 1 ||
      !has_joint_state(joint, hardware_interface::HW_IF_POSITION) ||
      !has_joint_state(joint, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"), "Invalid joint contract at index %zu",
          i);
      return false;
    }
    const bool wheel = joint.name == "left_wheel_joint" || joint.name == "right_wheel_joint";
    const std::string expected_interface = wheel ? hardware_interface::HW_IF_VELOCITY :
      hardware_interface::HW_IF_POSITION;
    const auto id_value = joint_int(joint, "servo_id", 0);
    if (joint.command_interfaces[0].name != expected_interface || id_value < 1 || id_value > 253 ||
      (head_only_control_ && id_value != static_cast<int>(7 + i)) ||
      (arm_only_control_ && id_value != static_cast<int>(1 + i)) ||
      !ids.insert(static_cast<uint8_t>(id_value)).second)
    {
      RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"), "Invalid interface or servo id for %s",
          joint.name.c_str());
      return false;
    }
    JointConfig config;
    config.name = joint.name;
    config.command_interface = expected_interface;
    config.servo_id = static_cast<uint8_t>(id_value);
    config.initial_position = initial_position(joint);
    if (!wheel) {
      PositionServoCalibration calibration;
      calibration.offset = joint_int(joint, "offset", -1);
      calibration.direction = joint_int(joint, "direction", 0);
      calibration.raw_min = joint_int(joint, "raw_min", -1);
      calibration.raw_max = joint_int(joint, "raw_max", -1);
      calibration.position_min = joint_double(joint, "limit_min",
          std::numeric_limits<double>::quiet_NaN());
      calibration.position_max = joint_double(joint, "limit_max",
          std::numeric_limits<double>::quiet_NaN());
      config.position_codec = std::make_unique<CalibratedServoCodec>(calibration);
      if (!config.position_codec->valid() ||
        !config.position_codec->encode(config.initial_position))
      {
        RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"), "Invalid calibration for %s",
            joint.name.c_str());
        return false;
      }
    }
    joints_.push_back(std::move(config));
  }
  positions_.assign(joints_.size(), 0.0);
  previous_positions_ = positions_;
  velocities_ = positions_;
  reset_commands_to_hold();
  return true;
}

std::unique_ptr<xlerobot_feetech::FeetechBus> BusSystemBase::make_bus()
{
  return std::make_unique<xlerobot_feetech::ScServoFeetechBus>();
}

bool BusSystemBase::connect_and_probe()
{
  bus_ = make_bus();
  if (!bus_->connect(port_, baudrate_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("xlerobot_hardware"), "%s bus could not open %s",
      bus_name_.c_str(), port_.c_str());
    return false;
  }
  wheel_feedback_available_ = true;
  for (const auto & joint : joints_) {
    if (!bus_->ping(joint.servo_id)) {
      if (!joint.position_codec) {
        wheel_feedback_available_ = false;
        RCLCPP_WARN(
          rclcpp::get_logger("xlerobot_hardware"),
          "%s bus wheel servo %u did not respond; wheel feedback is unavailable",
          bus_name_.c_str(), joint.servo_id);
        continue;
      }
      RCLCPP_ERROR(rclcpp::get_logger("xlerobot_hardware"), "%s bus servo %u did not respond",
        bus_name_.c_str(), joint.servo_id);
      return false;
    }
  }
  return true;
}

bool BusSystemBase::read_position_joints(double dt)
{
  std::vector<uint8_t> ids;
  std::vector<std::size_t> joint_indices;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].position_codec) {
      ids.push_back(joints_[i].servo_id);
      joint_indices.push_back(i);
    }
  }
  if (ids.empty()) {return true;}

  std::vector<int> raw_positions;
  if (!bus_->syncReadPositions(ids, raw_positions) || raw_positions.size() != ids.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("xlerobot_hardware"),
      "%s bus could not batch-read position servos", bus_name_.c_str());
    return false;
  }
  for (std::size_t index = 0; index < joint_indices.size(); ++index) {
    const auto joint_index = joint_indices[index];
    const auto decoded = joints_[joint_index].position_codec->decode_observation(
      raw_positions[index]);
    if (!decoded) {
      RCLCPP_ERROR(
        rclcpp::get_logger("xlerobot_hardware"),
        "%s bus servo %u returned an invalid calibrated position",
        bus_name_.c_str(), joints_[joint_index].servo_id);
      return false;
    }
    positions_[joint_index] = *decoded;
    velocities_[joint_index] =
      (positions_[joint_index] - previous_positions_[joint_index]) / dt;
  }
  return true;
}

bool BusSystemBase::initialize_motors()
{
  for (const auto & joint : joints_) {
    const bool ok = joint.position_codec ?
      bus_->initPositionMotor(joint.servo_id, false) :
      bus_->initVelocityMotor(joint.servo_id, false);
    if (!ok) {
      RCLCPP_ERROR(
        rclcpp::get_logger("xlerobot_hardware"),
        "%s bus could not initialize servo %u", bus_name_.c_str(), joint.servo_id);
      return false;
    }
  }
  return true;
}

bool BusSystemBase::write_position_commands(const std::vector<double> & positions)
{
  std::vector<xlerobot_feetech::PositionCommand> position_commands;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (!joints_[i].position_codec) {
      continue;
    }
    const auto raw = joints_[i].position_codec->encode(positions[i]);
    if (!raw) {
      RCLCPP_ERROR(
        rclcpp::get_logger("xlerobot_hardware"),
        "%s bus cannot encode %s (servo %u) position %.6f rad within calibrated "
        "command limits; at startup, reposition the unpowered joint within its range",
        bus_name_.c_str(), joints_[i].name.c_str(), joints_[i].servo_id, positions[i]);
      return false;
    }
    position_commands.push_back({
        joints_[i].servo_id, static_cast<int16_t>(*raw), position_speed_, position_acceleration_});
  }
  return position_commands.empty() || bus_->syncWritePosition(position_commands);
}

bool BusSystemBase::stop_wheels()
{
  const auto left = find_joint("left_wheel_joint");
  const auto right = find_joint("right_wheel_joint");
  if (left == joints_.size() && right == joints_.size()) {return true;}
  if (left == joints_.size() || right == joints_.size() || !bus_ || !bus_->isConnected()) {
    return false;
  }
  return bus_->syncWriteVelocity(
    joints_[left].servo_id, 0, joints_[right].servo_id, 0, wheel_acceleration_);
}

bool BusSystemBase::disable_all_torque()
{
  return set_all_torque(false);
}

bool BusSystemBase::set_all_torque(bool enabled)
{
  if (!bus_ || !bus_->isConnected()) {return false;}
  bool ok = true;
  for (const auto & joint : joints_) {
    bool changed = false;
    for (int attempt = 0; attempt < 3 && !changed; ++attempt) {
      changed = bus_->enableTorque(joint.servo_id, enabled);
      if (!changed && attempt < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if (!changed) {
      RCLCPP_ERROR(
        rclcpp::get_logger("xlerobot_hardware"),
        "%s bus could not set torque=%s on servo %u", bus_name_.c_str(),
        enabled ? "true" : "false", joint.servo_id);
    }
    ok = changed && ok;
  }
  return ok;
}

void BusSystemBase::reset_commands_to_hold()
{
  commands_.resize(positions_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    // Arms hold measured POSITION. Wheels must hold zero VELOCITY, never
    // their accumulated odometry. This also applies without a base controller.
    commands_[i] = joints_[i].position_codec ? positions_[i] : 0.0;
  }
}

bool BusSystemBase::validate_commands() const
{
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (!std::isfinite(commands_[i])) {return false;}
    if (joints_[i].position_codec && !joints_[i].position_codec->encode(commands_[i])) {
      return false;
    }
  }
  return true;
}

void BusSystemBase::disconnect()
{
  if (bus_) {bus_->disconnect(); bus_.reset();}
}

std::size_t BusSystemBase::find_joint(const std::string & name) const
{
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].name == name) {
      return i;
    }
  }
  return joints_.size();
}

RightBusSystem::RightBusSystem()
: BusSystemBase("right", {
    "left_wheel_joint", "right_wheel_joint", "right_arm_shoulder_pan",
    "right_arm_shoulder_lift", "right_arm_elbow_flex", "right_arm_wrist_flex",
    "right_arm_wrist_roll", "right_arm_gripper"})
{
}

LeftBusSystem::LeftBusSystem()
: BusSystemBase("left", {
    "head_pan_joint", "head_tilt_joint", "left_arm_shoulder_pan",
    "left_arm_shoulder_lift", "left_arm_elbow_flex", "left_arm_wrist_flex",
    "left_arm_wrist_roll", "left_arm_gripper"})
{
}

LeaderBusSystem::LeaderBusSystem()
: BusSystemBase("leader", {
    "leader_shoulder_pan", "leader_shoulder_lift", "leader_elbow_flex",
    "leader_wrist_flex", "leader_wrist_roll", "leader_gripper"})
{
}

}  // namespace xlerobot_hardware

PLUGINLIB_EXPORT_CLASS(xlerobot_hardware::RightBusSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(xlerobot_hardware::LeftBusSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(xlerobot_hardware::LeaderBusSystem, hardware_interface::SystemInterface)
