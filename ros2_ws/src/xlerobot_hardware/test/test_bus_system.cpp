#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "xlerobot_hardware/bus_system.hpp"
#include "xlerobot_hardware/wheel_servo_codec.hpp"

namespace xlerobot_hardware
{
namespace
{

hardware_interface::ComponentInfo wheel(const std::string & name)
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  hardware_interface::InterfaceInfo command;
  command.name = hardware_interface::HW_IF_VELOCITY;
  joint.command_interfaces.push_back(command);
  hardware_interface::InterfaceInfo position;
  position.name = hardware_interface::HW_IF_POSITION;
  joint.state_interfaces.push_back(position);
  hardware_interface::InterfaceInfo velocity;
  velocity.name = hardware_interface::HW_IF_VELOCITY;
  joint.state_interfaces.push_back(velocity);
  joint.parameters["servo_id"] = name == "left_wheel_joint" ? "9" : "10";
  return joint;
}

hardware_interface::ComponentInfo position_joint(
  const std::string & name, int servo_id, double initial = 0.0)
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  hardware_interface::InterfaceInfo command;
  command.name = hardware_interface::HW_IF_POSITION;
  joint.command_interfaces.push_back(command);
  hardware_interface::InterfaceInfo position;
  position.name = hardware_interface::HW_IF_POSITION;
  position.parameters["initial_value"] = std::to_string(initial);
  joint.state_interfaces.push_back(position);
  hardware_interface::InterfaceInfo velocity;
  velocity.name = hardware_interface::HW_IF_VELOCITY;
  joint.state_interfaces.push_back(velocity);
  joint.parameters["servo_id"] = std::to_string(servo_id);
  joint.parameters["offset"] = "2048";
  joint.parameters["direction"] = "1";
  joint.parameters["raw_min"] = "0";
  joint.parameters["raw_max"] = "4095";
  joint.parameters["limit_min"] = "-1.5";
  joint.parameters["limit_max"] = "1.5";
  return joint;
}

hardware_interface::HardwareInfo info(bool mock_hardware, bool hardware_enabled)
{
  hardware_interface::HardwareInfo result;
  result.name = "right_bus_system";
  result.type = "system";
  result.hardware_plugin_name = "xlerobot_hardware/RightBusSystem";
  result.hardware_parameters["mock_hardware"] = mock_hardware ? "true" : "false";
  result.hardware_parameters["hardware_enabled"] = hardware_enabled ? "true" : "false";
  result.hardware_parameters["torque_enabled"] = "false";
  result.hardware_parameters["read_only"] = "false";
  result.joints = {
    wheel("left_wheel_joint"), wheel("right_wheel_joint"),
    position_joint("right_arm_shoulder_pan", 1),
    position_joint("right_arm_shoulder_lift", 2),
    position_joint("right_arm_elbow_flex", 3),
    position_joint("right_arm_wrist_flex", 4),
    position_joint("right_arm_wrist_roll", 5),
    position_joint("right_arm_gripper", 6, 0.0)};
  return result;
}

hardware_interface::HardwareInfo left_info()
{
  hardware_interface::HardwareInfo result;
  result.name = "left_bus_system";
  result.type = "system";
  result.hardware_plugin_name = "xlerobot_hardware/LeftBusSystem";
  result.hardware_parameters["mock_hardware"] = "true";
  result.hardware_parameters["hardware_enabled"] = "false";
  result.hardware_parameters["torque_enabled"] = "false";
  result.hardware_parameters["read_only"] = "false";
  result.joints = {
    position_joint("head_pan_joint", 7), position_joint("head_tilt_joint", 8),
    position_joint("left_arm_shoulder_pan", 1),
    position_joint("left_arm_shoulder_lift", 2),
    position_joint("left_arm_elbow_flex", 3),
    position_joint("left_arm_wrist_flex", 4),
    position_joint("left_arm_wrist_roll", 5),
    position_joint("left_arm_gripper", 6, 0.05)};
  return result;
}

hardware_interface::HardwareInfo leader_info()
{
  hardware_interface::HardwareInfo result;
  result.name = "leader_bus_system";
  result.type = "system";
  result.hardware_plugin_name = "xlerobot_hardware/LeaderBusSystem";
  result.hardware_parameters["mock_hardware"] = "true";
  result.hardware_parameters["hardware_enabled"] = "false";
  result.hardware_parameters["torque_enabled"] = "false";
  result.hardware_parameters["runtime_torque_control"] = "true";
  result.hardware_parameters["read_only"] = "false";
  result.joints = {
    position_joint("leader_shoulder_pan", 1),
    position_joint("leader_shoulder_lift", 2),
    position_joint("leader_elbow_flex", 3),
    position_joint("leader_wrist_flex", 4),
    position_joint("leader_wrist_roll", 5),
    position_joint("leader_gripper", 6, 0.0)};
  return result;
}

hardware_interface::HardwareInfo head_info()
{
  auto result = left_info();
  result.hardware_parameters["head_only_control"] = "true";
  result.joints.resize(2);
  return result;
}

struct BusTrace
{
  std::vector<uint8_t> addressed_ids;
  std::vector<std::string> events;
  std::vector<std::vector<xlerobot_feetech::PositionCommand>> position_writes;
  std::string port;
  int velocity_calls = 0;
};

// Exercises the real hardware branch without opening any serial device.
class RecordingBus final : public xlerobot_feetech::FeetechBus
{
public:
  explicit RecordingBus(std::shared_ptr<BusTrace> trace) : trace_(std::move(trace)) {}
  bool connect(const std::string & port, int) override
  {trace_->port = port; connected_ = true; return true;}
  void disconnect() override {connected_ = false;}
  bool isConnected() const override {return connected_;}
  bool ping(uint8_t id) override {return record(id, "ping");}
  bool initVelocityMotor(uint8_t id, bool) override
  {++trace_->velocity_calls; return record(id, "wheel_init");}
  bool initPositionMotor(uint8_t id, bool enabled) override
  {EXPECT_FALSE(enabled); return record(id, "position_init");}
  bool enableTorque(uint8_t id, bool enabled) override
  {return record(id, enabled ? "torque_on" : "torque_off");}
  bool syncWriteVelocity(uint8_t left, int16_t, uint8_t right, int16_t, uint8_t) override
  {++trace_->velocity_calls; record(left, "wheel_write"); return record(right, "wheel_write");}
  std::optional<xlerobot_feetech::WheelVelocitySteps> syncReadVelocity(
    uint8_t left, uint8_t right) override
  {
    ++trace_->velocity_calls;
    record(left, "wheel_read"); record(right, "wheel_read");
    return xlerobot_feetech::WheelVelocitySteps{};
  }
  bool syncWritePosition(const std::vector<xlerobot_feetech::PositionCommand> & commands) override
  {
    trace_->position_writes.push_back(commands);
    for (const auto & command : commands) {record(command.id, "position_write");}
    return true;
  }
  bool syncReadPositions(const std::vector<uint8_t> & ids, std::vector<int> & positions) override
  {
    positions.clear();
    for (const auto id : ids) {positions.push_back(*readPosition(id));}
    return true;
  }
  std::optional<int> readPosition(uint8_t id) override
  {record(id, "position_read"); return id == 7 ? 2250 : 2500;}

private:
  bool record(uint8_t id, const std::string & event)
  {trace_->addressed_ids.push_back(id); trace_->events.push_back(event); return true;}
  std::shared_ptr<BusTrace> trace_;
  bool connected_ = false;
};

class HeadBusUnderTest final : public BusSystemBase
{
public:
  explicit HeadBusUnderTest(std::shared_ptr<BusTrace> trace)
  : BusSystemBase("left", {}), trace_(std::move(trace)) {}
protected:
  std::unique_ptr<xlerobot_feetech::FeetechBus> make_bus() override
  {return std::make_unique<RecordingBus>(trace_);}
private:
  std::shared_ptr<BusTrace> trace_;
};

class ArmBusUnderTest final : public BusSystemBase
{
public:
  explicit ArmBusUnderTest(std::shared_ptr<BusTrace> trace)
  : BusSystemBase("right", {}), trace_(std::move(trace)) {}
protected:
  std::unique_ptr<xlerobot_feetech::FeetechBus> make_bus() override
  {return std::make_unique<RecordingBus>(trace_);}
private:
  std::shared_ptr<BusTrace> trace_;
};

hardware_interface::CallbackReturn initialize(
  BusSystemBase & system, const hardware_interface::HardwareInfo & hardware_info)
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info;
  return system.on_init(params);
}

TEST(WheelServoCodecTest, PreservesVerifiedWheelDirections)
{
  WheelServoCodec codec(true, false, true, false, 3000);

  const auto encoded = codec.encode(1.0, 1.0);

  EXPECT_LT(encoded.left_steps, 0);
  EXPECT_GT(encoded.right_steps, 0);
  EXPECT_NEAR(codec.decode_left(encoded.left_steps), 1.0, 0.002);
  EXPECT_NEAR(codec.decode_right(encoded.right_steps), 1.0, 0.002);
}

TEST(WheelServoCodecTest, ScalesPairWithoutChangingRatio)
{
  WheelServoCodec codec(false, false, false, false, 1000);

  const auto encoded = codec.encode(4.0, 2.0);

  EXPECT_TRUE(encoded.scaled);
  EXPECT_EQ(encoded.left_steps, 1000);
  EXPECT_NEAR(
    static_cast<double>(encoded.right_steps) / encoded.left_steps, 0.5, 0.002);
}

TEST(RightBusSystemTest, MockLifecycleNeverNeedsHardwareEnable)
{
  RightBusSystem system;
  ASSERT_EQ(initialize(system, info(true, false)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  ASSERT_EQ(commands.size(), 8u);
  ASSERT_EQ(states.size(), 16u);
  ASSERT_TRUE(commands[0].set_value(1.0));
  ASSERT_TRUE(commands[1].set_value(2.0));
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  const auto left_position = states[0].get_optional<double>();
  const auto right_position = states[2].get_optional<double>();
  ASSERT_TRUE(left_position.has_value());
  ASSERT_TRUE(right_position.has_value());
  EXPECT_NEAR(*left_position, 0.1, 1e-12);
  EXPECT_NEAR(*right_position, 0.2, 1e-12);
}

TEST(LeftBusSystemTest, MockOwnsHeadAndLeftArmTogether)
{
  LeftBusSystem system;
  ASSERT_EQ(initialize(system, left_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()),
        hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
        hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_command_interfaces().size(), 8u);
  EXPECT_EQ(system.export_state_interfaces().size(), 16u);
}

TEST(HeadOnlyBusTest, IsOptInAndRejectsEveryOtherBusOrJointLayout)
{
  {
    LeftBusSystem system;
    auto config = head_info();
    config.hardware_parameters.erase("head_only_control");
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  {
    LeftBusSystem system;
    auto config = left_info();
    config.hardware_parameters["head_only_control"] = "true";
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  for (const int id : {1, 6, 8, 9, 10, 253}) {
    LeftBusSystem system;
    auto config = head_info();
    config.joints[0].parameters["servo_id"] = std::to_string(id);
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  {
    LeftBusSystem system;
    auto config = head_info();
    config.joints[1].name = "left_arm_gripper";
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  {
    LeftBusSystem system;
    auto config = head_info();
    config.joints[1].parameters["servo_id"] = "1";
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  {
    LeftBusSystem system;
    auto config = head_info();
    config.joints.pop_back();
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  {
    RightBusSystem system;
    auto config = info(true, false);
    config.hardware_parameters["head_only_control"] = "true";
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  {
    LeaderBusSystem system;
    auto config = leader_info();
    config.hardware_parameters["head_only_control"] = "true";
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST(HeadOnlyBusTest, ExistingLeftBusPluginAcceptsTheExactHeadSelection)
{
  LeftBusSystem system;
  ASSERT_EQ(initialize(system, head_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_command_interfaces().size(), 2u);
  EXPECT_EQ(system.export_state_interfaces().size(), 4u);
}

TEST(HeadOnlyBusTest, CompleteHardwareLifecycleOnlyAddressesIdsSevenAndEight)
{
  const auto trace = std::make_shared<BusTrace>();
  {
    HeadBusUnderTest system(trace);
    auto config = head_info();
    config.hardware_parameters["mock_hardware"] = "false";
    config.hardware_parameters["hardware_enabled"] = "true";
    config.hardware_parameters["torque_enabled"] = "true";
    config.hardware_parameters["port"] = "/dev/test-left-only";
    ASSERT_EQ(initialize(system, config), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(trace->position_writes.empty());
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    const auto commands = system.export_command_interfaces();
    ASSERT_EQ(commands.size(), 2u);
    EXPECT_EQ(commands[0].get_name(), "head_pan_joint/position");
    EXPECT_EQ(commands[1].get_name(), "head_tilt_joint/position");
    ASSERT_FALSE(trace->position_writes.empty());
    const auto & initial_hold = trace->position_writes.front();
    ASSERT_EQ(initial_hold.size(), 2u);
    EXPECT_EQ(initial_hold[0].position, 2250);
    EXPECT_EQ(initial_hold[1].position, 2500);
    const auto first_hold = std::find(trace->events.begin(), trace->events.end(), "position_write");
    const auto first_enable = std::find(trace->events.begin(), trace->events.end(), "torque_on");
    ASSERT_NE(first_enable, trace->events.end());
    EXPECT_LT(first_hold, first_enable);
    EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
      hardware_interface::return_type::OK);
    EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
      hardware_interface::return_type::OK);
    EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }
  EXPECT_EQ(trace->port, "/dev/test-left-only");
  EXPECT_EQ(trace->velocity_calls, 0);
  ASSERT_FALSE(trace->addressed_ids.empty());
  for (const auto id : trace->addressed_ids) {EXPECT_TRUE(id == 7 || id == 8);}
}

TEST(RightBusSystemTest, ReactivationNeverTurnsWheelOdometryIntoVelocity)
{
  RightBusSystem system;
  auto config = info(true, false);
  config.hardware_parameters["torque_enabled"] = "true";
  ASSERT_EQ(initialize(system, config), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  for (int attempt = 0; attempt < 3; ++attempt) {
    // Represent ordinary earlier navigation / manually rolled wheel odometry.
    ASSERT_TRUE(commands[0].set_value(1.2));
    ASSERT_TRUE(commands[1].set_value(-0.7));
    ASSERT_TRUE(commands[2].set_value(0.4));
    ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)),
      hardware_interface::return_type::OK);
    const double left_position = *states[0].get_optional<double>();
    const double right_position = *states[2].get_optional<double>();
    const double arm_position = *states[4].get_optional<double>();
    ASSERT_NE(left_position, 0.0);
    ASSERT_NE(right_position, 0.0);
    ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    // Exactly the collection Release -> Reset -> Start hardware transition.
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_DOUBLE_EQ(*commands[0].get_optional<double>(), 0.0);
    EXPECT_DOUBLE_EQ(*commands[1].get_optional<double>(), 0.0);
    EXPECT_DOUBLE_EQ(*commands[2].get_optional<double>(), arm_position);
    for (int cycle = 0; cycle < 5; ++cycle) {
      ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.02)),
        hardware_interface::return_type::OK);
      ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.02)),
        hardware_interface::return_type::OK);
    }
    EXPECT_DOUBLE_EQ(*states[0].get_optional<double>(), left_position);
    EXPECT_DOUBLE_EQ(*states[2].get_optional<double>(), right_position);
  }
}

TEST(ArmOnlyBusTest, LifecycleNeverAddressesWheelsAndHoldsBeforeTorque)
{
  const auto trace = std::make_shared<BusTrace>();
  {
    ArmBusUnderTest system(trace);
    auto config = info(false, true);
    config.joints.erase(config.joints.begin(), config.joints.begin() + 2);
    config.hardware_parameters["arm_only_control"] = "true";
    config.hardware_parameters["torque_enabled"] = "true";
    ASSERT_EQ(initialize(system, config), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.export_command_interfaces().size(), 6u);
    EXPECT_LT(std::find(trace->events.begin(), trace->events.end(), "position_write"),
      std::find(trace->events.begin(), trace->events.end(), "torque_on"));
    EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.02)), hardware_interface::return_type::OK);
    EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.02)), hardware_interface::return_type::OK);
    EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  }
  EXPECT_EQ(trace->velocity_calls, 0);
  ASSERT_FALSE(trace->addressed_ids.empty());
  for (const auto id : trace->addressed_ids) {EXPECT_TRUE(id >= 1 && id <= 6);}
}

TEST(ArmOnlyBusTest, RejectsWheelIdsAndWrongBus)
{
  auto config = info(true, false);
  config.joints.erase(config.joints.begin(), config.joints.begin() + 2);
  config.hardware_parameters["arm_only_control"] = "true";
  {
    LeftBusSystem system;
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
  for (const auto id : {7, 8, 9, 10}) {
    RightBusSystem system;
    config.joints[0].parameters["servo_id"] = std::to_string(id);
    EXPECT_EQ(initialize(system, config), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST(LeaderBusSystemTest, RuntimeTorqueKeepsLeaderStateOnItsOwnBus)
{
  LeaderBusSystem system;
  ASSERT_EQ(initialize(system, leader_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  ASSERT_EQ(commands.size(), 7u);
  ASSERT_EQ(states.size(), 12u);
  EXPECT_EQ(commands.back().get_prefix_name(), "leader_bus");
  EXPECT_EQ(commands.back().get_interface_name(), "torque_enable");

  ASSERT_TRUE(commands[0].set_value(0.4));
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*states[0].get_optional<double>(), 0.0);

  ASSERT_TRUE(commands.back().set_value(1.0));
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  // Torque enable primes the measured pose, never a stale passive target.
  EXPECT_DOUBLE_EQ(*states[0].get_optional<double>(), 0.0);
  ASSERT_TRUE(commands[0].set_value(0.4));
  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_NEAR(*states[0].get_optional<double>(), 0.05, 1e-12);
}

TEST(LeaderBusSystemTest, LifecycleRecoveryDiscardsOldTorqueLease)
{
  LeaderBusSystem system;
  ASSERT_EQ(initialize(system, leader_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_TRUE(commands.back().set_value(1.0));
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(*commands.back().get_optional<double>(), 0.0);
  ASSERT_TRUE(commands.back().set_value(1.0));
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(*commands.back().get_optional<double>(), 0.0);
  ASSERT_TRUE(commands.back().set_value(1.0));
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(*commands.back().get_optional<double>(), 0.0);
}

TEST(RightBusSystemTest, RealModeRequiresSecondHardwareEnableKey)
{
  RightBusSystem system;
  ASSERT_EQ(initialize(system, info(false, false)), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(RightBusSystemTest, RejectsUnexpectedJointContract)
{
  auto invalid = info(true, false);
  invalid.joints[0].name = "wrong_joint";
  RightBusSystem system;

  EXPECT_EQ(initialize(system, invalid), hardware_interface::CallbackReturn::ERROR);
}

TEST(RightBusSystemTest, RejectsMalformedSafetyGate)
{
  auto invalid = info(true, false);
  invalid.hardware_parameters["mock_hardware"] = "tru";
  RightBusSystem system;

  EXPECT_EQ(initialize(system, invalid), hardware_interface::CallbackReturn::ERROR);
}

TEST(RightBusSystemTest, ReadOnlyAndTorqueCannotBeEnabledTogether)
{
  auto invalid = info(true, false);
  invalid.hardware_parameters["read_only"] = "true";
  invalid.hardware_parameters["torque_enabled"] = "true";
  RightBusSystem system;

  EXPECT_EQ(initialize(system, invalid), hardware_interface::CallbackReturn::ERROR);
}

TEST(RightBusSystemTest, ReadOnlyModeIgnoresEveryCommandAtHardwareBoundary)
{
  auto read_only = info(true, false);
  read_only.hardware_parameters["read_only"] = "true";
  RightBusSystem system;
  ASSERT_EQ(initialize(system, read_only), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  ASSERT_TRUE(commands[2].set_value(100.0));

  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  ASSERT_TRUE(states[4].get_optional<double>().has_value());
  EXPECT_DOUBLE_EQ(*states[4].get_optional<double>(), 0.0);
}

TEST(RightBusSystemTest, RejectsOutOfRangePositionCommandInMockMode)
{
  RightBusSystem system;
  ASSERT_EQ(initialize(system, info(true, false)), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_TRUE(commands[2].set_value(1.6));

  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::ERROR);
}

void check_outside_startup(BusSystemBase & system,
  const hardware_interface::HardwareInfo & config, std::size_t joint_index)
{
  ASSERT_EQ(initialize(system, config), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  ASSERT_TRUE(states[joint_index * 2].set_value(1.8));
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(commands[joint_index].set_value(-1.0));
  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*states[joint_index * 2].get_optional<double>(), 1.8);
  // Manual repositioning clears the startup wait, not the old queued target.
  ASSERT_TRUE(states[joint_index * 2].set_value(1.0));
  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[joint_index].get_optional<double>(), 1.0);
}

TEST(BusStartupTest, LeaderAndBothFollowerBusesAcceptOutsideInitialObservations)
{
  LeaderBusSystem leader;
  check_outside_startup(leader, leader_info(), 2);
  RightBusSystem right;
  check_outside_startup(right, info(true, false), 4);
  LeftBusSystem left;
  check_outside_startup(left, left_info(), 4);
}

TEST(BusStartupTest, PassiveLeaderAcceptsOutsideObservationButRejectsTorqueEnable)
{
  LeaderBusSystem leader;
  ASSERT_EQ(initialize(leader, leader_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(leader.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(leader.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto states = leader.export_state_interfaces();
  auto commands = leader.export_command_interfaces();
  ASSERT_TRUE(states[4].set_value(1.8));
  EXPECT_EQ(leader.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::OK);
  ASSERT_TRUE(commands.back().set_value(1.0));
  EXPECT_EQ(leader.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
    hardware_interface::return_type::ERROR);
}

}  // namespace
}  // namespace xlerobot_hardware
