#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "xlerobot_feetech/feetech_bus.hpp"
#include "xlerobot_hardware/calibrated_servo_codec.hpp"
#include "xlerobot_hardware/wheel_servo_codec.hpp"

namespace xlerobot_hardware
{

class BusSystemBase : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  BusSystemBase(std::string bus_name, std::vector<std::string> expected_joints);
  ~BusSystemBase() override;
  virtual std::unique_ptr<xlerobot_feetech::FeetechBus> make_bus();

private:
  struct JointConfig
  {
    std::string name;
    std::string command_interface;
    uint8_t servo_id = 0;
    std::unique_ptr<CalibratedServoCodec> position_codec;
    double initial_position = 0.0;
  };

  bool parse_and_validate_joints();
  bool connect_and_probe();
  bool read_position_joints(double dt);
  bool initialize_motors();
  bool write_position_commands(const std::vector<double> & positions);
  bool stop_wheels();
  bool set_all_torque(bool enabled);
  bool disable_all_torque();
  bool validate_commands() const;
  void reset_commands_to_hold();
  void disconnect();
  std::size_t find_joint(const std::string & name) const;

  std::string bus_name_;
  std::vector<std::string> expected_joints_;
  bool mock_hardware_{true};
  bool hardware_enabled_{false};
  bool torque_enabled_{false};
  bool applied_torque_enabled_{false};
  bool startup_position_pending_{false};
  bool runtime_torque_control_{false};
  double torque_command_{0.0};
  bool read_only_{false};
  bool head_only_control_{false};
  bool wheel_feedback_available_{false};
  std::string port_;
  int baudrate_{1000000};
  uint8_t wheel_acceleration_{254};
  uint16_t position_speed_{500};
  uint8_t position_acceleration_{50};
  double mock_position_step_{0.05};
  std::unique_ptr<WheelServoCodec> wheel_codec_;
  std::unique_ptr<xlerobot_feetech::FeetechBus> bus_;
  std::mutex bus_mutex_;
  std::vector<JointConfig> joints_;
  std::vector<double> positions_;
  std::vector<double> previous_positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
};

class RightBusSystem final : public BusSystemBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RightBusSystem)
  RightBusSystem();
};

class LeftBusSystem final : public BusSystemBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeftBusSystem)
  LeftBusSystem();
};

class LeaderBusSystem final : public BusSystemBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeaderBusSystem)
  LeaderBusSystem();
};

}  // namespace xlerobot_hardware
