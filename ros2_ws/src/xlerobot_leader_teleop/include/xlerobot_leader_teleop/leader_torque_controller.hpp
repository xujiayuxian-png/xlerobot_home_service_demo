#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace xlerobot_leader_teleop
{

class LeaderTorqueController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  controller_interface::CallbackReturn clear_lease_and_torque(
    bool command_interface_required);

  std::atomic<std::int64_t> lease_deadline_ns_{0};
  std::int64_t enable_lease_ns_{0};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

}  // namespace xlerobot_leader_teleop
