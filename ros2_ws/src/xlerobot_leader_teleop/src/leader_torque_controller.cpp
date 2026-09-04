#include "xlerobot_leader_teleop/leader_torque_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace xlerobot_leader_teleop
{

namespace
{

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

controller_interface::CallbackReturn LeaderTorqueController::on_init()
{
  const auto enable_lease_s = auto_declare<double>("enable_lease_s", 1.0);
  if (!std::isfinite(enable_lease_s) || enable_lease_s <= 0.0 || enable_lease_s > 2.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "enable_lease_s must be finite, positive, and no greater than 2 seconds");
    return controller_interface::CallbackReturn::ERROR;
  }
  enable_lease_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(enable_lease_s)).count();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
LeaderTorqueController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"leader_bus/torque_enable"}};
}

controller_interface::InterfaceConfiguration
LeaderTorqueController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn LeaderTorqueController::on_configure(
  const rclcpp_lifecycle::State &)
{
  lease_deadline_ns_.store(0);
  service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "/leader_bus/set_torque_enabled",
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response)
    {
      lease_deadline_ns_.store(
        request->data ? steady_now_ns() + enable_lease_ns_ : 0,
        std::memory_order_release);
      response->success = true;
      response->message = request->data ?
      "Leader torque lease enabled or refreshed" : "Leader torque disabled";
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderTorqueController::clear_lease_and_torque(
  bool command_interface_required)
{
  lease_deadline_ns_.store(0, std::memory_order_release);
  if (command_interfaces_.empty()) {
    if (command_interface_required) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Leader torque command interface is unavailable during lifecycle stop");
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  if (command_interfaces_.size() != 1 ||
    !command_interfaces_[0].set_value(0.0))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Could not command Leader torque off during lifecycle transition");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderTorqueController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return clear_lease_and_torque(true);
}

controller_interface::CallbackReturn LeaderTorqueController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return clear_lease_and_torque(true);
}

controller_interface::CallbackReturn LeaderTorqueController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  service_.reset();
  return clear_lease_and_torque(false);
}

controller_interface::CallbackReturn LeaderTorqueController::on_error(
  const rclcpp_lifecycle::State &)
{
  service_.reset();
  return clear_lease_and_torque(false);
}

controller_interface::CallbackReturn LeaderTorqueController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  service_.reset();
  return clear_lease_and_torque(false);
}

controller_interface::return_type LeaderTorqueController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  auto deadline_ns = lease_deadline_ns_.load(std::memory_order_acquire);
  const bool enabled = deadline_ns > 0 && steady_now_ns() < deadline_ns;
  if (!enabled && deadline_ns > 0) {
    lease_deadline_ns_.compare_exchange_strong(
      deadline_ns, 0, std::memory_order_acq_rel);
  }
  if (command_interfaces_.size() != 1 ||
    !command_interfaces_[0].set_value(enabled ? 1.0 : 0.0))
  {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

}  // namespace xlerobot_leader_teleop

PLUGINLIB_EXPORT_CLASS(
  xlerobot_leader_teleop::LeaderTorqueController,
  controller_interface::ControllerInterface)
