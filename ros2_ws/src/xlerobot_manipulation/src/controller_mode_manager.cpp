#include "xlerobot_manipulation/controller_mode_manager.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <utility>

namespace xlerobot_manipulation
{

namespace
{

builtin_interfaces::msg::Duration duration_message(double seconds)
{
  builtin_interfaces::msg::Duration message;
  message.sec = static_cast<int32_t>(std::floor(seconds));
  message.nanosec = static_cast<uint32_t>((seconds - std::floor(seconds)) * 1.0e9);
  return message;
}

}  // namespace

ControllerModeManager::ControllerModeManager(rclcpp::Node & node, Config config)
: config_(std::move(config))
{
  list_client_ = node.create_client<ListControllers>(
    config_.controller_manager + "/list_controllers");
  switch_client_ = node.create_client<SwitchController>(
    config_.controller_manager + "/switch_controller");
}

bool ControllerModeManager::enter_policy_mode(std::string & error)
{
  return switch_mode(true, error);
}

bool ControllerModeManager::restore_normal_mode(std::string & error)
{
  return switch_mode(false, error);
}

bool ControllerModeManager::policy_mode_intact(std::string & error)
{
  return states_match(controller_states(error), true) && error.empty();
}

bool ControllerModeManager::switch_mode(bool policy_mode, std::string & error)
{
  error.clear();
  const auto timeout = std::chrono::duration<double>(config_.switch_timeout_s);
  if (!list_client_->wait_for_service(timeout) || !switch_client_->wait_for_service(timeout)) {
    error = "controller manager services unavailable";
    return false;
  }
  const auto states = controller_states(error);
  if (!error.empty()) {
    return false;
  }
  if (states_match(states, policy_mode)) {
    return true;
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = duration_message(config_.switch_timeout_s);
  request->activate_controllers = policy_mode ?
    std::vector<std::string>{config_.policy_controller} : config_.normal_controllers;
  request->deactivate_controllers = policy_mode ?
    config_.normal_controllers : std::vector<std::string>{config_.policy_controller};
  auto future = switch_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.switch_timeout_s + 0.5)) !=
    std::future_status::ready)
  {
    error = "controller switch service timed out";
    return false;
  }
  const auto response = future.get();
  if (!response->ok) {
    error = "strict controller switch failed: " + response->message;
    return false;
  }
  const auto verified = controller_states(error);
  if (!error.empty()) {
    return false;
  }
  if (!states_match(verified, policy_mode)) {
    error = "controller states did not reach the requested exclusive mode";
    return false;
  }
  return true;
}

bool ControllerModeManager::states_match(const StateMap & states, bool policy_mode) const
{
  const std::string policy_target = policy_mode ? "active" : "inactive";
  const auto policy = states.find(config_.policy_controller);
  if (policy == states.end() || policy->second != policy_target) {
    return false;
  }
  const std::string normal_target = policy_mode ? "inactive" : "active";
  for (const auto & controller : config_.normal_controllers) {
    const auto item = states.find(controller);
    if (item == states.end() || item->second != normal_target) {
      return false;
    }
  }
  return true;
}

ControllerModeManager::StateMap ControllerModeManager::controller_states(std::string & error)
{
  error.clear();
  auto future = list_client_->async_send_request(std::make_shared<ListControllers::Request>());
  if (future.wait_for(std::chrono::duration<double>(config_.switch_timeout_s)) !=
    std::future_status::ready)
  {
    error = "controller list service timed out";
    return {};
  }
  StateMap states;
  const auto response = future.get();
  for (const auto & controller : response->controller) {
    states[controller.name] = controller.state;
  }
  return states;
}

}  // namespace xlerobot_manipulation
