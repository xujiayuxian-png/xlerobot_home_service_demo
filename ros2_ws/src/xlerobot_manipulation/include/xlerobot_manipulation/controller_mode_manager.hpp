#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace xlerobot_manipulation
{

class ControllerModeManager
{
public:
  struct Config
  {
    std::string controller_manager;
    std::string policy_controller;
    std::vector<std::string> normal_controllers;
    double switch_timeout_s{2.0};
  };

  ControllerModeManager(rclcpp::Node & node, Config config);

  bool enter_policy_mode(std::string & error);
  bool restore_normal_mode(std::string & error);
  bool policy_mode_intact(std::string & error);

private:
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using StateMap = std::unordered_map<std::string, std::string>;

  bool switch_mode(bool policy_mode, std::string & error);
  bool states_match(const StateMap & states, bool policy_mode) const;
  StateMap controller_states(std::string & error);

  Config config_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
};

}  // namespace xlerobot_manipulation
