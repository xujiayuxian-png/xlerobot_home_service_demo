#include <chrono>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "xlerobot_leader_teleop/leader_torque_controller.hpp"

using namespace std::chrono_literals;

namespace
{

std_srvs::srv::SetBool::Response::SharedPtr call_enabled(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
  bool enabled)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  auto future = client->async_send_request(request);
  EXPECT_EQ(
    executor.spin_until_future_complete(future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  return future.get();
}

TEST(
  LeaderTorqueController,
  HeartbeatExpiryAndLifecycleTransitionsCommandTorqueOff)
{
  xlerobot_leader_teleop::LeaderTorqueController controller;
  controller_interface::ControllerInterfaceParams parameters;
  parameters.controller_name = "leader_torque_controller";
  parameters.update_rate = 50;
  parameters.controller_manager_update_rate = 50;
  parameters.node_namespace = "/leader";
  parameters.node_options.parameter_overrides({
      rclcpp::Parameter("enable_lease_s", 0.12)});
  ASSERT_EQ(
    controller.init(parameters), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller.configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  double torque_enable = 0.0;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
    "leader_bus", "torque_enable", &torque_enable);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command, []() {});
  controller.assign_interfaces(std::move(commands), {});
  ASSERT_EQ(
    controller.get_node()->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_DOUBLE_EQ(torque_enable, 0.0);

  auto client_node = std::make_shared<rclcpp::Node>(
    "leader_torque_watchdog_test");
  auto client = client_node->create_client<std_srvs::srv::SetBool>(
    "/leader_bus/set_torque_enabled");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(1s));

  auto response = call_enabled(executor, client, true);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration(0, 20'000'000)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(torque_enable, 1.0);

  std::this_thread::sleep_for(70ms);
  response = call_enabled(executor, client, true);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  std::this_thread::sleep_for(70ms);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration(0, 20'000'000)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(torque_enable, 1.0);

  ASSERT_EQ(
    controller.get_node()->deactivate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(torque_enable, 0.0);
  response = call_enabled(executor, client, true);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  ASSERT_EQ(
    controller.get_node()->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_DOUBLE_EQ(torque_enable, 0.0);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration(0, 20'000'000)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(torque_enable, 0.0);

  response = call_enabled(executor, client, false);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration(0, 20'000'000)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(torque_enable, 0.0);

  response = call_enabled(executor, client, true);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration(0, 20'000'000)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(torque_enable, 1.0);
  std::this_thread::sleep_for(180ms);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration(0, 20'000'000)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(torque_enable, 0.0);

  ASSERT_EQ(
    controller.get_node()->deactivate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  executor.remove_node(client_node);
  executor.remove_node(controller.get_node()->get_node_base_interface());
  controller.release_interfaces();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
