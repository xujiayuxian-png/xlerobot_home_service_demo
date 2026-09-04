cmake_minimum_required(VERSION 3.8)
project(xlerobot_leader_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(xlerobot_leader_controllers SHARED src/leader_torque_controller.cpp)
target_include_directories(xlerobot_leader_controllers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(xlerobot_leader_controllers
  controller_interface pluginlib rclcpp std_srvs)
target_compile_features(xlerobot_leader_controllers PUBLIC cxx_std_17)
pluginlib_export_plugin_description_file(
  controller_interface xlerobot_leader_controllers.xml)

add_executable(leader_follower_teleop src/leader_follower_teleop.cpp)
ament_target_dependencies(leader_follower_teleop
  rclcpp sensor_msgs std_srvs trajectory_msgs)
target_compile_features(leader_follower_teleop PUBLIC cxx_std_17)
install(TARGETS leader_follower_teleop xlerobot_leader_controllers
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES xlerobot_leader_controllers.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(
    leader_torque_controller_test
    test/test_leader_torque_controller.cpp)
  target_link_libraries(
    leader_torque_controller_test
    xlerobot_leader_controllers)
  ament_target_dependencies(
    leader_torque_controller_test
    controller_interface hardware_interface lifecycle_msgs rclcpp std_srvs)
  ament_add_pytest_test(
    leader_follower_watchdog_runtime
    test/test_leader_follower_watchdog_runtime.py
    TIMEOUT 30)
endif()

ament_package()
