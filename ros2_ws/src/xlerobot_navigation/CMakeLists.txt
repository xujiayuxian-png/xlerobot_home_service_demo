cmake_minimum_required(VERSION 3.8)
project(xlerobot_navigation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xlerobot_interfaces REQUIRED)

add_library(xlerobot_navigation_core
  src/localization_core.cpp
  src/navigation_core.cpp
)
target_compile_features(xlerobot_navigation_core PUBLIC cxx_std_17)
target_include_directories(xlerobot_navigation_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(named_navigation_server src/named_navigation_server.cpp)
target_compile_features(named_navigation_server PUBLIC cxx_std_17)
target_link_libraries(named_navigation_server xlerobot_navigation_core)
ament_target_dependencies(named_navigation_server
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
  xlerobot_interfaces
)

add_executable(approach_target_server src/approach_target_server.cpp)
target_compile_features(approach_target_server PUBLIC cxx_std_17)
target_link_libraries(approach_target_server xlerobot_navigation_core)
ament_target_dependencies(approach_target_server
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
  xlerobot_interfaces
)

add_executable(auto_localizer_server src/auto_localizer_server.cpp)
target_compile_features(auto_localizer_server PUBLIC cxx_std_17)
target_link_libraries(auto_localizer_server xlerobot_navigation_core)
ament_target_dependencies(auto_localizer_server
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_srvs
  xlerobot_interfaces
)

add_executable(scan_map_consistency src/scan_map_consistency_node.cpp)
target_compile_features(scan_map_consistency PUBLIC cxx_std_17)
target_link_libraries(scan_map_consistency xlerobot_navigation_core)
ament_target_dependencies(scan_map_consistency
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  xlerobot_interfaces
)

install(TARGETS
  xlerobot_navigation_core
  approach_target_server
  auto_localizer_server
  named_navigation_server
  scan_map_consistency
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_navigation_core test/test_navigation_core.cpp)
  target_link_libraries(test_navigation_core xlerobot_navigation_core)
  ament_add_gtest(test_localization_core test/test_localization_core.cpp)
  target_link_libraries(test_localization_core xlerobot_navigation_core)
  add_launch_test(test/test_named_navigation_safety.py TIMEOUT 30)
  add_launch_test(test/test_named_navigation_runtime.py TIMEOUT 45)
  add_launch_test(test/test_approach_target_safety.py TIMEOUT 30)
  add_launch_test(test/test_approach_target_runtime.py TIMEOUT 45)
  add_launch_test(test/test_auto_localizer_safety.py TIMEOUT 30)
  add_launch_test(test/test_auto_localizer_runtime.py TIMEOUT 45)
  add_launch_test(test/test_amcl_localization_runtime.py TIMEOUT 40)
  add_launch_test(test/test_slam_mapping_runtime.py TIMEOUT 30)
  ament_add_pytest_test(test_nav2_contract test/test_nav2_contract.py)
endif()

ament_package()
