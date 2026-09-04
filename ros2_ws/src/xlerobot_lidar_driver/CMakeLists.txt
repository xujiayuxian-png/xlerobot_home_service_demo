cmake_minimum_required(VERSION 3.8)
project(xlerobot_lidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(xlerobot_lidar_driver_core
  src/lidar_packet_parser.cpp
  src/lidar_protocol.cpp
  src/scan_accumulator.cpp
  src/scan_builder.cpp
  src/scan_filter.cpp
  src/serial_port.cpp
)
target_include_directories(xlerobot_lidar_driver_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(xlerobot_lidar_driver_core sensor_msgs)

add_executable(lidar_node src/lidar_node.cpp)
target_include_directories(lidar_node PRIVATE include)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)
target_link_libraries(lidar_node xlerobot_lidar_driver_core)

install(TARGETS xlerobot_lidar_driver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lidar_core test/test_lidar_core.cpp)
  target_link_libraries(test_lidar_core xlerobot_lidar_driver_core)
  ament_add_pytest_test(test_lidar_hardware_gate test/test_lidar_hardware_gate.py)
endif()

ament_package()
