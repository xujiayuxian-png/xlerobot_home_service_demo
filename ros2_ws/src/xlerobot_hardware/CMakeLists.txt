cmake_minimum_required(VERSION 3.8)
project(xlerobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(xlerobot_feetech REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/calibrated_servo_codec.cpp
  src/bus_system.cpp
  src/wheel_servo_codec.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  xlerobot_feetech
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(hardware_interface xlerobot_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES xlerobot_hardware.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bus_system test/test_bus_system.cpp)
  target_link_libraries(test_bus_system ${PROJECT_NAME})
  ament_target_dependencies(test_bus_system hardware_interface rclcpp rclcpp_lifecycle)
  ament_add_gtest(test_calibrated_servo_codec test/test_calibrated_servo_codec.cpp)
  target_link_libraries(test_calibrated_servo_codec ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle xlerobot_feetech)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
