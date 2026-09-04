cmake_minimum_required(VERSION 3.8)
project(xlerobot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(xlerobot_drive_core
  src/drive_arbiter.cpp
)
target_include_directories(xlerobot_drive_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(xlerobot_drive_core geometry_msgs)
target_compile_features(xlerobot_drive_core PUBLIC cxx_std_17)

add_executable(drive_safety_node src/drive_safety_node.cpp)
target_include_directories(drive_safety_node PRIVATE include)
ament_target_dependencies(drive_safety_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(drive_safety_node xlerobot_drive_core)

install(TARGETS
  xlerobot_drive_core
  drive_safety_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_drive_arbiter test/test_drive_arbiter.cpp)
  target_link_libraries(test_drive_arbiter xlerobot_drive_core)
  ament_target_dependencies(test_drive_arbiter geometry_msgs)
  ament_add_pytest_test(
    test_drive_stop_runtime
    test/test_drive_stop_runtime.py
    TIMEOUT 30
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(xlerobot_drive_core)
ament_package()
