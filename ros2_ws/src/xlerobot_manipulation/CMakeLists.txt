cmake_minimum_required(VERSION 3.8)
project(xlerobot_manipulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xlerobot_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(xlerobot_streaming_safety
  src/policy_replay.cpp
  src/streaming_validator.cpp
)
target_compile_features(xlerobot_streaming_safety PUBLIC cxx_std_17)
target_include_directories(xlerobot_streaming_safety PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(xlerobot_streaming_safety yaml-cpp)

add_library(xlerobot_grasp_safety src/grasp_plan_validator.cpp)
target_compile_features(xlerobot_grasp_safety PUBLIC cxx_std_17)
target_include_directories(xlerobot_grasp_safety PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(xlerobot_grasp_safety trajectory_msgs)

add_executable(policy_replay_check src/policy_replay_check.cpp)
target_compile_features(policy_replay_check PUBLIC cxx_std_17)
target_link_libraries(policy_replay_check xlerobot_streaming_safety)

add_executable(streaming_joint_executor
  src/controller_mode_manager.cpp
  src/streaming_joint_executor.cpp
)

add_executable(grasp_object_server src/grasp_object_server.cpp)
target_compile_features(grasp_object_server PUBLIC cxx_std_17)
target_link_libraries(grasp_object_server xlerobot_grasp_safety yaml-cpp)
ament_target_dependencies(grasp_object_server
  control_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  xlerobot_interfaces
)

add_executable(handover_object_server src/handover_object_server.cpp)
target_compile_features(handover_object_server PUBLIC cxx_std_17)
ament_target_dependencies(handover_object_server
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  xlerobot_interfaces
)

add_executable(startup_ready_pose src/startup_ready_pose.cpp)
target_compile_features(startup_ready_pose PUBLIC cxx_std_17)
ament_target_dependencies(startup_ready_pose
  control_msgs
  diagnostic_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)
add_executable(maintenance_preset_server src/maintenance_preset_server.cpp)
target_compile_features(maintenance_preset_server PUBLIC cxx_std_17)
ament_target_dependencies(maintenance_preset_server
  control_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs
  xlerobot_interfaces
)
add_executable(calibration_pose_server src/calibration_pose_server.cpp)
target_compile_features(calibration_pose_server PUBLIC cxx_std_17)
target_link_libraries(calibration_pose_server yaml-cpp)
ament_target_dependencies(calibration_pose_server
  control_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs
  xlerobot_interfaces
)
target_compile_features(streaming_joint_executor PUBLIC cxx_std_17)
target_link_libraries(streaming_joint_executor xlerobot_streaming_safety)
ament_target_dependencies(streaming_joint_executor
  controller_manager_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  xlerobot_interfaces
)

install(TARGETS
  xlerobot_streaming_safety
  xlerobot_grasp_safety
  policy_replay_check
  streaming_joint_executor
  grasp_object_server
  handover_object_server
  startup_ready_pose
  maintenance_preset_server
  calibration_pose_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY test/data/ DESTINATION share/${PROJECT_NAME}/replays)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(trajectory_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_streaming_validator test/test_streaming_validator.cpp)
  target_link_libraries(test_streaming_validator xlerobot_streaming_safety)
  ament_add_gtest(test_policy_replay test/test_policy_replay.cpp)
  target_link_libraries(test_policy_replay xlerobot_streaming_safety)
  target_compile_definitions(test_policy_replay PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
  )
  ament_add_gtest(test_grasp_plan_validator test/test_grasp_plan_validator.cpp)
  target_link_libraries(test_grasp_plan_validator xlerobot_grasp_safety)
  add_launch_test(test/test_streaming_executor_safety.py TIMEOUT 30)
  add_launch_test(test/test_grasp_object_safety.py TIMEOUT 30)
  add_launch_test(test/test_grasp_object_runtime.py TIMEOUT 45)
  add_launch_test(test/test_handover_object_safety.py TIMEOUT 30)
  add_launch_test(test/test_handover_object_runtime.py TIMEOUT 45)
  add_launch_test(test/test_maintenance_preset_safety.py TIMEOUT 30)
  add_launch_test(test/test_calibration_pose_safety.py TIMEOUT 30)
  add_launch_test(test/test_calibration_pose_runtime.py TIMEOUT 30)
endif()

ament_package()
