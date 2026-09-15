cmake_minimum_required(VERSION 3.8)
project(xlerobot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xlerobot_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio imgproc)
add_executable(wrist_camera_native src/wrist_camera_native.cpp)
target_compile_features(wrist_camera_native PUBLIC cxx_std_17)
target_include_directories(wrist_camera_native PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(wrist_camera_native ${OpenCV_LIBS})
ament_target_dependencies(wrist_camera_native rclcpp sensor_msgs diagnostic_msgs)
install(TARGETS wrist_camera_native DESTINATION lib/${PROJECT_NAME})

add_executable(robot_state_gateway src/robot_state_gateway.cpp)
target_compile_features(robot_state_gateway PUBLIC cxx_std_17)
ament_target_dependencies(robot_state_gateway rclcpp sensor_msgs tf2_ros xlerobot_interfaces)
install(TARGETS robot_state_gateway DESTINATION lib/${PROJECT_NAME})

add_executable(camera_health src/camera_health_node.cpp)
target_compile_features(camera_health PUBLIC cxx_std_17)
ament_target_dependencies(camera_health rclcpp sensor_msgs diagnostic_msgs)
install(TARGETS camera_health DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/robot_description_publisher
  scripts/wrist_camera_node
  scripts/position_ready_spawner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_camera_health_runtime test/test_camera_health_runtime.py TIMEOUT 30)
  ament_add_pytest_test(test_robot_state_gateway test/test_robot_state_gateway.py TIMEOUT 30)
  ament_add_pytest_test(test_cpu_profile test/test_cpu_profile.py TIMEOUT 30)
  ament_add_pytest_test(test_initial_pose test/test_initial_pose.py TIMEOUT 30)
  ament_add_pytest_test(test_handeye_pose_geometry test/test_handeye_pose_geometry.py TIMEOUT 30)
  ament_add_pytest_test(test_lidar_geometry_source test/test_lidar_geometry_source.py TIMEOUT 30)
  ament_add_pytest_test(head_only_runtime test/test_head_only_runtime.py TIMEOUT 30)
  ament_add_pytest_test(collection_leader_lifecycle test/test_collection_leader_lifecycle.py TIMEOUT 90)
  ament_add_pytest_test(
    test_fetch_deliver_demo_contract
    test/test_fetch_deliver_demo_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_site_mapping_contract
    test/test_site_mapping_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_calibration_tools_contract
    test/test_calibration_tools_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_act_collection_contract
    test/test_act_collection_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
