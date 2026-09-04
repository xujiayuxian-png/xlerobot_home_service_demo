cmake_minimum_required(VERSION 3.8)
project(xlerobot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_base_xacro test/test_base_xacro.py)
  ament_add_pytest_test(test_no_active_imu test/test_no_active_imu.py)
endif()

ament_package()
