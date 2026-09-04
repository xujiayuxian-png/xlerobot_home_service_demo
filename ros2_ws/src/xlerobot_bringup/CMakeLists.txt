cmake_minimum_required(VERSION 3.8)
project(xlerobot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
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
