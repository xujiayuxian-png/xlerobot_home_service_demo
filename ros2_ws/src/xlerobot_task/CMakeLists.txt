cmake_minimum_required(VERSION 3.8)
project(xlerobot_task)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/fetch_deliver_task_node
  scripts/person_search_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_flow test/test_flow.py)
  ament_add_pytest_test(test_localization_quality test/test_localization_quality.py)
  ament_add_pytest_test(test_person_search test/test_person_search.py)
  ament_add_pytest_test(test_task_readiness test/test_task_readiness.py)
  ament_add_pytest_test(test_timeout_contract test/test_timeout_contract.py)
endif()

ament_package()
