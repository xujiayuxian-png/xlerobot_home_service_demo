cmake_minimum_required(VERSION 3.8)
project(xlerobot_feetech)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xlerobot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SCSERVO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/SCServo_Linux")
if(NOT EXISTS "${SCSERVO_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "SCServo_Linux is missing. Run: git submodule update --init --recursive")
endif()

add_subdirectory(${SCSERVO_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/SCServo)

add_library(${PROJECT_NAME} SHARED
  src/scservo_feetech_bus.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} PRIVATE SCServo)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_executable(servo_calibration_server src/servo_calibration_server.cpp)
add_library(servo_calibration_session STATIC src/servo_calibration_session.cpp)
target_include_directories(servo_calibration_session PUBLIC include)
target_link_libraries(servo_calibration_session PUBLIC yaml-cpp)
target_compile_features(servo_calibration_session PUBLIC cxx_std_17)
target_link_libraries(servo_calibration_server ${PROJECT_NAME} servo_calibration_session)
target_compile_features(servo_calibration_server PUBLIC cxx_std_17)
ament_target_dependencies(servo_calibration_server rclcpp xlerobot_interfaces)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS servo_calibration_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_servo_calibration_session test/test_servo_calibration_session.cpp)
  target_link_libraries(test_servo_calibration_session servo_calibration_session)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_servo_calibration_contract
    test/test_servo_calibration_contract.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
