cmake_minimum_required(VERSION 3.5)
# FIXME Change the project name to match the <name> value in package.xml.
project(dynamixel_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pipebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Source files.
set(SRC_FILES
  src/battery_state_node.cpp
  src/driver.cpp
  src/diagnostics_node.cpp
  src/dynamixel_comms.cpp
  src/dynamixel_servo.cpp
  src/encoder_node.cpp
  src/multiple_wheel_drive_node.cpp
  src/motor_node.cpp
  src/servo_node.cpp
)

set(PROJECT_EXEC ${PROJECT_NAME}_exec)
add_executable(${PROJECT_EXEC} ${SRC_FILES})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_EXEC} PUBLIC -Wall -Wextra -pedantic)
endif()

ament_target_dependencies(${PROJECT_EXEC}
  rclcpp
  rclcpp_components
  diagnostic_msgs
  dynamixel_sdk
  geometry_msgs
  pipebot_msgs
  sensor_msgs
)

install(TARGETS ${PROJECT_EXEC}
  # PROJECT_NAME __MUST__ be used here for launch files to work.
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  set(PROJECT_TEST ${PROJECT_NAME}_test)
  ament_add_gtest(${PROJECT_TEST}
    test/mock_communications.cpp
    test/motor_node_test.cpp
    src/motor_node.cpp
  )
  target_include_directories(${PROJECT_TEST} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_TEST}
    rclcpp
    rclcpp_components
    pipebot_msgs
  )
  #target_link_libraries(${PROJECT_TEST} name_of_local_library)
endif()  # BUILD_TESTING

# Installs the package.xml file.
ament_package()
