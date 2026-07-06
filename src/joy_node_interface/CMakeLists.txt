cmake_minimum_required(VERSION 3.8)
project(joy_node_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(joy_node_interface src/joy_node_interface.cpp)
ament_target_dependencies(joy_node_interface
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_sdk)
target_include_directories(joy_node_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy_node_interface PUBLIC c_std_99 cxx_std_17)

install(TARGETS joy_node_interface
  DESTINATION lib/${PROJECT_NAME})


  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()