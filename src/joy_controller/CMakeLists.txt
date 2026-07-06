cmake_minimum_required(VERSION 3.8)
project(joy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_controller
  src/joy_controller.cpp
)

ament_target_dependencies(joy_controller
  rclcpp
  std_msgs
  sensor_msgs
)

target_include_directories(joy_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../PARAM>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(joy_controller PUBLIC c_std_99 cxx_std_17)

install(TARGETS joy_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
