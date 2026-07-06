cmake_minimum_required(VERSION 3.8)
project(pfr_state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "Disable deprecated hpp-fcl compatibility warnings" FORCE)
find_package(pinocchio 3.9.0 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(pfr_state_estimator src/pfr_state_estimator.cpp)
ament_target_dependencies(pfr_state_estimator
  ament_index_cpp
  geometry_msgs
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_sdk)
target_link_libraries(pfr_state_estimator
  pinocchio::pinocchio
)
target_compile_definitions(pfr_state_estimator PRIVATE
  COAL_DISABLE_HPP_FCL_WARNINGS
)
target_include_directories(pfr_state_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pfr_state_estimator PUBLIC c_std_99 cxx_std_17)

install(TARGETS pfr_state_estimator
  DESTINATION lib/${PROJECT_NAME})


  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
