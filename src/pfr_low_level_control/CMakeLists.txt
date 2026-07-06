cmake_minimum_required(VERSION 3.8)
project(pfr_low_level_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "Disable deprecated hpp-fcl compatibility warnings" FORCE)
find_package(pinocchio 3.9.0 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pfr_low_level_control
  src/pfr_low_level_control.cpp
)

ament_target_dependencies(pfr_low_level_control
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(pfr_low_level_control
  pinocchio::pinocchio
)

target_compile_definitions(pfr_low_level_control PRIVATE
  COAL_DISABLE_HPP_FCL_WARNINGS
)

target_include_directories(pfr_low_level_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(pfr_low_level_control PUBLIC cxx_std_17)

install(TARGETS pfr_low_level_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
