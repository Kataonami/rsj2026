cmake_minimum_required(VERSION 3.8)
project(PFR_Arm2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES PFR_Arm2.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
