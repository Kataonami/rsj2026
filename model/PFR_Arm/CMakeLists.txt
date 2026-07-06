cmake_minimum_required(VERSION 3.8)
project(PFR_Arm2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES PFR_Arm2.urdf
  DESTINATION share/${PROJECT_NAME}
)

# Keep the legacy share/PFR_Arm2/urdf path pointed at the same canonical URDF.
install(
  FILES PFR_Arm2.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  FILES urdf/PFR_Arm2.csv
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
