cmake_minimum_required(VERSION 3.8)
project(pfr_tf_broadcaster)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/base_pose_transformer.py
  scripts/room_to_world_static_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
