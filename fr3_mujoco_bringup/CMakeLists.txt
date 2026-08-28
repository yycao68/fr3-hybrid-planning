cmake_minimum_required(VERSION 3.5)
project(fr3_mujoco_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  mujoco_models
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
