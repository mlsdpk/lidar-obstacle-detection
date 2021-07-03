cmake_minimum_required(VERSION 3.5)
project(lexus_rx_450h_description)

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()