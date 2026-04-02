cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

# Install launch, urdf, and config directories
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
