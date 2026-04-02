cmake_minimum_required(VERSION 3.8)
project(nav2_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY config launch maps scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/fake_hazards.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_hazards
)

ament_package()
