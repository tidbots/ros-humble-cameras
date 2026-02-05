cmake_minimum_required(VERSION 3.8)
project(camera_launch)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cv_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
