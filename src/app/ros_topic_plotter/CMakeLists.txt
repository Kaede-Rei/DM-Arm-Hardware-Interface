cmake_minimum_required(VERSION 3.8)
project(ros_topic_plotter)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/ros_topic_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
