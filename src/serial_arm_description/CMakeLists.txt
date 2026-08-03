cmake_minimum_required(VERSION 3.8)
project(serial_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
