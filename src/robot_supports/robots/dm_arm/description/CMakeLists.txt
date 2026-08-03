cmake_minimum_required(VERSION 3.8)
project(dm_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
