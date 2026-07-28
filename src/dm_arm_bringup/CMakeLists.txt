cmake_minimum_required(VERSION 3.8)
project(dm_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
