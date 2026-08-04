cmake_minimum_required(VERSION 3.8)
project(serial_arm_robot_profiles)

find_package(ament_cmake QUIET)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
  ament_package()
endif()
