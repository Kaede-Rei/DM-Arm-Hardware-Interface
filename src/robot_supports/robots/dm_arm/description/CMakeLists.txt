cmake_minimum_required(VERSION 3.8)
project(dm_arm_description)

find_package(ament_cmake QUIET)

install(DIRECTORY config model
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
  ament_package()
endif()
