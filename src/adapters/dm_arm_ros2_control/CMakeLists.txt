cmake_minimum_required(VERSION 3.8)
project(dm_arm_ros2_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(dm_arm_core CONFIG REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pinocchio CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(dm_arm_system SHARED
  src/dm_arm_system.cpp
)

target_include_directories(dm_arm_system
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_arm_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  realtime_tools
)

target_link_libraries(dm_arm_system
  dm_arm::core
  dm_arm::config
  dm_arm::robot
  dm_arm::damiao
  dm_arm::dynamics
)

pluginlib_export_plugin_description_file(hardware_interface dm_arm_hardware_plugin.xml)

install(TARGETS
  dm_arm_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES dm_arm_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib)
ament_package()
