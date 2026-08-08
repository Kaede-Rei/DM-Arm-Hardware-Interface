cmake_minimum_required(VERSION 3.8)
project(serial_arm_ros2_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(serial_arm_core CONFIG REQUIRED)
if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
endif()
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(serial_arm_system SHARED
  src/serial_arm_system.cpp
)

target_include_directories(serial_arm_system
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_arm_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  realtime_tools
)

target_link_libraries(serial_arm_system
  serial_arm::core
  serial_arm::config
  serial_arm::robot
  serial_arm::dynamics
)

pluginlib_export_plugin_description_file(hardware_interface serial_arm_hardware_plugin.xml)

install(TARGETS
  serial_arm_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch urdf rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES serial_arm_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib)

if(BUILD_TESTING)
    ament_add_pytest_test(test_profile_utils test/test_profile_utils.py)
endif()

ament_package()
