cmake_minimum_required(VERSION 3.20)
project(dm_arm_hardware_interface VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(DM_ARM_BUILD_ROS2 "Build ROS 2 integration" OFF)
option(DM_ARM_BUILD_PYTHON "Build Python binding" OFF)
option(DM_ARM_BUILD_DAMIAO "Build Damiao backend" OFF)
option(DM_ARM_ENABLE_DYNAMICS "Enable Pinocchio dynamics" OFF)
option(DM_ARM_BUILD_TOOLS "Build development tools" OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(dm_arm_core
    src/core/joints_ctrller.cpp
    src/core/joint_actuator_mapper.cpp
)

add_library(dm_arm::core ALIAS dm_arm_core)

target_include_directories(dm_arm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_lib/tl/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(dm_arm_core PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(dm_arm_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

configure_package_config_file(
    cmake/dm_arm_hardware_interfaceConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfig.cmake
    INSTALL_DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/dm_arm_hardware_interface
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    TARGETS dm_arm_core
    EXPORT dm_arm_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT dm_arm_targets
    FILE dm_arm_targets.cmake
    NAMESPACE dm_arm::
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/dm_arm_hardware_interface
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfigVersion.cmake
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/dm_arm_hardware_interface
)
