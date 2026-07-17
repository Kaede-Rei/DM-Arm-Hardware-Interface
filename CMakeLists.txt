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
option(DM_ARM_BUILD_TOOLS "Build development tools" ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(dm_arm_core
    src/core/joints_ctrller.cpp
)

add_library(dm_arm::core ALIAS dm_arm_core)

file(
    GLOB THIRD_LIB_INCLUDE_DIRS
    CONFIGURE_DEPENDS
    LIST_DIRECTORIES true
    "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/*/include"
)

target_include_directories(dm_arm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        ${THIRD_LIB_INCLUDE_DIRS}
)

target_compile_features(dm_arm_core PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(dm_arm_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
