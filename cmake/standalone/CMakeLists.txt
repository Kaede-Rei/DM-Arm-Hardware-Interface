cmake_minimum_required(VERSION 3.16)
project(dm_arm_core_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DM_ARM_SOURCE_DIR)
    get_filename_component(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
endif()

include(${DM_ARM_SOURCE_DIR}/cmake/dm_arm_core_targets.cmake)

dm_arm_add_tl_target(tl)
dm_arm_add_dm_hw_target(dm_hw)
add_library(dm_hw::dm_hw ALIAS dm_hw)
add_library(tl::tl ALIAS tl)
dm_arm_add_dm_control_core_target(dm_control_core)
add_library(dm_control_core::dm_control_core ALIAS dm_control_core)

option(DM_ARM_BUILD_TESTING "Build non-ROS unit tests." OFF)
if(DM_ARM_BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(test_joint_impedance_controller
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/test/test_joint_impedance_controller.cpp
    )
    target_link_libraries(test_joint_impedance_controller
        PRIVATE
            dm_control_core
            GTest::gtest_main
    )
    add_test(NAME test_joint_impedance_controller COMMAND test_joint_impedance_controller)
endif()
