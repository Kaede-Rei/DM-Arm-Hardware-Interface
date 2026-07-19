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

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(DM_ARM_TL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/tl/include")
set(DM_ARM_DAMIAO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/damiao/include")

add_library(dm_arm_core
    src/core/joints_ctrller.cpp
    src/core/joint_actuator_mapper.cpp
)
add_library(dm_arm::core ALIAS dm_arm_core)
set_target_properties(dm_arm_core PROPERTIES EXPORT_NAME core)

target_include_directories(dm_arm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(dm_arm_core PUBLIC cxx_std_17)

add_library(dm_arm_config src/config/config.cpp)
add_library(dm_arm::config ALIAS dm_arm_config)
set_target_properties(dm_arm_config PROPERTIES EXPORT_NAME config)
target_link_libraries(dm_arm_config PUBLIC dm_arm_core)
target_include_directories(dm_arm_config
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(DM_ARM_BUILD_DAMIAO)
    add_library(dm_arm_damiao src/hardware/damiao_motor_bus.cpp)
    add_library(dm_arm::damiao ALIAS dm_arm_damiao)
    set_target_properties(dm_arm_damiao PROPERTIES EXPORT_NAME damiao)
    target_link_libraries(dm_arm_damiao PUBLIC dm_arm_config)
    target_include_directories(dm_arm_damiao
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
            $<BUILD_INTERFACE:${DM_ARM_DAMIAO_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
endif()

foreach(target dm_arm_core dm_arm_config)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endforeach()
if(TARGET dm_arm_damiao AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(dm_arm_damiao PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(DM_ARM_INSTALL_TARGETS dm_arm_core dm_arm_config)
if(TARGET dm_arm_damiao)
    list(APPEND DM_ARM_INSTALL_TARGETS dm_arm_damiao)
endif()

install(TARGETS ${DM_ARM_INSTALL_TARGETS}
    EXPORT dm_arm_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/dm_arm/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dm_arm
    PATTERN "damiao_motor_bus.hpp" EXCLUDE)
install(DIRECTORY third_lib/tl/include/tl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(TARGET dm_arm_damiao)
    install(FILES include/dm_arm/hardware/damiao_motor_bus.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dm_arm/hardware)
    install(DIRECTORY third_lib/damiao/include/dm_hw DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(DIRECTORY config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config)

configure_package_config_file(
    cmake/dm_arm_hardware_interfaceConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(EXPORT dm_arm_targets
    FILE dm_arm_targets.cmake
    NAMESPACE dm_arm::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dm_arm_hardware_interfaceConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
