cmake_minimum_required(VERSION 3.20)
project(serial_arm_core VERSION 0.1.0 LANGUAGES CXX)

# =============================================================================
# 全局 C++ / CMake 配置
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# When built by ROS 2/colcon, also register this CMake package in the ament
# index so package://serial_arm_core URIs resolve in tools such as MoveIt Setup
# Assistant. The library targets remain usable as a plain CMake package.
find_package(ament_cmake QUIET)

# =============================================================================
# 构建选项
# =============================================================================

# 终端主程序：用于达妙真机、动力学与完整纯 C++ API 的交互式联调
option(SERIAL_ARM_BUILD_TERMINAL "Build terminal main application" ON)

# 达妙后端：编译串口 / USB-CAN / MIT 模式适配层
option(SERIAL_ARM_BUILD_DAMIAO "Build Damiao backend" ON)

# 动力学：编译 Pinocchio 模型、运动学、重力补偿和完整动力学接口
option(SERIAL_ARM_ENABLE_DYNAMICS "Enable Pinocchio dynamics" ON)

# Python：编译 pybind11 离线 API 与 C++ RobotSession 真机接口
option(SERIAL_ARM_BUILD_PYTHON "Build Python binding" ON)

# =============================================================================
# 公共辅助函数
# =============================================================================

function(serial_arm_enable_warnings target_name)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# =============================================================================
# 公共依赖与第三方头文件路径
# =============================================================================

# 配置模块始终需要 yaml-cpp
find_package(yaml-cpp REQUIRED)
find_package(pinocchio CONFIG REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(SERIAL_ARM_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(SERIAL_ARM_YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR
        "yaml-cpp was found, but neither yaml-cpp::yaml-cpp nor yaml-cpp target exists")
endif()

if(TARGET pinocchio::pinocchio)
    set(SERIAL_ARM_PINOCCHIO_TARGET pinocchio::pinocchio)
elseif(pinocchio_LIBRARIES)
    set(SERIAL_ARM_PINOCCHIO_TARGET ${pinocchio_LIBRARIES})
else()
    message(FATAL_ERROR
        "Pinocchio was found, but neither pinocchio::pinocchio nor "
        "pinocchio_LIBRARIES is available")
endif()

set(SERIAL_ARM_TL_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/tl/include")
set(SERIAL_ARM_DAMIAO_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/damiao/include")

# 需要安装和导出的库 target 在各分区中追加到该列表
set(SERIAL_ARM_INSTALL_TARGETS)

# 用于生成安装后的 package config
set(SERIAL_ARM_CONFIG_HAS_DYNAMICS OFF)

# =============================================================================
# 必选模块：serial_arm_core
# =============================================================================

add_library(serial_arm_core
    src/core/joints_ctrller.cpp
    src/core/joint_actuator_mapper.cpp
    src/core/safety.cpp
)
add_library(serial_arm::core ALIAS serial_arm_core)
set_target_properties(serial_arm_core PROPERTIES EXPORT_NAME core)

target_include_directories(serial_arm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${SERIAL_ARM_TL_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(serial_arm_core PUBLIC cxx_std_17)
serial_arm_enable_warnings(serial_arm_core)

list(APPEND SERIAL_ARM_INSTALL_TARGETS serial_arm_core)

# =============================================================================
# 必选模块：serial_arm_config
# =============================================================================

add_library(serial_arm_config
    src/config/config.cpp
    src/config/limit_resolver.cpp
    src/model/model_loader.cpp
    src/hardware/hardware_capability.cpp
)
add_library(serial_arm::config ALIAS serial_arm_config)
set_target_properties(serial_arm_config PROPERTIES EXPORT_NAME config)

target_link_libraries(serial_arm_config
    PUBLIC
        serial_arm_core
        ${SERIAL_ARM_YAML_CPP_TARGET}
        ${SERIAL_ARM_PINOCCHIO_TARGET}
)
target_include_directories(serial_arm_config
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${SERIAL_ARM_TL_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${SERIAL_ARM_DAMIAO_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
if(pinocchio_INCLUDE_DIRS)
    target_include_directories(serial_arm_config PUBLIC ${pinocchio_INCLUDE_DIRS})
endif()
target_compile_features(serial_arm_config PUBLIC cxx_std_17)
serial_arm_enable_warnings(serial_arm_config)

list(APPEND SERIAL_ARM_INSTALL_TARGETS serial_arm_config)

# =============================================================================
# 必选模块：serial_arm_robot
# =============================================================================

add_library(serial_arm_robot
    src/robot.cpp
)
add_library(serial_arm::robot ALIAS serial_arm_robot)
set_target_properties(serial_arm_robot PROPERTIES EXPORT_NAME robot)

target_link_libraries(serial_arm_robot
    PUBLIC
        serial_arm_core
        serial_arm_config
)
target_include_directories(serial_arm_robot
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${SERIAL_ARM_TL_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(serial_arm_robot PUBLIC cxx_std_17)
serial_arm_enable_warnings(serial_arm_robot)

list(APPEND SERIAL_ARM_INSTALL_TARGETS serial_arm_robot)

# =============================================================================
# 可选模块：SERIAL_ARM_BUILD_DAMIAO
# =============================================================================

if(SERIAL_ARM_BUILD_DAMIAO)
    add_library(serial_arm_damiao
        src/hardware/damiao_motor_bus.cpp
    )
    add_library(serial_arm::damiao ALIAS serial_arm_damiao)
    set_target_properties(serial_arm_damiao PROPERTIES EXPORT_NAME damiao)

    target_link_libraries(serial_arm_damiao
        PUBLIC
            serial_arm_core
    )
    target_include_directories(serial_arm_damiao
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${SERIAL_ARM_TL_INCLUDE_DIR}>
            $<BUILD_INTERFACE:${SERIAL_ARM_DAMIAO_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_compile_features(serial_arm_damiao PUBLIC cxx_std_17)
    serial_arm_enable_warnings(serial_arm_damiao)

    list(APPEND SERIAL_ARM_INSTALL_TARGETS serial_arm_damiao)
endif()

# =============================================================================
# 可选模块：SERIAL_ARM_ENABLE_DYNAMICS
# =============================================================================

if(SERIAL_ARM_ENABLE_DYNAMICS)
    # Dynamics 公共接口使用 Eigen，具体模型与算法由 Pinocchio 提供
    find_package(Eigen3 REQUIRED NO_MODULE)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/dynamics.cpp")
        message(FATAL_ERROR
            "SERIAL_ARM_ENABLE_DYNAMICS=ON, but src/dynamics/dynamics.cpp does not exist. "
            "Complete mainline B1 before enabling the dynamics target.")
    endif()

    add_library(serial_arm_dynamics
        src/dynamics/dynamics.cpp
    )
    add_library(serial_arm::dynamics ALIAS serial_arm_dynamics)
    set_target_properties(serial_arm_dynamics PROPERTIES EXPORT_NAME dynamics)

    target_include_directories(serial_arm_dynamics
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${SERIAL_ARM_TL_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

    target_link_libraries(serial_arm_dynamics
        PUBLIC
            serial_arm_core
            Eigen3::Eigen
    )

    # 优先使用现代 imported target；兼容部分旧版 Pinocchio 包
    if(TARGET pinocchio::pinocchio)
        target_link_libraries(serial_arm_dynamics PUBLIC pinocchio::pinocchio)
    elseif(pinocchio_LIBRARIES)
        target_include_directories(serial_arm_dynamics PUBLIC ${pinocchio_INCLUDE_DIRS})
        target_link_libraries(serial_arm_dynamics PUBLIC ${pinocchio_LIBRARIES})
    else()
        message(FATAL_ERROR
            "Pinocchio was found, but neither pinocchio::pinocchio nor "
            "pinocchio_LIBRARIES is available")
    endif()

    target_compile_features(serial_arm_dynamics PUBLIC cxx_std_17)
    serial_arm_enable_warnings(serial_arm_dynamics)

    list(APPEND SERIAL_ARM_INSTALL_TARGETS serial_arm_dynamics)
    set(SERIAL_ARM_CONFIG_HAS_DYNAMICS ON)
endif()

# =============================================================================
# 可选模块：SERIAL_ARM_BUILD_PYTHON
# =============================================================================

if(SERIAL_ARM_BUILD_PYTHON)
    if(NOT TARGET serial_arm_dynamics)
        message(FATAL_ERROR "SERIAL_ARM_BUILD_PYTHON=ON requires SERIAL_ARM_ENABLE_DYNAMICS=ON")
    endif()

    find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)
    find_package(Threads REQUIRED)

    pybind11_add_module(_serial_arm MODULE
        python/bindings.cpp
        python/robot_session.cpp
    )

    target_include_directories(_serial_arm
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/python
    )

    target_link_libraries(_serial_arm
        PRIVATE
            serial_arm_core
            serial_arm_config
            serial_arm_robot
            serial_arm_dynamics
            Threads::Threads
    )

    if(TARGET serial_arm_damiao)
        target_link_libraries(_serial_arm PRIVATE serial_arm_damiao)
        target_compile_definitions(_serial_arm PRIVATE SERIAL_ARM_PYTHON_HAS_DAMIAO=1)
    endif()

    target_compile_features(_serial_arm PRIVATE cxx_std_17)
    serial_arm_enable_warnings(_serial_arm)

    install(TARGETS _serial_arm
        LIBRARY DESTINATION serial_arm
        COMPONENT python
    )
    install(FILES python/serial_arm/__init__.py
        DESTINATION serial_arm
        COMPONENT python
    )
endif()

# =============================================================================
# 可选应用：SERIAL_ARM_BUILD_TERMINAL
# =============================================================================

if(SERIAL_ARM_BUILD_TERMINAL)
    if(NOT TARGET serial_arm_damiao)
        message(FATAL_ERROR "SERIAL_ARM_BUILD_TERMINAL=ON requires SERIAL_ARM_BUILD_DAMIAO=ON")
    endif()
    if(NOT TARGET serial_arm_dynamics)
        message(FATAL_ERROR "SERIAL_ARM_BUILD_TERMINAL=ON requires SERIAL_ARM_ENABLE_DYNAMICS=ON")
    endif()

    find_package(Threads REQUIRED)

    add_executable(serial_arm_terminal
        app/serial_arm_terminal.cpp
    )

    target_link_libraries(serial_arm_terminal
        PRIVATE
            serial_arm_robot
            serial_arm_config
            serial_arm_damiao
            serial_arm_dynamics
            Threads::Threads
    )

    target_compile_features(serial_arm_terminal PRIVATE cxx_std_17)
    serial_arm_enable_warnings(serial_arm_terminal)

    install(TARGETS serial_arm_terminal
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# =============================================================================
# 安装：库、公共头文件与 CMake package
# =============================================================================

install(TARGETS ${SERIAL_ARM_INSTALL_TARGETS}
    EXPORT serial_arm_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 通用头文件始终安装；Damiao 专用头文件在后端开启时单独安装
install(DIRECTORY include/serial_arm/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/serial_arm
    PATTERN "damiao_motor_bus.hpp" EXCLUDE
)

install(DIRECTORY third_lib/tl/include/tl
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(TARGET serial_arm_damiao)
    install(FILES include/serial_arm/hardware/damiao_motor_bus.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/serial_arm/hardware
    )
    install(DIRECTORY third_lib/damiao/include/dm_hw
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif()

install(FILES package.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

# =============================================================================
# 安装：CMake package 导出
# =============================================================================

configure_package_config_file(
    cmake/serial_arm_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(EXPORT serial_arm_targets
    FILE serial_arm_targets.cmake
    NAMESPACE serial_arm::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
    ament_export_targets(serial_arm_targets HAS_LIBRARY_TARGET)
    ament_export_dependencies(yaml-cpp)
    ament_package()
endif()

# =============================================================================
# 配置摘要
# =============================================================================

message(STATUS "SerialArm-Core build configuration:")
message(STATUS "  SERIAL_ARM_BUILD_TERMINAL = ${SERIAL_ARM_BUILD_TERMINAL}")
message(STATUS "  SERIAL_ARM_BUILD_DAMIAO   = ${SERIAL_ARM_BUILD_DAMIAO}")
message(STATUS "  SERIAL_ARM_ENABLE_DYNAMICS= ${SERIAL_ARM_ENABLE_DYNAMICS}")
message(STATUS "  SERIAL_ARM_BUILD_PYTHON   = ${SERIAL_ARM_BUILD_PYTHON}")
