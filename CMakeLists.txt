cmake_minimum_required(VERSION 3.20)
project(dm_arm_hardware_interface VERSION 0.1.0 LANGUAGES CXX)

# =============================================================================
# 全局 C++ / CMake 配置
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# =============================================================================
# 构建选项
# =============================================================================

# 终端主程序：用于达妙真机、动力学与完整纯 C++ API 的交互式联调
option(DM_ARM_BUILD_TERMINAL "Build terminal main application" ON)

# 达妙后端：编译串口 / USB-CAN / MIT 模式适配层
option(DM_ARM_BUILD_DAMIAO "Build Damiao backend" ON)

# 动力学：编译 Pinocchio 模型、运动学、重力补偿和完整动力学接口
option(DM_ARM_ENABLE_DYNAMICS "Enable Pinocchio dynamics" ON)

# Python：预留 pybind11 绑定开关；对应 target 尚未实现
option(DM_ARM_BUILD_PYTHON "Build Python binding" OFF)

# ROS 2：预留 ros2_control 适配开关；对应 target 尚未实现
option(DM_ARM_BUILD_ROS2 "Build ROS 2 integration" OFF)

# =============================================================================
# 公共辅助函数
# =============================================================================

function(dm_arm_enable_warnings target_name)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# =============================================================================
# 公共依赖与第三方头文件路径
# =============================================================================

# 配置模块始终需要 yaml-cpp
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(DM_ARM_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(DM_ARM_YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR
        "yaml-cpp was found, but neither yaml-cpp::yaml-cpp nor yaml-cpp target exists")
endif()

set(DM_ARM_TL_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/tl/include")
set(DM_ARM_DAMIAO_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_lib/damiao/include")

# 需要安装和导出的库 target 在各分区中追加到该列表
set(DM_ARM_INSTALL_TARGETS)

# 用于生成安装后的 package config
set(DM_ARM_CONFIG_HAS_DYNAMICS OFF)

# =============================================================================
# 必选模块：dm_arm_core
# =============================================================================

add_library(dm_arm_core
    src/core/joints_ctrller.cpp
    src/core/joint_actuator_mapper.cpp
    src/core/safety.cpp
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
dm_arm_enable_warnings(dm_arm_core)

list(APPEND DM_ARM_INSTALL_TARGETS dm_arm_core)

# =============================================================================
# 必选模块：dm_arm_config
# =============================================================================

add_library(dm_arm_config
    src/config/config.cpp
)
add_library(dm_arm::config ALIAS dm_arm_config)
set_target_properties(dm_arm_config PROPERTIES EXPORT_NAME config)

target_link_libraries(dm_arm_config
    PUBLIC
        dm_arm_core
        ${DM_ARM_YAML_CPP_TARGET}
)
target_include_directories(dm_arm_config
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(dm_arm_config PUBLIC cxx_std_17)
dm_arm_enable_warnings(dm_arm_config)

list(APPEND DM_ARM_INSTALL_TARGETS dm_arm_config)

# =============================================================================
# 必选模块：dm_arm_robot
# =============================================================================

add_library(dm_arm_robot
    src/robot.cpp
)
add_library(dm_arm::robot ALIAS dm_arm_robot)
set_target_properties(dm_arm_robot PROPERTIES EXPORT_NAME robot)

target_link_libraries(dm_arm_robot
    PUBLIC
        dm_arm_core
        dm_arm_config
)
target_include_directories(dm_arm_robot
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(dm_arm_robot PUBLIC cxx_std_17)
dm_arm_enable_warnings(dm_arm_robot)

list(APPEND DM_ARM_INSTALL_TARGETS dm_arm_robot)

# =============================================================================
# 可选模块：DM_ARM_BUILD_DAMIAO
# =============================================================================

if(DM_ARM_BUILD_DAMIAO)
    add_library(dm_arm_damiao
        src/hardware/damiao_motor_bus.cpp
    )
    add_library(dm_arm::damiao ALIAS dm_arm_damiao)
    set_target_properties(dm_arm_damiao PROPERTIES EXPORT_NAME damiao)

    target_link_libraries(dm_arm_damiao
        PUBLIC
            dm_arm_core
    )
    target_include_directories(dm_arm_damiao
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
            $<BUILD_INTERFACE:${DM_ARM_DAMIAO_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_compile_features(dm_arm_damiao PUBLIC cxx_std_17)
    dm_arm_enable_warnings(dm_arm_damiao)

    list(APPEND DM_ARM_INSTALL_TARGETS dm_arm_damiao)
endif()

# =============================================================================
# 可选模块：DM_ARM_ENABLE_DYNAMICS
# =============================================================================

if(DM_ARM_ENABLE_DYNAMICS)
    # Dynamics 公共接口使用 Eigen，具体模型与算法由 Pinocchio 提供
    find_package(Eigen3 REQUIRED NO_MODULE)
    find_package(pinocchio CONFIG REQUIRED)

    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/dynamics.cpp")
        message(FATAL_ERROR
            "DM_ARM_ENABLE_DYNAMICS=ON, but src/dynamics/dynamics.cpp does not exist. "
            "Complete mainline B1 before enabling the dynamics target.")
    endif()

    add_library(dm_arm_dynamics
        src/dynamics/dynamics.cpp
    )
    add_library(dm_arm::dynamics ALIAS dm_arm_dynamics)
    set_target_properties(dm_arm_dynamics PROPERTIES EXPORT_NAME dynamics)

    target_include_directories(dm_arm_dynamics
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${DM_ARM_TL_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

    target_link_libraries(dm_arm_dynamics
        PUBLIC
            dm_arm_core
            Eigen3::Eigen
    )

    # 优先使用现代 imported target；兼容部分旧版 Pinocchio 包
    if(TARGET pinocchio::pinocchio)
        target_link_libraries(dm_arm_dynamics PUBLIC pinocchio::pinocchio)
    elseif(pinocchio_LIBRARIES)
        target_include_directories(dm_arm_dynamics PUBLIC ${pinocchio_INCLUDE_DIRS})
        target_link_libraries(dm_arm_dynamics PUBLIC ${pinocchio_LIBRARIES})
    else()
        message(FATAL_ERROR
            "Pinocchio was found, but neither pinocchio::pinocchio nor "
            "pinocchio_LIBRARIES is available")
    endif()

    target_compile_features(dm_arm_dynamics PUBLIC cxx_std_17)
    dm_arm_enable_warnings(dm_arm_dynamics)

    list(APPEND DM_ARM_INSTALL_TARGETS dm_arm_dynamics)
    set(DM_ARM_CONFIG_HAS_DYNAMICS ON)
endif()

# =============================================================================
# 可选模块：DM_ARM_BUILD_PYTHON
# =============================================================================

if(DM_ARM_BUILD_PYTHON)
    # 后续在该分区中加入：
    #   find_package(pybind11 CONFIG REQUIRED)
    #   pybind11_add_module(...)
    #   target_link_libraries(... dm_arm_robot dm_arm_dynamics ...)
    message(FATAL_ERROR
        "DM_ARM_BUILD_PYTHON=ON, but the Python binding target has not been "
        "implemented yet. Keep this option OFF until mainline E is added.")
endif()

# =============================================================================
# 可选模块：DM_ARM_BUILD_ROS2
# =============================================================================

if(DM_ARM_BUILD_ROS2)
    # ROS 2 / ros2_control 适配建议位于独立 ament_cmake 包，避免根项目强依赖 ROS
    message(FATAL_ERROR
        "DM_ARM_BUILD_ROS2=ON, but the ROS 2 integration target has not been "
        "implemented yet. Keep this option OFF until mainline D is added.")
endif()

# =============================================================================
# 可选应用：DM_ARM_BUILD_TERMINAL
# =============================================================================

if(DM_ARM_BUILD_TERMINAL)
    if(NOT TARGET dm_arm_damiao)
        message(FATAL_ERROR "DM_ARM_BUILD_TERMINAL=ON requires DM_ARM_BUILD_DAMIAO=ON")
    endif()
    if(NOT TARGET dm_arm_dynamics)
        message(FATAL_ERROR "DM_ARM_BUILD_TERMINAL=ON requires DM_ARM_ENABLE_DYNAMICS=ON")
    endif()

    find_package(Threads REQUIRED)

    add_executable(dm_arm_terminal
        app/dm_arm_terminal.cpp
    )

    target_link_libraries(dm_arm_terminal
        PRIVATE
            dm_arm_robot
            dm_arm_config
            dm_arm_damiao
            dm_arm_dynamics
            Threads::Threads
    )

    target_compile_features(dm_arm_terminal PRIVATE cxx_std_17)
    dm_arm_enable_warnings(dm_arm_terminal)

    install(TARGETS dm_arm_terminal
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# =============================================================================
# 安装：库、公共头文件与配置文件
# =============================================================================

install(TARGETS ${DM_ARM_INSTALL_TARGETS}
    EXPORT dm_arm_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 通用头文件始终安装；Damiao 专用头文件在后端开启时单独安装
install(DIRECTORY include/dm_arm/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dm_arm
    PATTERN "damiao_motor_bus.hpp" EXCLUDE
)

install(DIRECTORY third_lib/tl/include/tl
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(TARGET dm_arm_damiao)
    install(FILES include/dm_arm/hardware/damiao_motor_bus.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dm_arm/hardware
    )
    install(DIRECTORY third_lib/damiao/include/dm_hw
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif()

install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
)

# =============================================================================
# 安装：CMake package 导出
# =============================================================================

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

# =============================================================================
# 配置摘要
# =============================================================================

message(STATUS "DM-Arm build configuration:")
message(STATUS "  DM_ARM_BUILD_TERMINAL = ${DM_ARM_BUILD_TERMINAL}")
message(STATUS "  DM_ARM_BUILD_DAMIAO   = ${DM_ARM_BUILD_DAMIAO}")
message(STATUS "  DM_ARM_ENABLE_DYNAMICS= ${DM_ARM_ENABLE_DYNAMICS}")
message(STATUS "  DM_ARM_BUILD_PYTHON   = ${DM_ARM_BUILD_PYTHON}")
message(STATUS "  DM_ARM_BUILD_ROS2     = ${DM_ARM_BUILD_ROS2}")
