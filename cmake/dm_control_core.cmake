# ========================= LeRobot / Isaac 独立构建入口 ========================= #
# 使用方式：
#   set(DM_ARM_SOURCE_DIR /path/to/DM-Arm-Hardware-Interface)
#   include(${DM_ARM_SOURCE_DIR}/cmake/dm_control_core.cmake)
#   target_link_libraries(your_target PRIVATE dm_control_core::dm_control_core)

if(NOT DM_ARM_SOURCE_DIR)
    get_filename_component(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

if(NOT TARGET dm_hw::dm_hw)
    add_library(dm_hw_standalone INTERFACE)
    add_library(dm_hw::dm_hw ALIAS dm_hw_standalone)

    target_include_directories(dm_hw_standalone
        INTERFACE
            ${DM_ARM_SOURCE_DIR}/src/platform/dm_hw/include
    )

    target_compile_features(dm_hw_standalone INTERFACE cxx_std_17)
endif()

if(NOT TARGET tl::tl)
    add_library(tl_standalone INTERFACE)
    add_library(tl::tl ALIAS tl_standalone)

    target_include_directories(tl_standalone
        INTERFACE
            ${DM_ARM_SOURCE_DIR}/src/infra/tl/include
    )

    target_compile_features(tl_standalone INTERFACE cxx_std_17)
endif()

if(NOT TARGET dm_control_core::dm_control_core)
    add_library(dm_control_core_standalone SHARED
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/dm_motor_bus.cpp
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/dynamics_observer.cpp
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/joint_impedance_controller.cpp
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/pinocchio_dynamics_model.cpp
    )
    add_library(dm_control_core::dm_control_core ALIAS dm_control_core_standalone)

    target_include_directories(dm_control_core_standalone
        PUBLIC
            ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/include
    )

    target_link_libraries(dm_control_core_standalone
        PUBLIC
            dm_hw::dm_hw
            tl::tl
            Eigen3::Eigen
    )

    if(TARGET pinocchio::pinocchio)
        target_link_libraries(dm_control_core_standalone PUBLIC pinocchio::pinocchio)
    else()
        target_include_directories(dm_control_core_standalone PUBLIC ${pinocchio_INCLUDE_DIRS})
        target_link_libraries(dm_control_core_standalone PUBLIC ${pinocchio_LIBRARIES})
    endif()

    target_compile_features(dm_control_core_standalone PUBLIC cxx_std_17)
endif()
