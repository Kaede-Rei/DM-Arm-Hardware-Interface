include_guard(GLOBAL)

if(NOT DM_ARM_SOURCE_DIR)
    get_filename_component(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

option(DM_ARM_REJECT_ROS_PINOCCHIO "Reject Pinocchio found below /opt/ros." ON)
set(DM_ARM_PINOCCHIO_PREFIX "" CACHE PATH "Explicit non-ROS Pinocchio installation prefix.")
set(DM_ARM_OPENROBOTS_PREFIX "/opt/openrobots" CACHE PATH "OpenRobots robotpkg installation prefix.")
set(DM_ARM_PINOCCHIO_RUNTIME_PREFIX "${DM_ARM_OPENROBOTS_PREFIX}")

set(_dm_arm_pinocchio_prefix_candidates)

if(DM_ARM_PINOCCHIO_PREFIX)
    list(APPEND _dm_arm_pinocchio_prefix_candidates "${DM_ARM_PINOCCHIO_PREFIX}")
endif()

if(DEFINED ENV{CONDA_PREFIX} AND EXISTS "$ENV{CONDA_PREFIX}/lib/cmake/pinocchio")
    list(APPEND _dm_arm_pinocchio_prefix_candidates "$ENV{CONDA_PREFIX}")
endif()

if(EXISTS "${DM_ARM_OPENROBOTS_PREFIX}/lib/cmake/pinocchio")
    list(APPEND _dm_arm_pinocchio_prefix_candidates "${DM_ARM_OPENROBOTS_PREFIX}")
endif()

if(_dm_arm_pinocchio_prefix_candidates)
    list(PREPEND CMAKE_PREFIX_PATH ${_dm_arm_pinocchio_prefix_candidates})
    list(GET _dm_arm_pinocchio_prefix_candidates 0 DM_ARM_PINOCCHIO_RUNTIME_PREFIX)
endif()

function(dm_arm_require_pinocchio)
    find_package(Eigen3 REQUIRED)
    set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "Disable hpp-fcl compatibility warnings from coal." FORCE)
    find_package(pinocchio CONFIG REQUIRED)

    set(_pinocchio_origin "")
    if(pinocchio_DIR)
        set(_pinocchio_origin "${pinocchio_DIR}")
    elseif(TARGET pinocchio::pinocchio)
        get_target_property(_pinocchio_origin pinocchio::pinocchio INTERFACE_INCLUDE_DIRECTORIES)
    endif()

    if(DM_ARM_REJECT_ROS_PINOCCHIO AND _pinocchio_origin MATCHES "^/opt/ros/")
        message(FATAL_ERROR
            "Found ROS Pinocchio at ${_pinocchio_origin}. "
            "Use a non-ROS Pinocchio from conda/mamba, robotpkg, or "
            "-DDM_ARM_PINOCCHIO_PREFIX=/path/to/prefix.")
    endif()
endfunction()

function(dm_arm_add_tl_target target_name)
    if(TARGET ${target_name})
        return()
    endif()

    add_library(${target_name} INTERFACE)
    target_include_directories(${target_name}
        INTERFACE
            $<BUILD_INTERFACE:${DM_ARM_SOURCE_DIR}/src/infra/tl/include>
            $<INSTALL_INTERFACE:include>
    )
    target_compile_features(${target_name} INTERFACE cxx_std_17)
endfunction()

function(dm_arm_add_dm_hw_target target_name)
    if(TARGET ${target_name})
        return()
    endif()

    add_library(${target_name} SHARED
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_hw/src/clangd_provider.cpp
    )
    set_target_properties(${target_name}
        PROPERTIES
            BUILD_RPATH "${DM_ARM_PINOCCHIO_RUNTIME_PREFIX}/lib"
            INSTALL_RPATH "${DM_ARM_PINOCCHIO_RUNTIME_PREFIX}/lib"
    )
    target_include_directories(${target_name}
        PUBLIC
            $<BUILD_INTERFACE:${DM_ARM_SOURCE_DIR}/src/platform/dm_hw/include>
            $<INSTALL_INTERFACE:include>
    )
    target_compile_features(${target_name} PUBLIC cxx_std_17)
endfunction()

function(dm_arm_link_pinocchio target_name)
    dm_arm_require_pinocchio()
    target_compile_definitions(${target_name} PUBLIC COAL_DISABLE_HPP_FCL_WARNINGS)
    target_link_libraries(${target_name} PUBLIC Eigen3::Eigen)

    if(TARGET pinocchio::pinocchio)
        target_link_libraries(${target_name} PUBLIC pinocchio::pinocchio)
    else()
        target_include_directories(${target_name} PUBLIC ${pinocchio_INCLUDE_DIRS})
        target_link_libraries(${target_name} PUBLIC ${pinocchio_LIBRARIES})
    endif()
endfunction()

function(dm_arm_add_dm_control_core_target target_name)
    if(TARGET ${target_name})
        return()
    endif()

    if(NOT TARGET tl::tl AND NOT TARGET tl)
        dm_arm_add_tl_target(tl)
    endif()
    if(NOT TARGET dm_hw::dm_hw AND NOT TARGET dm_hw)
        dm_arm_add_dm_hw_target(dm_hw)
    endif()

    if(TARGET tl::tl)
        set(_dm_arm_tl_target tl::tl)
    else()
        set(_dm_arm_tl_target tl)
    endif()

    if(TARGET dm_hw::dm_hw)
        set(_dm_arm_hw_target dm_hw::dm_hw)
    else()
        set(_dm_arm_hw_target dm_hw)
    endif()

    add_library(${target_name} SHARED
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/dm_motor_bus.cpp
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/dynamics_observer.cpp
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/joint_impedance_controller.cpp
        ${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/src/pinocchio_dynamics_model.cpp
    )
    set_target_properties(${target_name}
        PROPERTIES
            BUILD_RPATH "${DM_ARM_PINOCCHIO_RUNTIME_PREFIX}/lib"
            INSTALL_RPATH "${DM_ARM_PINOCCHIO_RUNTIME_PREFIX}/lib"
    )
    target_include_directories(${target_name}
        PUBLIC
            $<BUILD_INTERFACE:${DM_ARM_SOURCE_DIR}/src/platform/dm_control_core/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${target_name} PUBLIC ${_dm_arm_hw_target} ${_dm_arm_tl_target})
    target_compile_features(${target_name} PUBLIC cxx_std_17)
    dm_arm_link_pinocchio(${target_name})
endfunction()
