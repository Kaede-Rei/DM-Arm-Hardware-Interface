# ========================= LeRobot / Isaac 独立构建入口 ========================= #
# 使用方式：
#   set(DM_ARM_SOURCE_DIR /path/to/DM-Arm-Hardware-Interface)
#   include(${DM_ARM_SOURCE_DIR}/cmake/dm_control_core.cmake)
#   target_link_libraries(your_target PRIVATE dm_control_core::dm_control_core)

if(NOT DM_ARM_SOURCE_DIR)
    get_filename_component(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

include(${DM_ARM_SOURCE_DIR}/cmake/dm_arm_core_targets.cmake)

if(NOT TARGET tl::tl)
    dm_arm_add_tl_target(tl)
    add_library(tl::tl ALIAS tl)
endif()

if(NOT TARGET dm_hw::dm_hw)
    dm_arm_add_dm_hw_target(dm_hw)
    add_library(dm_hw::dm_hw ALIAS dm_hw)
endif()

if(NOT TARGET dm_control_core::dm_control_core)
    dm_arm_add_dm_control_core_target(dm_control_core)
    add_library(dm_control_core::dm_control_core ALIAS dm_control_core)
endif()
