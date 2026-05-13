# ========================= LeRobot / Isaac 独立构建入口 ========================= #
# 使用方式：
#   set(DM_ARM_SOURCE_DIR /path/to/DM-Arm-Hardware-Interface)
#   include(${DM_ARM_SOURCE_DIR}/cmake/impedance_controller.cmake)
#   target_link_libraries(your_target PRIVATE impedance_controller::impedance_controller)

if(NOT DM_ARM_SOURCE_DIR)
    get_filename_component(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

include(${DM_ARM_SOURCE_DIR}/cmake/dm_arm_core_targets.cmake)

if(NOT TARGET tl::tl)
    dm_arm_add_tl_target(tl)
    add_library(tl::tl ALIAS tl)
endif()

if(NOT TARGET impedance_controller::impedance_controller)
    dm_arm_add_impedance_controller_target(impedance_controller)
    add_library(impedance_controller::impedance_controller ALIAS impedance_controller)
endif()
