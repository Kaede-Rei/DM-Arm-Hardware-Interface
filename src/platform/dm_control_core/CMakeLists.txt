cmake_minimum_required(VERSION 3.16)
project(dm_control_core)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tl REQUIRED)
find_package(dm_hw REQUIRED)

set(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.." CACHE PATH "DM arm repository root")
include(${DM_ARM_SOURCE_DIR}/cmake/dm_arm_core_targets.cmake)

dm_arm_add_dm_control_core_target(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(tl dm_hw Eigen3 pinocchio)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  ament_add_gtest(test_joint_impedance_controller
    test/test_joint_impedance_controller.cpp
  )
  if(TARGET test_joint_impedance_controller)
    target_link_libraries(test_joint_impedance_controller ${PROJECT_NAME})
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
