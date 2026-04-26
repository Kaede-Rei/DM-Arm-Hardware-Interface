cmake_minimum_required(VERSION 3.8)

# ========================= 项目配置(必填) ========================= #
# TODO: 修改项目名称，必须与 package.xml 中的 <name> 一致
project(dm_ros_control)

# ========================= 基本配置 ========================= #
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

# ========================= 依赖配置(按需修改) ========================= #
# TODO: 在此列表中添加所有 ROS 依赖；如果依赖你自己的接口包，也写在这里
set(DEPENDENCIES
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    controller_manager
    pinocchio
    tl
    dm_hw
)

# 必需依赖
find_package(ament_cmake REQUIRED)

# 遍历所有依赖
foreach(pkg IN LISTS DEPENDENCIES)
    find_package(${pkg} REQUIRED)
endforeach()

# TODO: 如需第三方库，在这里手动添加
# 例如：find_package(Eigen3 REQUIRED)
# 例如：find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ========================= 共享库配置(可选) ========================= #
# TODO: 若无公共库，保持 LIB_SOURCES 为空即可
set(LIB_NAME ${PROJECT_NAME})
set(LIB_SOURCES
    # src/library.cpp
    src/dm_hardware_interface.cpp
    src/dm_motor_bus.cpp
    src/dynamics_observer.cpp
    src/pinocchio_dynamics_model.cpp
)

if(LIB_SOURCES)
    add_library(${LIB_NAME} SHARED ${LIB_SOURCES})
    target_include_directories(${LIB_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(${LIB_NAME} ${DEPENDENCIES})

    # TODO: 如需链接第三方库，在这里补充
    target_link_libraries(${LIB_NAME} Eigen3::Eigen yaml-cpp)
endif()

pluginlib_export_plugin_description_file(hardware_interface dm_ros_control_plugins.xml)

# ========================= 节点配置(按需修改) ========================= #
# TODO: 在此列表中添加所有 C++ 节点名称(不带 .cpp 后缀)
set(NODES
    # node_name
)

foreach(node IN LISTS NODES)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp)
        message(FATAL_ERROR "节点源码文件不存在：src/${node}.cpp，请检查文件路径/节点名！")
    endif()

    add_executable(${node} src/${node}.cpp)
    target_include_directories(${node}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(${node} ${DEPENDENCIES})

    # 如果有共享库，链接库
    if(LIB_SOURCES)
        target_link_libraries(${node} ${LIB_NAME})
    endif()

    # TODO: 如需链接第三方库，在这里补充
    # target_link_libraries(${node} Eigen3::Eigen)
endforeach()

# ========================= Python 脚本安装(可选) ========================= #
# TODO: 若无 Python 脚本，保持为空即可
set(PYTHON_SCRIPTS
    # scripts/tool.py
    # scripts/demo_node.py
)

if(PYTHON_SCRIPTS)
    install(PROGRAMS
        ${PYTHON_SCRIPTS}
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

# ========================= 安装配置 ========================= #
if(NODES)
    install(TARGETS ${NODES}
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

if(LIB_SOURCES)
    install(TARGETS ${LIB_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    install(DIRECTORY include/
        DESTINATION include
    )
endif()

# TODO: 自动安装常见资源目录；按需增删
foreach(dir IN ITEMS launch config params urdf meshes textures rviz srdf worlds maps)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
        install(
            DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endforeach()

# ========================= 导出配置 ========================= #
ament_export_dependencies(${DEPENDENCIES})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_export_include_directories(include)
endif()

# ========================= 测试配置(可选) ========================= #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # TODO: 如需跳过某些检查，取消相应注释
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_uncrustify_FOUND TRUE)

  # TODO: 如需 gtest / pytest，可在这里补充
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_example test/test_example.cpp)

  ament_lint_auto_find_test_dependencies()
endif()

# ========================= 包配置 ========================= #
ament_package()
