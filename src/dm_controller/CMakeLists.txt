cmake_minimum_required(VERSION 3.5)

# ========================= 项目配置(必填) ========================= #

# TODO: 修改项目名称
project(dm_controller)

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

# TODO: 在此列表中添加所有依赖
set(DEPENDENCIES
    # dependencies_name
    rclcpp
    sensor_msgs
    std_msgs
    pinocchio
)

# 必需依赖
find_package(ament_cmake REQUIRED)
# 遍历所有依赖
foreach(pkg IN LISTS DEPENDENCIES)
    find_package(${pkg} REQUIRED)
endforeach()

# ========================= 自定义接口配置(可选) ========================= #
# 自定义接口依赖
find_package(rosidl_default_generators REQUIRED)

# TODO: 定义接口文件：消息、服务、动作
set(MSG_FILES
    # msg/CustomMsg.msg

)
set(SRV_FILES
    # srv/CustomSrv.srv

)
set(ACTION_FILES
    # action/CustomAction.action

)

# 如果没有接口文件，跳过接口生成
if(MSG_FILES OR SRV_FILES OR ACTION_FILES)
    rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        ${SRV_FILES}
        ${ACTION_FILES}
        DEPENDENCIES ${DEPENDENCIES}
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

# ========================= 共享库配置(可选) ========================= #

# TODO: 定义库名称和源文件
set(LIB_NAME ${PROJECT_NAME}_lib)
set(LIB_SOURCES
    # src/library.cpp

)

# 如果没有库文件，则跳过库的创建
if(LIB_SOURCES)
    add_library(${LIB_NAME} SHARED ${LIB_SOURCES})
    target_include_directories(${LIB_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(${LIB_NAME} ${DEPENDENCIES})
endif()

# ========================= 节点配置(按需修改) ========================= #

# TODO: 在此列表中添加所有节点名称(不带.cpp后缀)
set(NODES
    # node_name
    # arm
)

# 统一编译节点
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

    # 如果有自定义接口，链接类型支持库
    if(MSG_FILES OR SRV_FILES OR ACTION_FILES)
        target_link_libraries(${node} "${cpp_typesupport_target}")
    endif()
endforeach()

# ========================= 安装配置 ========================= #

# 安装可执行文件
install(TARGETS ${NODES}
    DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include
)

# 自动检测并安装 launch 及 config
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        PATTERN "*.launch.py"
        PATTERN "*.launch.xml"
        PATTERN "*.launch"
        PATTERN "*.py"
    )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING 
        PATTERN "*.yaml"
        PATTERN "*.yml"
        PATTERN "*.json"
        PATTERN "*.xml"
    )
endif()

# 如果有共享库，安装库文件并导出库目标
if(LIB_SOURCES)
    install(TARGETS ${LIB_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()

# ========================= 导出配置 ========================= #

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

# 如果定义了自定义接口，导出接口类型支持
if(MSG_FILES OR SRV_FILES OR ACTION_FILES)
    ament_export_targets(${cpp_typesupport_target})
    ament_export_dependencies(rosidl_default_runtime)
endif()

# ========================= 测试配置(可选) ========================= #

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # TODO: 如需跳过某些检查，取消相应注释
  # set(ament_cmake_copyright_FOUND TRUE)    # 跳过版权检查
  # set(ament_cmake_cpplint_FOUND TRUE)      # 跳过cpplint检查
  # set(ament_cmake_uncrustify_FOUND TRUE)   # 跳过uncrustify检查
  
  ament_lint_auto_find_test_dependencies()
endif()

# ========================= 包配置 ========================= #

ament_package()
