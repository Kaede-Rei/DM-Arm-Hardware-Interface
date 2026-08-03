cmake_minimum_required(VERSION 3.20)
project(serial_arm_hardware_damiao VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)

find_package(ament_cmake QUIET)
find_package(serial_arm_core CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(SERIAL_ARM_DAMIAO_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(SERIAL_ARM_DAMIAO_YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp was found, but no usable target exists")
endif()

add_library(serial_arm_hardware_damiao SHARED
    src/damiao_motor_bus.cpp
)

target_include_directories(serial_arm_hardware_damiao
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_lib/damiao/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(serial_arm_hardware_damiao
    PUBLIC
        serial_arm::core
        ${SERIAL_ARM_DAMIAO_YAML_CPP_TARGET}
)

install(TARGETS serial_arm_hardware_damiao
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY third_lib/damiao/include/dm_hw
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES package.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
    ament_export_include_directories(include)
    ament_export_libraries(serial_arm_hardware_damiao)
    ament_export_dependencies(serial_arm_core yaml-cpp)
    ament_package()
endif()
