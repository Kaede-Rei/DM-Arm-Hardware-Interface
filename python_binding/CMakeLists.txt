cmake_minimum_required(VERSION 3.18)
project(dm_impedance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PYBIND11_FINDPYTHON ON)

find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

if(NOT DM_ARM_SOURCE_DIR)
    get_filename_component(DM_ARM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

include(${DM_ARM_SOURCE_DIR}/cmake/dm_control_core.cmake)

pybind11_add_module(dm_impedance
    src/bindings.cpp
)

target_link_libraries(dm_impedance
    PRIVATE
        dm_control_core::dm_control_core
)

target_compile_features(dm_impedance
    PRIVATE
        cxx_std_17
)

target_compile_definitions(dm_impedance
    PRIVATE
        VERSION_INFO="0.1.0"
)

if(DEFINED ENV{CONDA_PREFIX})
    set(_DM_ARM_EXTRA_RPATH "$ENV{CONDA_PREFIX}/lib")
else()
    set(_DM_ARM_EXTRA_RPATH "")
endif()

set_target_properties(dm_impedance
    PROPERTIES
        BUILD_RPATH "$ORIGIN;${_DM_ARM_EXTRA_RPATH};${DM_ARM_PINOCCHIO_RUNTIME_PREFIX}/lib"
        INSTALL_RPATH "$ORIGIN;${_DM_ARM_EXTRA_RPATH};${DM_ARM_PINOCCHIO_RUNTIME_PREFIX}/lib"
)

install(TARGETS dm_impedance
    LIBRARY DESTINATION "${Python3_SITEARCH}"
)