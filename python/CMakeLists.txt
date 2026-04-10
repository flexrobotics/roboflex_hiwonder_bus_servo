cmake_minimum_required(VERSION 3.18)

project(roboflex_hiwonder_bus_servo_ext)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(PYBIND11_CPP_STANDARD -std=c++20)

find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG QUIET)

include(FetchContent)

if(NOT pybind11_FOUND)
    FetchContent_Declare(pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
    )
    FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(roboflex_hiwonder_bus_servo_ext
    pybindings.cpp
)

target_link_libraries(roboflex_hiwonder_bus_servo_ext PUBLIC
    roboflex_hiwonder_bus_servo
)

set_target_properties(roboflex_hiwonder_bus_servo_ext PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
