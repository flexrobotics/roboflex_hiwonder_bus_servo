cmake_minimum_required(VERSION 3.18)

project(roboflex_hiwonder_bus_servo)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(FetchContent)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../roboflex/CMakeLists.txt")
    set(BUILD_ROBOFLEX_PYTHON_EXT OFF CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../roboflex" "${CMAKE_BINARY_DIR}/roboflex_core-build")
else()
    FetchContent_Declare(roboflex_core
        GIT_REPOSITORY https://github.com/flexrobotics/roboflex.git
        GIT_TAG        main
    )
    set(BUILD_ROBOFLEX_PYTHON_EXT OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(roboflex_core)
endif()

add_library(roboflex_hiwonder_bus_servo STATIC
    src/hiwonder_bus_servo_controller.cpp
    src/hiwonder_bus_servo.cpp
    include/roboflex_hiwonder_bus_servo/hiwonder_bus_servo_controller.h
    include/roboflex_hiwonder_bus_servo/hiwonder_bus_servo.h
)

set_property(TARGET roboflex_hiwonder_bus_servo PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(roboflex_hiwonder_bus_servo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(roboflex_hiwonder_bus_servo PUBLIC roboflex_core)

if(APPLE)
    find_library(IOKIT_FRAMEWORK IOKit)
    target_link_libraries(roboflex_hiwonder_bus_servo PRIVATE ${IOKIT_FRAMEWORK})
endif()

install(TARGETS roboflex_hiwonder_bus_servo
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/roboflex_hiwonder_bus_servo
    DESTINATION include
)

add_subdirectory(python)
