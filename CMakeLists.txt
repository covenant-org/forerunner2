cmake_minimum_required(VERSION 3.15)
project(forerunner2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall -Wextra")

# add_subdirectory(vendor)
# add_subdirectory(messages)
add_subdirectory(core)
# add_subdirectory(registry)
set(EXCLUDED_SUBDIRS
    "vendor"
    "core"
    "zed"
    "test"
    "controller"
)

file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */CMakeLists.txt)
foreach(child ${children})
    get_filename_component(subdir ${child} DIRECTORY)
    list(FIND EXCLUDED_SUBDIRS ${subdir} is_excluded)
    if(NOT subdir STREQUAL "." AND NOT subdir STREQUAL ".." AND is_excluded EQUAL -1)
        message(STATUS "Auto-adding subdirectory: ${subdir}")
        add_subdirectory(${subdir})
    endif()
endforeach()