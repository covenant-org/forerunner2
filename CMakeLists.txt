cmake_minimum_required(VERSION 3.15)
project(forerunner2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall -Wextra")

add_subdirectory(core)

set(EXCLUDED_SUBDIRS
    "vendor"
    "core"
    "zed"
    "test"
)

if(DEFINED ENV{SUBDIR})
    set(SUBDIR $ENV{SUBDIR})
endif()

if(SUBDIR)
    foreach(sub ${SUBDIR})
        message(STATUS "Building only subdirectory: ${sub}")
        add_subdirectory(${sub})
    endforeach()
else()
    file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */CMakeLists.txt)
    foreach(child ${children})
        get_filename_component(subdir ${child} DIRECTORY)
        list(FIND EXCLUDED_SUBDIRS ${subdir} is_excluded)
        if(NOT subdir STREQUAL "." AND NOT subdir STREQUAL ".." AND is_excluded EQUAL -1)
            message(STATUS "Auto-adding subdirectory: ${subdir}")
            add_subdirectory(${subdir})
        endif()
    endforeach()
endif()
