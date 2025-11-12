cmake_minimum_required(VERSION 3.15)
project(forerunner2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall -Wextra")

set(EXCLUDED_SUBDIRS
    "vendor"
    "core"
    "messages"
)

## Experimental
find_package(capnp_schemas REQUIRED)
if (TARGET capnp_schemas::capnp_schemas AND NOT TARGET capnp_schemas)
    add_library(capnp_schemas INTERFACE IMPORTED)
    set_target_properties(capnp_schemas PROPERTIES
        INTERFACE_LINK_LIBRARIES capnp_schemas::capnp_schemas
    )
endif()
## End Experimental

if (EXISTS "/etc/nv_tegra_release")
  message(WARNING "Jetson Detected not compiling rerun")
  list(APPEND EXCLUDED_SUBDIRS "rerun")
endif()

find_package(ZED)
if(NOT DEFINED ZED_FOUND OR NOT ZED_FOUND)
  message(WARNING "ZED SDK not found, ignoring zed subdir")
  list(APPEND EXCLUDED_SUBDIRS "zed")
endif()

find_package(gz-msgs10)
if(NOT DEFINED gz-msgs10_FOUND OR NOT gz-msgs10_FOUND)
  message(WARNING "GZ msgs not found, ignoring gz subdir")
  list(APPEND EXCLUDED_SUBDIRS "gz")
endif()

add_subdirectory(core)
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


