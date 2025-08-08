cmake_minimum_required(VERSION 3.15)
project(core)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall -Wextra -Werror")

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(argparse REQUIRED)

add_library(core INTERFACE)
target_include_directories(core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(core INTERFACE cppzmq capnp capnp_schemas kj)
target_compile_features(core INTERFACE)

# only for intellisense purposes
file(GLOB_RECURSE CORE_HEADERS "*.h" "*.hpp" "*.hxx")
if(CORE_HEADERS)
    add_library(core_intellisense EXCLUDE_FROM_ALL ${CORE_HEADERS})
    target_include_directories(core_intellisense PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(core_intellisense PRIVATE cppzmq capnp capnp_schemas kj)
    target_compile_features(core_intellisense PRIVATE cxx_std_17)
    set_target_properties(core_intellisense PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_STANDARD 17
    )
endif()
