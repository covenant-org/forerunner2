cmake_minimum_required(VERSION 3.15)
project(core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall -Wextra -Werror")

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)

add_executable(testing testing.cpp)
target_link_libraries(testing PRIVATE cppzmq capnp capnp_schemas kj)

add_executable(subtesting subtest.cpp)
target_link_libraries(subtesting PRIVATE cppzmq capnp capnp_schemas kj)
