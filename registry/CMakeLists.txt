cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)

project(registry)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)

add_executable(registry registry.cpp)
target_link_libraries(registry PRIVATE cppzmq capnp capnp_schemas kj core)
