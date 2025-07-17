cmake_minimum_required(VERSION 3.15)
project(rerun)

find_package(rerun_sdk REQUIRED)
find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)

add_executable(rerun demo.cpp)
add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)

target_link_libraries(rerun rerun_sdk core cppzmq capnp capnp_schemas kj)
