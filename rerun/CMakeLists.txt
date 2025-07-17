cmake_minimum_required(VERSION 3.15)
project(rerun)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)

add_executable(rerun demo.cpp)
add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)

target_link_libraries(rerun rerun_sdk core cppzmq capnp capnp_schemas kj)
