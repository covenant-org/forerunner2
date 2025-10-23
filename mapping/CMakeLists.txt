cmake_minimum_required(VERSION 3.15)
project(rerun)


find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(PCL 1.15 REQUIRED)

add_executable(rerun viewer.cpp)

target_link_libraries(core cppzmq capnp capnp_schemas kj ${PCL_LIBRARIES})
