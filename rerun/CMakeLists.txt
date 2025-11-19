cmake_minimum_required(VERSION 3.15)
project(rerun)


include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/download/0.25.0/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(PCL 1.15 REQUIRED)

add_executable(rerun viewer.cpp)
add_executable(rerun_comprobation service/comprobation.cpp)
add_executable(rerun_render_registry_example render_registry_example.cpp)

target_link_libraries(rerun rerun_sdk core cppzmq capnpc capnp capnp_schemas kj ${PCL_LIBRARIES})
target_link_libraries(rerun_comprobation core cppzmq capnpc capnp capnp_schemas kj)
target_link_libraries(rerun_render_registry_example rerun_sdk core cppzmq capnpc capnp capnp_schemas kj)