cmake_minimum_required(VERSION 3.15)
project(test)

find_package(capnp_schemas REQUIRED)

add_executable(test main.cpp)

target_link_libraries(test PRIVATE
    capnp_schemas
    capnp
    kj
)