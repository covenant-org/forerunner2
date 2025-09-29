cmake_minimum_required(VERSION 3.15)
project(test)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(cppzmq REQUIRED)

add_executable(test main.cpp)

target_include_directories(test PRIVATE
    ${CMAKE_SOURCE_DIR}/messages/generated
)

target_link_libraries(test PRIVATE
    capnp_schemas
    capnp
    kj
)

add_executable(remote remote.cpp)

target_link_libraries(remote PRIVATE
    core
    cppzmq 
    capnp 
    capnp_schemas 
    kj
)

add_executable(chatter chatter.cpp)

target_link_libraries(chatter PRIVATE
    core
    cppzmq
    capnp_schemas
    capnp
    kj
)

add_executable(action action.cpp)

target_link_libraries(action PRIVATE
    core
    cppzmq
    capnp_schemas
    capnp
    kj
)
