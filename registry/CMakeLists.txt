cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)

project(registry)

find_package(cppzmq REQUIRED)

add_executable(registry registry.cpp)
target_link_libraries(registry cppzmq)

add_executable(dummy dummy.cpp)
target_link_libraries(dummy cppzmq)