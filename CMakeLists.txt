cmake_minimum_required(VERSION 3.15)
project(forerunner2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall -Wextra")

add_subdirectory(vendor)
add_subdirectory(messages)
add_subdirectory(core)
add_subdirectory(registry)
