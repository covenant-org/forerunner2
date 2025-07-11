cmake_minimum_required(VERSION 3.15)
project(core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Wextra -Werror")

add_library(core STATIC
	src/vertex.cpp
	src/subscriber.cpp
	src/publisher.cpp
	src/action.cpp
)

target_link_libraries()
