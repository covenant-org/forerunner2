cmake_minimum_required(VERSION 3.15)
project(planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)

add_executable(planner planner.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vendor/eigen)

target_include_directories(planner PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(planner PRIVATE
	core
	cppzmq 
	capnp 
	capnp_schemas 
	kj 
	MAVSDK::mavsdk
)
