cmake_minimum_required(VERSION 3.15)
project(mavlink)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core})

add_executable(mavlink-vertex mavlink-vertex.cpp)

target_include_directories(mavlink-vertex PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(mavlink-vertex PRIVATE
	core
	cppzmq 
	capnp 
	capnp_schemas 
	kj 
	MAVSDK::mavsdk
)
