cmake_minimum_required(VERSION 3.15)
project(controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(capnp_schemas REQUIRED)
find_package(cppzmq REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)
add_executable(controller controller.cpp)
target_include_directories(controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(controller PRIVATE
	cppzmq 
  	core
	capnp 
	capnp_schemas 
	kj 
)
