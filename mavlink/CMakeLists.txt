cmake_minimum_required(VERSION 3.15)
project(mavlink)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)

add_executable(${PROJECT_NAME} mavlink.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(${PROJECT_NAME} PRIVATE
	core
	cppzmq 
	capnp 
	capnp_schemas 
	kj 
	MAVSDK::mavsdk
)

add_executable(${PROJECT_NAME}-mission mavlink-mission.cpp)
target_include_directories(${PROJECT_NAME}-mission PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(${PROJECT_NAME}-mission PRIVATE
	core
	cppzmq
	capnp
	capnp_schemas
	kj)

add_executable(${PROJECT_NAME}-commander commander.cpp)
target_include_directories(${PROJECT_NAME}-commander PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(${PROJECT_NAME}-commander PRIVATE
	core
	cppzmq
	capnp
	capnp_schemas
	kj)
