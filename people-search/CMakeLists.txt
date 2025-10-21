cmake_minimum_required(VERSION 3.15)
project(people_search)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)

add_executable(${PROJECT_NAME} people_search.cpp)

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