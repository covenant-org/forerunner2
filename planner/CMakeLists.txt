cmake_minimum_required(VERSION 3.15)
project(planner)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(PCL 1.15 REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)


add_executable(${PROJECT_NAME} planner.cpp theta-star.cpp kdtree.cpp utils.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE
	core
	cppzmq
	capnp
	capnp_schemas
	kj
	MAVSDK::mavsdk
		Eigen3::Eigen
	${PCL_LIBRARIES})

add_executable(sound-${PROJECT_NAME} sound-planner.cpp)
target_link_libraries(sound-${PROJECT_NAME} PRIVATE
	core
	cppzmq 
	capnp 
	capnp_schemas 
	kj 
	MAVSDK::mavsdk
		Eigen3::Eigen
	${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}-input-goal input-goal.cpp)
target_link_libraries(${PROJECT_NAME}-input-goal PRIVATE
	core
	cppzmq
	capnp
	capnp_schemas
	kj)
