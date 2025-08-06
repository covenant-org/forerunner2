cmake_minimum_required(VERSION 3.15)
project(controller)


find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)


add_executable(${PROJECT_NAME} controller.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE
	core
	cppzmq 
	capnp 
	capnp_schemas 
	kj 
	Eigen3::Eigen)
