cmake_minimum_required(VERSION 3.15)
project(gz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(PCL 1.15 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)

add_executable(${PROJECT_NAME} gz.cpp)

target_link_libraries(${PROJECT_NAME} 
                        core
                        cppzmq 
                        capnp 
                        capnp_schemas 
                        kj
                        Eigen3::Eigen
                        ${GZ-MSGS_LIBRARIES}
                        ${GZ-TRANSPORT_LIBRARIES}
                        ${PCL_LIBRARIES})
