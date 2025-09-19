cmake_minimum_required(VERSION 3.15)
project(gz)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(PCL 1.15 REQUIRED)
find_package(gz-msgs10)
find_package(gz-transport13)

if(NOT gz-msgs10_FOUND OR NOT gz-transport13_FOUND)
  message(WARNING "Gazebo dependencies not found (gz-msgs10: ${gz-msgs10_FOUND}, gz-transport13: ${gz-transport13_FOUND}), skipping gz target")
  return()
endif()

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
