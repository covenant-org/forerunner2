cmake_minimum_required(VERSION 3.15)
project(crazybridge)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)

add_executable(${PROJECT_NAME} crazybridge.cpp)

add_subdirectory(crazyflie_cpp)

target_link_libraries(${PROJECT_NAME} 
                        core
                        cppzmq 
                        capnp 
                        capnp_schemas 
                        kj
                        crazyflie_cpp
                        Eigen3::Eigen)

include_directories(crazyflie_cpp/include)
include_directories(crazyflie_cpp/crazyflie-link-cpp/include)
