cmake_minimum_required(VERSION 3.15)
project(optitrack-client)

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)

add_executable(${PROJECT_NAME} utils.cpp optitrack-client.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
                        NatNet/include)

target_link_directories(${PROJECT_NAME} PUBLIC
                        NatNet/lib)

target_link_libraries(${PROJECT_NAME} 
                        core
                        cppzmq 
                        capnp 
                        capnp_schemas 
                        kj
                        NatNet)
