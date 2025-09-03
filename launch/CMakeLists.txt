cmake_minimum_required(VERSION 3.10)
project(launch)

find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME} launch.cpp nodes_yaml_parser.cpp)

# Solo enlaza launch con las librerías necesarias
target_link_libraries(${PROJECT_NAME}
    core
    pthread
    yaml-cpp::yaml-cpp
)


