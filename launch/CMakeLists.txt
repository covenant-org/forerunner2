cmake_minimum_required(VERSION 3.10)
project(launch)

find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME} launch.cpp nodes_yaml_parser.cpp)
add_executable(executable_test test/executable_test.cpp)
add_executable(executable_test2 test/executable_test2.cpp)

# Solo enlaza launch con las librerías necesarias
target_link_libraries(${PROJECT_NAME}
    core
    pthread
    yaml-cpp
)


