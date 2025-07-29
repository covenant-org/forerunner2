cmake_minimum_required(VERSION 3.10)
project(launch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(BOOST_ROOT "/ruta/a/boost")
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)


add_subdirectory(${CMAKE_SOURCE_DIR}/../core ${CMAKE_BINARY_DIR}/core)


add_executable(launch launch.cpp nodes_yaml_parser.cpp)
add_executable(executable_test test/executable_test.cpp)
add_executable(executable_test2 test/executable_test2.cpp)

# Solo enlaza launch con las librerías necesarias
target_link_libraries(launch
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    core
    pthread
    yaml-cpp
)

# target_link_libraries(nodes_yaml_parser
#     yaml-cpp
# )



