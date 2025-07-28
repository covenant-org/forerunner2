cmake_minimum_required(VERSION 3.10)
project(launch_exec)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(BOOST_ROOT "/ruta/a/boost")
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(../core)
include_directories(../registry)
include_directories(../vendor/argparse/include)

add_executable(launch launch.cpp)

target_link_libraries(launch
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    pthread
)