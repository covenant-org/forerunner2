cmake_minimum_required(VERSION 3.15)

project(diagnostic_tool)

add_library(diagnostic_tool_lib
    src/command_checker.cpp
    src/modem_checker.cpp
    src/utils.cpp
)

target_include_directories(diagnostic_tool_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(diagnostic_tool_lib PUBLIC core)

add_executable(diagnostic_tool src/main.cpp)

target_link_libraries(diagnostic_tool PRIVATE diagnostic_tool_lib)
