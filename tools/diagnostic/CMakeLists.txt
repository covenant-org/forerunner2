cmake_minimum_required(VERSION 3.15)

project(diagnostic_tool)

add_executable(diagnostic_tool
    diagnostic_app.cpp
    command_checker.cpp
    modem_checker.cpp
    utils.cpp
    main.cpp
)

target_link_libraries(diagnostic_tool PRIVATE core)
target_include_directories(diagnostic_tool PRIVATE ${CMAKE_SOURCE_DIR})
