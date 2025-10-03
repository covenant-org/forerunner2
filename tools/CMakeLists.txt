cmake_minimum_required(VERSION 3.20)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Add echo executable
add_executable(echo echo.cpp)

target_link_libraries(echo 
    core
    ${ZMQ_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(echo PRIVATE 
    ${ZMQ_INCLUDE_DIRS}
    ../core
    ../messages/generated_inspect
)

target_compile_options(echo PRIVATE ${ZMQ_CFLAGS_OTHER})
