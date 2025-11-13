cmake_minimum_required(VERSION 3.20)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(Threads REQUIRED)

add_executable(echo echo.cpp)

target_include_directories(echo PRIVATE
    ${ZMQ_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/core
    ${PROJECT_SOURCE_DIR}/messages/generated_inspect
)

target_compile_options(echo PRIVATE ${ZMQ_CFLAGS_OTHER})

target_link_libraries(echo PRIVATE
    core
    capnpc
    ${ZMQ_LIBRARIES}
    Threads::Threads
)
