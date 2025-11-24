cmake_minimum_required(VERSION 3.20)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pub pub.cpp)

target_include_directories(pub PRIVATE
    ${ZMQ_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/core
    ${PROJECT_SOURCE_DIR}/messages/generated_inspect
)

target_compile_options(pub PRIVATE ${ZMQ_CFLAGS_OTHER})

target_link_libraries(pub PRIVATE
    core
    capnpc
    capnp
    kj
    capnp-json
    ${ZMQ_LIBRARIES}
    Threads::Threads
    yaml-cpp::yaml-cpp
)
