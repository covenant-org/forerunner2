cmake_minimum_required(VERSION 3.20)

add_executable(registry_topics topics.cpp)
target_link_libraries(registry_topics PRIVATE core)
target_include_directories(registry_topics PRIVATE ${CMAKE_SOURCE_DIR})

add_subdirectory(echo)
add_subdirectory(diagnostic)
add_subdirectory(minio)
