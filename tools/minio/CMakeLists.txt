cmake_minimum_required(VERSION 3.20)
project(minio_tool LANGUAGES CXX)

find_package(miniocpp REQUIRED)

add_executable(minio_tool minio.cpp)
target_link_libraries(minio_tool PRIVATE miniocpp::miniocpp)
target_compile_features(minio_tool PRIVATE cxx_std_17)
