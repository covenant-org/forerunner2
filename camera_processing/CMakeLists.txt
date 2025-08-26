cmake_minimum_required(VERSION 3.5)

project(camera_viewer)

find_package(gz-transport13 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_viewer camera.cpp)

target_include_directories(camera_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_viewer PRIVATE gz-transport13::gz-transport13 ${OpenCV_LIBS})