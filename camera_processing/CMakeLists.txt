CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(people_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(OpenCV REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(PCL 1.15 REQUIRED)
find_package(cppzmq REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)

ADD_EXECUTABLE(${PROJECT_NAME} 
                camera.cpp 
                ../people-detection/src/camera_tracking/camera_tracking.cpp
                )

ADD_EXECUTABLE(ftp_bridge_pd ftp_bridge_pd.cpp)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                        ${ZED_LIBS} 
                        ${OpenCV_LIBS} 
                        curl 
                        core 
                        capnp capnp_schemas)

TARGET_LINK_LIBRARIES(ftp_bridge_pd
                        core
                        cppzmq 
                        capnp 
                        capnp_schemas 
                        kj
	                    MAVSDK::mavsdk
                        ${PCL_LIBRARIES})