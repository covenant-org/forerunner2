CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(zed)

set(CMAKE_BUILD_TYPE "Release")

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(PCL 1.15 REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)


ADD_EXECUTABLE(${PROJECT_NAME} zed.cpp)
ADD_EXECUTABLE(ftp_bridge ftp_bridge.cpp)

# Suppress warnings from external ZED SDK headers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE 
        -Wno-deprecated-copy
        -Wno-unused-parameter
        -Wno-ignored-qualifiers
        -Wno-class-memaccess
        -Wno-strict-aliasing)
    target_compile_options(ftp_bridge PRIVATE 
        -Wno-deprecated-copy
        -Wno-unused-parameter
        -Wno-ignored-qualifiers)
endif()

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                        ${ZED_LIBS}
                        core
                        cppzmq 
                        capnp 
                        capnp_schemas 
                        kj
                        ${PCL_LIBRARIES})

TARGET_LINK_LIBRARIES(ftp_bridge
                        core
                        cppzmq 
                        capnp 
                        capnp_schemas 
                        kj
	                      MAVSDK::mavsdk
                        ${PCL_LIBRARIES})

if(INSTALL_SAMPLES)
    LIST(APPEND SAMPLE_LIST ${PROJECT_NAME})
    SET(SAMPLE_LIST "${SAMPLE_LIST}" PARENT_SCOPE)
endif()

