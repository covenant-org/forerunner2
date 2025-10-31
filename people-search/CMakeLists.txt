cmake_minimum_required(VERSION 3.15)
project(people_search)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
if(NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED QUIET)
if(NOT ZED_FOUND)
    message(WARNING "ZED SDK not found, skipping people_detection targets")
    return()
endif()

find_package(cppzmq REQUIRED)
find_package(capnp_schemas REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)

if(LINK_SHARED_ZED)
    set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

add_executable(${PROJECT_NAME}_detection
    detection.cpp camera-tracking.cpp)

target_compile_features(${PROJECT_NAME}_detection PRIVATE cxx_std_17)

target_include_directories(${PROJECT_NAME}_detection PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME}_detection PRIVATE
    core
    cppzmq
    capnp
    capnp_schemas
    kj
    CURL::libcurl
    ${OpenCV_LIBS}
    ${ZED_LIBS})

if(ZED_LIBRARY_DIR)
    target_link_directories(${PROJECT_NAME}_detection PRIVATE ${ZED_LIBRARY_DIR})
endif()
if(CUDA_LIBRARY_DIRS)
    target_link_directories(${PROJECT_NAME}_detection PRIVATE ${CUDA_LIBRARY_DIRS})
endif()
if(OpenCV_LIBRARY_DIRS)
    target_link_directories(${PROJECT_NAME}_detection PRIVATE ${OpenCV_LIBRARY_DIRS})
endif()

add_executable(${PROJECT_NAME}_llm
    llm-detection.cpp base64.cpp)

target_compile_features(${PROJECT_NAME}_llm PRIVATE cxx_std_17)

target_include_directories(${PROJECT_NAME}_llm PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME}_llm PRIVATE
    core
    cppzmq
    capnp
    capnp_schemas
    kj
    CURL::libcurl
    ${OpenCV_LIBS})

if(OpenCV_LIBRARY_DIRS)
	target_link_directories(${PROJECT_NAME}_llm PRIVATE ${OpenCV_LIBRARY_DIRS})
endif()

add_executable(${PROJECT_NAME}_controller people_search.cpp)

target_include_directories(${PROJECT_NAME}_controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_link_libraries(${PROJECT_NAME}_controller PRIVATE
	core
	cppzmq 
	capnp 
	capnp_schemas 
	kj 
	MAVSDK::mavsdk
)
