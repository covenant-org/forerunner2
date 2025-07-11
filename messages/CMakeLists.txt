cmake_minimum_required(VERSION 3.15)
project(messages VERSION 1.0.0)

# check for capnp executable
find_program(CAPNP_EXECUTABLE capnp)

if(NOT CAPNP_EXECUTABLE)
    message(FATAL_ERROR "capnp executable not found. Please install capnp.")
else()
    message(STATUS "capnp: ${CAPNP_EXECUTABLE}")
endif()

set(CAPNP_SOURCES)
set(CAPNP_HEADERS)

# find all .capnp files
file(GLOB CAPNP_FILES "schemas/*.capnp")
foreach(file ${CAPNP_FILES}) 
    message(STATUS "Processing Cap'n Proto file: ${file}")

    # Extract the filename without the directory
    get_filename_component(file_name ${file} NAME)

    # compile file with capnp
    execute_process(COMMAND ${CAPNP_EXECUTABLE} compile -oc++ ${file} --src-prefix ${CMAKE_CURRENT_SOURCE_DIR}/schemas)

    # append the generated source and header
    list(APPEND CAPNP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file_name}.c++)
    list(APPEND CAPNP_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/${file_name}.h)
endforeach()

# create a library from the generated files
add_library(capnp_schemas STATIC ${CAPNP_SOURCES} ${CAPNP_HEADERS})

install(TARGETS capnp_schemas
    EXPORT capnp_schemasTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES ${CAPNP_HEADERS}
    DESTINATION include/capnp_schemas
)

# Install the export file
install(EXPORT capnp_schemasTargets
    FILE capnp_schemasTargets.cmake
    NAMESPACE capnp_schemas::
    DESTINATION lib/cmake/capnp_schemas
)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    capnp_schemasConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/capnp_schemasConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/capnp_schemasConfig.cmake
    INSTALL_DESTINATION lib/cmake/capnp_schemas
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/capnp_schemasConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/capnp_schemasConfigVersion.cmake
    DESTINATION lib/cmake/capnp_schemas
)