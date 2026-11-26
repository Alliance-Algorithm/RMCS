cmake_minimum_required(VERSION 3.24)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
set(RMCS_LIBRMCS_SOURCE_DIR "" CACHE PATH "Local librmcs SDK source directory")
if(NOT RMCS_LIBRMCS_SOURCE_DIR)
    set(RMCS_LIBRMCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../librmcs-sdk")
endif()
cmake_path(ABSOLUTE_PATH RMCS_LIBRMCS_SOURCE_DIR NORMALIZE)
set(librmcs_cmake_source_dir "${RMCS_LIBRMCS_SOURCE_DIR}")
if(NOT EXISTS "${librmcs_cmake_source_dir}/CMakeLists.txt")
    string(APPEND librmcs_cmake_source_dir "/host")
endif()
if(NOT EXISTS "${librmcs_cmake_source_dir}/CMakeLists.txt")
    message(FATAL_ERROR
        "Local librmcs SDK not found at '${RMCS_LIBRMCS_SOURCE_DIR}'. "
        "Place the SDK in RMCS/librmcs-sdk or set RMCS_LIBRMCS_SOURCE_DIR to its path.")
endif()

option(RMCS_ENABLE_ULTRA_ETHERCAT "Enable the Ultra EtherCAT transport in librmcs" OFF)
if(RMCS_ENABLE_ULTRA_ETHERCAT)
    set(LIBRMCS_ENABLE_ETHERCAT ON)
endif()

message(STATUS "Using local librmcs SDK from ${RMCS_LIBRMCS_SOURCE_DIR}")
add_subdirectory("${librmcs_cmake_source_dir}" "${CMAKE_CURRENT_BINARY_DIR}/librmcs-sdk")

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()
