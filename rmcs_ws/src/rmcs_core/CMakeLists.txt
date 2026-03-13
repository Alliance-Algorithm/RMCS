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
include(FetchContent)
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
  librmcs
  URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.0.0/librmcs-sdk-src-3.0.0.zip
  URL_HASH SHA256=b39f51c21baacdcbf3f0176119b8850137a108b88a67e12395d37d89e5ef53e8
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)

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

set(ODIN_DRIVER_DIR "${PROJECT_SOURCE_DIR}/../odin_ros_driver")
include_directories(SYSTEM "${ODIN_DRIVER_DIR}/include")

execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE _HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(_HOST_ARCH MATCHES "arm|aarch64")
  set(ODIN_HOST_API_LIB "${ODIN_DRIVER_DIR}/lib/liblydHostApi_arm.a")
else()
  set(ODIN_HOST_API_LIB "${ODIN_DRIVER_DIR}/lib/liblydHostApi_amd.a")
endif()

if(NOT EXISTS "${ODIN_HOST_API_LIB}")
  message(FATAL_ERROR "Missing Odin Host API library: ${ODIN_HOST_API_LIB}")
endif()

target_link_libraries(
  ${PROJECT_NAME}
  librmcs-sdk
  usb-1.0
  ${ODIN_HOST_API_LIB}
  pthread
  ${CMAKE_DL_LIBS}
)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()
