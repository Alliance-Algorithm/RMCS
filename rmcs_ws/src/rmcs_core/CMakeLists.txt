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
set(LIBRMCS_SOURCE_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../librmcs-sdk-src-3.1.0-beta.1"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../librmcs-sdk-src-3.1.0"
)
set(LIBRMCS_SOURCE_DIR "")
foreach(candidate IN LISTS LIBRMCS_SOURCE_CANDIDATES)
  if(EXISTS "${candidate}/CMakeLists.txt"
     AND EXISTS "${candidate}/host/include/librmcs/agent/rmcs_board.hpp")
    set(LIBRMCS_SOURCE_DIR "${candidate}")
    break()
  endif()
endforeach()

if(LIBRMCS_SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR
    "Expected a librmcs source tree that provides librmcs/agent/rmcs_board.hpp. "
    "Checked candidates: ${LIBRMCS_SOURCE_CANDIDATES}"
  )
endif()

message(STATUS "Using librmcs SDK source from: ${LIBRMCS_SOURCE_DIR}")

fetchcontent_declare(
  librmcs
  SOURCE_DIR ${LIBRMCS_SOURCE_DIR}
)
fetchcontent_makeavailable(librmcs)

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
