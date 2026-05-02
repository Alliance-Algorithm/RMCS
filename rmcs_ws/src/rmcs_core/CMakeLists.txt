cmake_minimum_required(VERSION 3.28)
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
include(FetchContent)
ament_auto_find_build_dependencies()




set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
  librmcs
  URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.1.0/librmcs-sdk-src-3.1.0.zip
  URL_HASH SHA256=07107e251745ddb23f7b3e39edec5d6910be1a197025d167ec9849c5c80dd954
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)



find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen librmcs-sdk atomic)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()
