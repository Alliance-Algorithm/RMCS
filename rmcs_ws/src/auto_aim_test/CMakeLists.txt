cmake_minimum_required(VERSION 3.22)
project(auto_aim_test VERSION 0.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

find_package(OpenCV 4.5 REQUIRED)
find_package(pluginlib REQUIRED)

if(CMAKE_SYSROOT)
  set(HIK_MVS_SDK_ROOT "${CMAKE_SYSROOT}/opt/mvs-usb3-core")
else()
  set(HIK_MVS_SDK_ROOT "/opt/mvs-usb3-core")
endif()

find_package(MVSUSB3Core REQUIRED CONFIG
  PATHS "${HIK_MVS_SDK_ROOT}" NO_DEFAULT_PATH
)

include(FetchContent)
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
  librmcs
  URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.2.0rc0/librmcs-sdk-src-3.2.0-rc.0.zip
  URL_HASH SHA256=accefeb3a46d7da32a215765d0907de36c25353dd7951e342aaddfe98d2971e5
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)

ament_auto_add_library(
  ${PROJECT_NAME}_component SHARED
  src/component.cpp
)

target_include_directories(
  ${PROJECT_NAME}_component
  SYSTEM PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(
  ${PROJECT_NAME}_component
  PRIVATE
  ${LIBRMCS_DEV10_ROOT}/core/include
  ${LIBRMCS_DEV10_ROOT}/host/include
)

target_link_libraries(
  ${PROJECT_NAME}_component
  ${OpenCV_LIBS}
  librmcs-sdk
  MVSUSB3Core::MVSUSB3Core
)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

install(
  TARGETS ${PROJECT_NAME}_component
  DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
