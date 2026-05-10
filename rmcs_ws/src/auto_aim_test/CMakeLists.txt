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

if(CMAKE_SYSROOT)
  set(HIK_MVS_SDK_ROOT "${CMAKE_SYSROOT}/opt/mvs-usb3-core")
else()
  set(HIK_MVS_SDK_ROOT "/opt/mvs-usb3-core")
endif()

find_package(MVSUSB3Core REQUIRED CONFIG
  PATHS "${HIK_MVS_SDK_ROOT}" NO_DEFAULT_PATH
)

set(
  LIBRMCS_DEV10_ROOT
  "/workspaces/RMCS/librmcs_tmp/librmcs-sdk-src-3.2.0-beta.0.dev.10.g72381f1"
  CACHE PATH "Path to librmcs 3.2.0-beta.0.dev.10.g72381f1 source snapshot"
)
set(LIBRMCS_DEV10_BUILD_DIR "${LIBRMCS_DEV10_ROOT}/build-auto-aim-test")
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)

if(NOT EXISTS "${LIBRMCS_DEV10_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "LIBRMCS_DEV10_ROOT does not point to a librmcs source snapshot: ${LIBRMCS_DEV10_ROOT}")
endif()

file(READ "${LIBRMCS_DEV10_ROOT}/CMakeLists.txt" LIBRMCS_DEV10_ROOT_CMAKE)
if(NOT LIBRMCS_DEV10_ROOT_CMAKE MATCHES "LIBRMCS_PROJECT_VERSION \"3\\.2\\.0-beta\\.0\\.dev\\.10\\.g72381f1\"")
  message(FATAL_ERROR "LIBRMCS_DEV10_ROOT must point to librmcs 3.2.0-beta.0.dev.10.g72381f1")
endif()

if(NOT TARGET librmcs-sdk)
  add_subdirectory("${LIBRMCS_DEV10_ROOT}" "${LIBRMCS_DEV10_BUILD_DIR}")
endif()

ament_auto_add_executable(
  ${PROJECT_NAME}_exe
  src/main.cpp
)

target_include_directories(
  ${PROJECT_NAME}_exe
  SYSTEM PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}_exe
  ${OpenCV_LIBS}
  librmcs-sdk
  MVSUSB3Core::MVSUSB3Core
)

target_include_directories(
  ${PROJECT_NAME}_exe
  PRIVATE
  ${LIBRMCS_DEV10_ROOT}/core/include
  ${LIBRMCS_DEV10_ROOT}/host/include
)

set_property(TARGET ${PROJECT_NAME}_exe PROPERTY OUTPUT_NAME ${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
