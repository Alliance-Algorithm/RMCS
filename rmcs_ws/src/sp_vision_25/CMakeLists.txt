cmake_minimum_required(VERSION 3.16.3)

project(sp_vision_25)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hikcamera REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(rmcs_executor REQUIRED)
find_package(rmcs_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/" CACHE PATH "OpenVINO CMake directory")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

include(GNUInstallDirs)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/omniperception)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hero_auto_aim_bridge.cpp
  src/sp_vision_response_test_bridge.cpp
  $<TARGET_OBJECTS:tools>
  $<TARGET_OBJECTS:auto_aim>
  $<TARGET_OBJECTS:omniperception>
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
)

target_link_libraries(
  ${PROJECT_NAME}
  sp_vision_25_io
  fmt::fmt
  spdlog::spdlog
  yaml-cpp
  openvino::runtime
  tinympcstatic
  ${OpenCV_LIBS}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ament_index_cpp
  pluginlib
  rclcpp
  rmcs_description
  rmcs_executor
  rmcs_msgs
)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  DIRECTORY assets configs
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY io tasks tools
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(FILES readme.md DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  OpenCV
  OpenVINO
  ament_index_cpp
  fmt
  hikcamera
  pluginlib
  rclcpp
  rmcs_description
  rmcs_executor
  rmcs_msgs
  spdlog
  yaml-cpp
)

ament_package()
