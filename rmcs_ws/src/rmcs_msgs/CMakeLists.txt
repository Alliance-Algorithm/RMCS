cmake_minimum_required(VERSION 3.12)
project(rmcs_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB RMCS_MSGS_MSGS "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

foreach(MSG ${RMCS_MSGS_MSGS})
  get_filename_component(MSG_NAME ${MSG} NAME_WE)
  set(MSG_PATH "${MSG_PATH};msg/${MSG_NAME}.msg")
endforeach()

include_directories(${PROJECT_SOURCE_DIR}/include)

ament_auto_package()