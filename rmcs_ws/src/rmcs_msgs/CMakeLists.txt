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
find_package(rosidl_default_generators REQUIRED)

file(GLOB RMCS_MSGS_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "msg/*.msg")
rosidl_generate_interfaces(${PROJECT_NAME} ${RMCS_MSGS_MSGS}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
# Existing RMCS headers live in include/rmcs_msgs/, alongside generated msg headers
# in the scoped install layout. Keep both visible through the ROSIDL C++ target.
target_include_directories(${PROJECT_NAME}__rosidl_generator_cpp INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_dependencies(rosidl_default_runtime)

include_directories(${PROJECT_SOURCE_DIR}/include)

ament_auto_package()
