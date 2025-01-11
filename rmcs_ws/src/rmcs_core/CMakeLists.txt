cmake_minimum_required(VERSION 3.12)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()


find_package(rosbag2_cpp REQUIRED)
include_directories(${rosbag2_cpp_INCLUDE_DIRS})
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library (
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} -lusb-1.0)
target_link_libraries(${PROJECT_NAME} ${rosbag2_cpp_LIBRARIES})
file(GLOB DAT_FILES "${CMAKE_SOURCE_DIR}/src/*.dat")

install(FILES ${DAT_FILES} DESTINATION share/${PROJECT_NAME}/)




pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()