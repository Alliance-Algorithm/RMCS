cmake_minimum_required(VERSION 3.12)
project(rmcs_executor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g1 -rdynamic -O2 -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_library (
  ${PROJECT_NAME}_lib
  SHARED
  src/component.cpp
)

ament_auto_add_executable (
  ${PROJECT_NAME}_exe
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME}_lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set_property(TARGET ${PROJECT_NAME}_lib PROPERTY OUTPUT_NAME ${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME}_exe PROPERTY OUTPUT_NAME ${PROJECT_NAME})

ament_auto_package()
