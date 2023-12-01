cmake_minimum_required(VERSION 3.8)
project(serial_deliver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_executable (
  ${PROJECT_NAME}
  src/main.cpp
)

target_include_directories (${PROJECT_NAME}
  PUBLIC
  include
)

ament_auto_package()
