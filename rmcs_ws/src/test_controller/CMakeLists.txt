cmake_minimum_required(VERSION 3.12)
project(test_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

ament_auto_add_executable (
  ${PROJECT_NAME}
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)

ament_auto_package()