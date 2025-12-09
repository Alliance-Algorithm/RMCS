cmake_minimum_required(VERSION 3.12)
project(rmcs_utility)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE PROJECT_SOURCE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/include/*.cpp
    ${PROJECT_SOURCE_DIR}/include/*.cxx
    ${PROJECT_SOURCE_DIR}/include/*.cc
)
add_library(
    rmcs_utility_lib
    ${PROJECT_SOURCE}
)
target_link_libraries(
    rmcs_utility_lib
    rclcpp::rclcpp
)

install(
    TARGETS rmcs_utility_lib
    DESTINATION lib
)

ament_auto_package()

add_executable(
    test_rclcpp
    test/rclcpp.cpp
)
target_link_libraries(
    test_rclcpp
    rmcs_utility_lib
    rclcpp::rclcpp
)
