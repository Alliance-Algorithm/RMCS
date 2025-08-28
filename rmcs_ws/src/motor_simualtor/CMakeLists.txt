cmake_minimum_required(VERSION 3.8)
project(motor_simualtor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include/)

add_executable(motor_simualtor_node src/motor_simulator_node.cpp)

ament_target_dependencies(motor_simualtor_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  motor_simualtor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# 包的元信息
ament_package()