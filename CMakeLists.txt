cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

## 依赖查找
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

## ROS2 相关依赖
find_package(type_description_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rmcs_executor REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(fast_tf REQUIRED)
find_package(hikcamera REQUIRED)

## 目录包含
include_directories(
    ${PROJECT_SOURCE_DIR}/src/

    ${type_description_interfaces_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${ament_index_cpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}

    ${rmcs_executor_INCLUDE_DIRS}
    ${rmcs_description_INCLUDE_DIRS}
    ${fast_tf_INCLUDE_DIRS}
)

## 代码资源搜索
file(GLOB_RECURSE AUTO_AIM_KERNEL
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/kernel/*.cpp
    ${PROJECT_SOURCE_DIR}/src/kernel/*.cc
)
file(GLOB_RECURSE AUTO_AIM_MODULE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/modules/*.cpp
    ${PROJECT_SOURCE_DIR}/src/modules/*.cc
    ${PROJECT_SOURCE_DIR}/src/utility/*.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/*.cc
)

## 核心构建: kernel and module
add_library(
    ${PROJECT_NAME}_kernel SHARED
    ${AUTO_AIM_KERNEL}
)
target_link_libraries(
    ${PROJECT_NAME}_kernel
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
)
add_library(
    ${PROJECT_NAME}_module SHARED
    ${AUTO_AIM_MODULE}
)
target_link_libraries(
    ${PROJECT_NAME}_module
    yaml-cpp::yaml-cpp
    ament_index_cpp::ament_index_cpp
    openvino::runtime
    rclcpp::rclcpp
    ${OpenCV_LIBS}
    ${hikcamera_LIBRARIES}
)

## 组件构建: component
add_library(
    ${PROJECT_NAME}_component SHARED
    ${PROJECT_SOURCE_DIR}/src/component.cpp
)
target_link_libraries(
    ${PROJECT_NAME}_component
    ${PROJECT_NAME}_kernel
    ${PROJECT_NAME}_module
    ${rmcs_executor_LIBRARIES}
)

## 运行时构建: runtime
add_executable(
    ${PROJECT_NAME}_runtime
    ${PROJECT_SOURCE_DIR}/src/runtime.cpp
)
target_link_libraries(
    ${PROJECT_NAME}_runtime
    ${PROJECT_NAME}_kernel
    ${PROJECT_NAME}_module
)

## 安装配置
install(
    TARGETS
        ${PROJECT_NAME}_component
        ${PROJECT_NAME}_kernel
        ${PROJECT_NAME}_module
    DESTINATION
        lib/
)
install(
    DIRECTORY
        config/
        launch/
        models/
    DESTINATION
        share/${PROJECT_NAME}
)
install(
    TARGETS
        ${PROJECT_NAME}_runtime
    DESTINATION
        lib/${PROJECT_NAME}
)

## plugins for rmcs_executor
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
    rmcs_executor plugins.xml
)

find_package(ament_cmake REQUIRED)
ament_package()
