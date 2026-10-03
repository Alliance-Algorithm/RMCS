cmake_minimum_required(VERSION 3.24)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(FetchContent)
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
    librmcs
    URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.3.0b1/librmcs-sdk-src-3.3.0-beta.1-debug.zip
    URL_HASH SHA256=446d632b23b652dd309f35075ef90928dee89f42e8ec71417fa2439445e0b4ff
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

add_executable(wheel_leg_recording_preview tools/wheel_leg_recording_preview.cpp)
target_include_directories(wheel_leg_recording_preview PRIVATE ${PROJECT_SOURCE_DIR}/src)
install(TARGETS wheel_leg_recording_preview DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(wheel_leg_chassis_controller_test test/wheel_leg_chassis_controller_test.cpp)
    target_include_directories(wheel_leg_chassis_controller_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    ament_target_dependencies(wheel_leg_chassis_controller_test
        rclcpp pluginlib rmcs_executor rmcs_msgs rmcs_description rmcs_utility)
    ament_add_gtest(dm_motor_test test/dm_motor_test.cpp)
    target_include_directories(dm_motor_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    ament_target_dependencies(dm_motor_test rmcs_executor rmcs_msgs)
    ament_add_gtest(wheel_leg_sensor_snapshot_test test/wheel_leg_sensor_snapshot_test.cpp)
    target_include_directories(wheel_leg_sensor_snapshot_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    ament_target_dependencies(wheel_leg_sensor_snapshot_test rmcs_executor rmcs_msgs rmcs_utility)
    ament_add_gtest(wheel_leg_pair_identification_test test/wheel_leg_pair_identification_test.cpp)
    target_include_directories(wheel_leg_pair_identification_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    ament_target_dependencies(wheel_leg_pair_identification_test rmcs_msgs)
    ament_add_gtest(wheel_leg_pair_recording_test test/wheel_leg_pair_recording_test.cpp)
    target_include_directories(wheel_leg_pair_recording_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    ament_add_gtest(wheel_leg_pair_controller_test test/wheel_leg_pair_controller_test.cpp)
    target_include_directories(wheel_leg_pair_controller_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    target_compile_definitions(wheel_leg_pair_controller_test PRIVATE
        PAIR_CONTROLLER_TEST_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-pair-pd-loaded-identification.yaml"
        V6_CONTROLLER_TEST_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-v6-L01-identification.yaml")
    ament_target_dependencies(wheel_leg_pair_controller_test
        rclcpp pluginlib rmcs_executor rmcs_msgs rmcs_utility)
    ament_add_gtest(wheel_leg_wheel_identification_test test/wheel_leg_wheel_identification_test.cpp)
    target_include_directories(wheel_leg_wheel_identification_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    ament_add_gtest(wheel_leg_wheel_controller_test test/wheel_leg_wheel_controller_test.cpp)
    target_include_directories(wheel_leg_wheel_controller_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
    target_compile_definitions(wheel_leg_wheel_controller_test PRIVATE
        WHEEL_CONTROLLER_TEST_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-wheel-identification.yaml")
    ament_target_dependencies(wheel_leg_wheel_controller_test
        rclcpp pluginlib rmcs_executor rmcs_msgs rmcs_utility)
endif()

ament_auto_package()
