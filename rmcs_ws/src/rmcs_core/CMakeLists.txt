cmake_minimum_required(VERSION 3.28)
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
include(FetchContent)
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
target_link_libraries(${PROJECT_NAME} librmcs-sdk atomic)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

if(BUILD_TESTING)
  enable_testing()
  add_executable(gimbal_dbus_test test/gimbal_dbus_test.cpp)
  target_include_directories(gimbal_dbus_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME gimbal_dbus_test COMMAND gimbal_dbus_test)
  add_executable(big_yaw_stop_test test/big_yaw_stop_test.cpp)
  target_include_directories(big_yaw_stop_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME big_yaw_stop_test COMMAND big_yaw_stop_test)
  add_executable(coordinated_yaw_test test/coordinated_yaw_test.cpp)
  target_include_directories(coordinated_yaw_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME coordinated_yaw_test COMMAND coordinated_yaw_test)
  add_executable(big_yaw_identification_waveform_test test/big_yaw_identification_waveform_test.cpp)
  target_include_directories(big_yaw_identification_waveform_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME big_yaw_identification_waveform_test COMMAND big_yaw_identification_waveform_test)
  add_executable(small_yaw_damping_test test/small_yaw_damping_test.cpp)
  target_include_directories(small_yaw_damping_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME small_yaw_damping_test COMMAND small_yaw_damping_test)
  add_executable(small_yaw_robust_torque_slew_test test/small_yaw_robust_torque_slew_test.cpp)
  target_include_directories(small_yaw_robust_torque_slew_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME small_yaw_robust_torque_slew_test COMMAND small_yaw_robust_torque_slew_test)
  add_executable(small_yaw_position_hold_test test/small_yaw_position_hold_test.cpp)
  target_include_directories(small_yaw_position_hold_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME small_yaw_position_hold_test COMMAND small_yaw_position_hold_test)
  add_executable(small_yaw_identification_test test/small_yaw_identification_test.cpp)
  target_include_directories(small_yaw_identification_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME small_yaw_identification_test COMMAND small_yaw_identification_test)
  add_executable(big_yaw_position_reference_test test/big_yaw_position_reference_test.cpp)
  target_include_directories(big_yaw_position_reference_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME big_yaw_position_reference_test COMMAND big_yaw_position_reference_test)
  add_executable(joint_motor_angle_test test/joint_motor_angle_test.cpp)
  target_include_directories(joint_motor_angle_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME joint_motor_angle_test COMMAND joint_motor_angle_test)
  add_executable(joint2_lift_pid_test test/joint2_lift_pid_test.cpp)
  target_include_directories(joint2_lift_pid_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME joint2_lift_pid_test COMMAND joint2_lift_pid_test)
  add_executable(joint2_encoder_velocity_test test/joint2_encoder_velocity_test.cpp)
  target_include_directories(joint2_encoder_velocity_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME joint2_encoder_velocity_test COMMAND joint2_encoder_velocity_test)
  add_executable(robust_calculator_test test/robust_calculator_test.cpp)
  target_include_directories(robust_calculator_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
  add_test(NAME robust_calculator_test COMMAND robust_calculator_test)
endif()

ament_auto_package()
