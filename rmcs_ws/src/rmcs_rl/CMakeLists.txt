cmake_minimum_required(VERSION 3.24)
project(rmcs_rl)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The official x86_64 release contains headers and a prebuilt shared library (no
# CMakeLists.txt). ONNXRUNTIME_ROOT permits a verified offline copy.
if(DEFINED ONNXRUNTIME_ROOT AND EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  set(ORT_ROOT "${ONNXRUNTIME_ROOT}")
else()
  include(FetchContent)
  FetchContent_Declare(ort_binary
    URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-1.20.0.tgz
    URL_HASH SHA256=aa70d48b22e264b82e83f63245b51ddc9a47ae4a3a66903efaff1ba68b7b5930
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(ort_binary)
  set(ORT_ROOT "${ort_binary_SOURCE_DIR}")
endif()
if(NOT EXISTS "${ORT_ROOT}/lib/libonnxruntime.so")
  message(FATAL_ERROR "ONNX Runtime 1.20.0 x86_64 not found; set ONNXRUNTIME_ROOT")
endif()
file(STRINGS "${ORT_ROOT}/VERSION_NUMBER" ORT_VERSION LIMIT_COUNT 1)
if(NOT ORT_VERSION STREQUAL "1.20.0")
  message(FATAL_ERROR "Expected ONNX Runtime 1.20.0, found ${ORT_VERSION}")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rl_controller.cpp
  src/configuration.cpp
  src/controller_recovery.cpp
  src/observation.cpp
  src/action.cpp
  src/onnx_policy.cpp
  src/recovery_controller.cpp
  src/recovery_observer.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${ORT_ROOT}/include)
target_link_libraries(${PROJECT_NAME} ${ORT_ROOT}/lib/libonnxruntime.so)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
install(DIRECTORY ${ORT_ROOT}/lib/ DESTINATION lib FILES_MATCHING PATTERN "*.so*")
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING PATTERN "*.onnx")
pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(control_interval_test test/control_interval_test.cpp)
  target_include_directories(control_interval_test PRIVATE src)
  ament_add_gtest(recovery_controller_test test/recovery_controller_test.cpp
    src/recovery_controller.cpp src/recovery_observer.cpp)
  target_include_directories(recovery_controller_test PRIVATE src)
  ament_add_gtest(rl_controller_test test/rl_controller_test.cpp)
  target_include_directories(rl_controller_test PRIVATE src)
  target_link_libraries(rl_controller_test ${PROJECT_NAME})
  ament_target_dependencies(rl_controller_test
    rclcpp rmcs_executor rmcs_description rmcs_msgs rmcs_utility)
  target_compile_definitions(rl_controller_test PRIVATE
    RL_CONTROLLER_TEST_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-rl.yaml"
    RL_CONTROLLER_TEST_MODEL="${PROJECT_SOURCE_DIR}/models/wheel_leg/policy.onnx")
endif()

ament_auto_package()
