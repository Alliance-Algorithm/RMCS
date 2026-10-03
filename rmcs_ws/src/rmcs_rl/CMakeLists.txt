cmake_minimum_required(VERSION 3.24)
project(rmcs_rl)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)
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
  src/v6_recovery_controller.cpp
  src/v6_recovery_profile.cpp
  src/v6_recovery_observer.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${ORT_ROOT}/include)
target_link_libraries(${PROJECT_NAME} ${ORT_ROOT}/lib/libonnxruntime.so OpenSSL::Crypto nlohmann_json::nlohmann_json)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
install(DIRECTORY ${ORT_ROOT}/lib/ DESTINATION lib FILES_MATCHING PATTERN "*.so*")
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING PATTERN "*.onnx" PATTERN "*.json" PATTERN "*.md"
  PATTERN "*.py" PATTERN "SHA256SUMS")
pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(POLICY_ABI_FIXTURE "${CMAKE_CURRENT_BINARY_DIR}/policy_abi_fixture.hpp")
  add_custom_command(OUTPUT "${POLICY_ABI_FIXTURE}"
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/test/generate_policy_abi_fixture.py"
      "${PROJECT_SOURCE_DIR}/models/wheel_leg/deployment/abi_test_vectors.json"
      "${POLICY_ABI_FIXTURE}"
    DEPENDS test/generate_policy_abi_fixture.py models/wheel_leg/deployment/abi_test_vectors.json
    VERBATIM)
  ament_add_gtest(onnx_policy_abi_test test/onnx_policy_abi_test.cpp
    src/onnx_policy.cpp "${POLICY_ABI_FIXTURE}")
  target_include_directories(onnx_policy_abi_test PRIVATE src "${CMAKE_CURRENT_BINARY_DIR}" "${ORT_ROOT}/include")
  target_link_libraries(onnx_policy_abi_test "${ORT_ROOT}/lib/libonnxruntime.so" OpenSSL::Crypto)
  target_compile_definitions(onnx_policy_abi_test PRIVATE
    RL_CONTROLLER_TEST_MODEL="${PROJECT_SOURCE_DIR}/models/wheel_leg/policy.onnx"
    RL_CONTROLLER_LEGACY_TEST_MODEL="${PROJECT_SOURCE_DIR}/models/wheel_leg/legacy_v5/policy.onnx")
  set(CONTROLLER_ABI_FIXTURE "${CMAKE_CURRENT_BINARY_DIR}/controller_abi_fixture.hpp")
  add_custom_command(OUTPUT "${CONTROLLER_ABI_FIXTURE}"
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/test/generate_policy_abi_fixture.py"
      "${PROJECT_SOURCE_DIR}/test/v6_controller_reference.json" "${CONTROLLER_ABI_FIXTURE}"
    DEPENDS test/generate_policy_abi_fixture.py test/v6_controller_reference.json
    VERBATIM)
  ament_add_gtest(v6_controller_abi_test test/v6_controller_abi_test.cpp "${CONTROLLER_ABI_FIXTURE}")
  target_include_directories(v6_controller_abi_test PRIVATE src "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(v6_controller_abi_test ${PROJECT_NAME})
  ament_target_dependencies(v6_controller_abi_test
    rclcpp rmcs_executor rmcs_description rmcs_msgs rmcs_utility)
  target_compile_definitions(v6_controller_abi_test PRIVATE
    RL_CONTROLLER_TEST_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-rl.yaml"
    RL_CONTROLLER_TEST_MODEL="${PROJECT_SOURCE_DIR}/models/wheel_leg/policy.onnx")
  ament_add_gtest(v6_recovery_controller_test test/v6_recovery_controller_test.cpp src/v6_recovery_controller.cpp)
  target_include_directories(v6_recovery_controller_test PRIVATE src)
  target_link_libraries(v6_recovery_controller_test nlohmann_json::nlohmann_json)
  target_compile_definitions(v6_recovery_controller_test PRIVATE
    V6_RECOVERY_REFERENCE_JSON="${PROJECT_SOURCE_DIR}/test/v6_recovery_reference.json")
  ament_add_gtest(v6_recovery_observer_test test/v6_recovery_observer_test.cpp
    src/v6_recovery_observer.cpp src/v6_recovery_profile.cpp)
  target_include_directories(v6_recovery_observer_test PRIVATE src)
  target_link_libraries(v6_recovery_observer_test OpenSSL::Crypto nlohmann_json::nlohmann_json)
  target_compile_definitions(v6_recovery_observer_test PRIVATE
    V6_RECOVERY_PROFILE="${PROJECT_SOURCE_DIR}/models/wheel_leg/deployment/v6_recovery_profiles_v1.json"
    V6_HEIGHT_LOOKUP="${PROJECT_SOURCE_DIR}/models/wheel_leg/deployment/v6_height_lookup_v1.json"
    V6_OBSERVER_REFERENCE="${PROJECT_SOURCE_DIR}/test/v6_recovery_observer_reference.json")
  ament_add_gtest(control_interval_test test/control_interval_test.cpp)
  target_include_directories(control_interval_test PRIVATE src)
  ament_add_gtest(recovery_controller_test test/recovery_controller_test.cpp
    src/recovery_controller.cpp src/recovery_observer.cpp)
  target_include_directories(recovery_controller_test PRIVATE src)
  ament_add_gtest(recovery_sensor_guard_test test/recovery_sensor_guard_test.cpp)
  target_include_directories(recovery_sensor_guard_test PRIVATE src)
  ament_add_gtest(recovery_parity_test test/recovery_parity_test.cpp src/recovery_controller.cpp)
  target_include_directories(recovery_parity_test PRIVATE src)
  ament_add_gtest(recovery_observer_test test/recovery_observer_test.cpp
    src/recovery_observer.cpp src/recovery_controller.cpp)
  target_include_directories(recovery_observer_test PRIVATE src)
  foreach(component_test IN ITEMS rl_controller_test recovery_handover_test)
    ament_add_gtest(${component_test} test/${component_test}.cpp)
    target_include_directories(${component_test} PRIVATE src)
    target_link_libraries(${component_test} ${PROJECT_NAME})
    ament_target_dependencies(${component_test}
      rclcpp rmcs_executor rmcs_description rmcs_msgs rmcs_utility)
    target_compile_definitions(${component_test} PRIVATE
      RL_CONTROLLER_TEST_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-rl.yaml"
      RL_CONTROLLER_TEST_MODEL="${PROJECT_SOURCE_DIR}/models/wheel_leg/policy.onnx"
      RL_CONTROLLER_LEGACY_TEST_MODEL="${PROJECT_SOURCE_DIR}/models/wheel_leg/legacy_v5/policy.onnx")
  endforeach()
endif()

ament_auto_package()
