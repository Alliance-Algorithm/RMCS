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
install(FILES models/wheel_leg/policy.onnx models/wheel_leg/SHA256SUMS
  DESTINATION share/${PROJECT_NAME}/models/wheel_leg)
install(FILES
  models/wheel_leg/deployment/v6_recovery_profiles_v1.json
  models/wheel_leg/deployment/v6_height_lookup_v1.json
  DESTINATION share/${PROJECT_NAME}/models/wheel_leg/deployment)
pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()
