cmake_minimum_required(VERSION 3.22)
project(rmcs_v6_component_sim_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmcs_executor REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(rmcs_msgs REQUIRED)
find_package(rmcs_utility REQUIRED)
find_package(rmcs_rl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

get_filename_component(RMCS_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_executable(v6_component_sim_bridge v6_component_sim_bridge.cpp)
target_include_directories(v6_component_sim_bridge PRIVATE
  "${RMCS_REPOSITORY_ROOT}/rmcs_ws/src/rmcs_core/src"
  "${RMCS_REPOSITORY_ROOT}/rmcs_ws/src/rmcs_rl/src")
ament_target_dependencies(v6_component_sim_bridge
  rclcpp rmcs_executor rmcs_description rmcs_msgs rmcs_utility rmcs_rl pluginlib)
target_link_libraries(v6_component_sim_bridge nlohmann_json::nlohmann_json OpenSSL::Crypto)
target_compile_options(v6_component_sim_bridge PRIVATE -Wall -Wextra -Wpedantic)

# Prefer the just-built production library over an older installed copy.
set_target_properties(v6_component_sim_bridge PROPERTIES BUILD_RPATH
  "${RMCS_REPOSITORY_ROOT}/rmcs_ws/build/rmcs_rl;${RMCS_REPOSITORY_ROOT}/rmcs_ws/build/rmcs_rl/_deps/ort_binary-src/lib")
