cmake_minimum_required(VERSION 3.8)
project(rmcs_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add rmcs_controller library related compile commands
generate_parameter_library(rmcs_controller_parameters
  src/rmcs_controller.yaml
  include/rmcs_controller/validate_rmcs_controller_parameters.hpp
)
add_library(
  rmcs_controller
  SHARED
  src/rmcs_controller.cpp
)
target_include_directories(rmcs_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(rmcs_controller rmcs_controller_parameters)
ament_target_dependencies(rmcs_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(rmcs_controller PRIVATE "RMCS_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface rmcs_controller.xml)

install(
  TARGETS
  rmcs_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Removed Tests #
ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  rmcs_controller
)

ament_package()
