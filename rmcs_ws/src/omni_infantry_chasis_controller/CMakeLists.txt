cmake_minimum_required(VERSION 3.8)
project(omni_infantry_chasis_controller)

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
  rmcs_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add omni_infantry_chasis_controller library related compile commands
generate_parameter_library(omni_infantry_chasis_controller_parameters
  src/omni_infantry_chasis_controller.yaml
  include/omni_infantry_chasis_controller/validate_omni_infantry_chasis_controller_parameters.hpp
)
add_library(
  omni_infantry_chasis_controller
  SHARED
  src/omni_infantry_chasis_controller.cpp
  src/chasiscontroll.cpp
)
target_include_directories(omni_infantry_chasis_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(omni_infantry_chasis_controller omni_infantry_chasis_controller_parameters)
ament_target_dependencies(omni_infantry_chasis_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(omni_infantry_chasis_controller PRIVATE "OMNI_INFANTRY_CHASIS_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface omni_infantry_chasis_controller.xml)

install(
  TARGETS
  omni_infantry_chasis_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  omni_infantry_chasis_controller
)

ament_package()
