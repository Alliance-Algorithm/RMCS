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

# -----------------------------------------------------------------------------
# 机械臂代码（controller/arm）依赖 MoveIt。本分支用于发弹台架，不需要机械臂，
# 默认关闭：既不编译这些源文件，也不去查找 MoveIt。
# 这样产物 librmcs_core.so 完全不依赖 MoveIt，真机也不必再装 1~2GB 的 MoveIt。
# 需要机械臂时：colcon build --cmake-args -DRMCS_WITH_ARM=ON
# -----------------------------------------------------------------------------
option(RMCS_WITH_ARM "Build arm controllers that require MoveIt" OFF)

if(NOT RMCS_WITH_ARM)
  # 让下面的 ament_auto_find_build_dependencies() 找不到这些包，
  # 它们就不会被写进链接列表，也就不会出现在产物的 DT_NEEDED 里。
  foreach(_moveit_pkg
          moveit_core
          moveit_ros_planning_interface
          moveit_ros_move_group
          moveit_ros_planning
          moveit_ros_visualization
          moveit_visual_tools)
    set(CMAKE_DISABLE_FIND_PACKAGE_${_moveit_pkg} TRUE)
  endforeach()
endif()

find_package(ament_cmake_auto REQUIRED)
include(FetchContent)

ament_auto_find_build_dependencies()

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

if(NOT RMCS_WITH_ARM)
  # 把整个 controller/arm 目录从编译列表里剔除
  list(FILTER PROJECT_SOURCE EXCLUDE REGEX "/controller/arm/")
endif()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk atomic)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

install(DIRECTORY src/controller/arm/obstacle/meshes
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
