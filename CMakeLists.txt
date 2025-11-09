cmake_minimum_required(VERSION 3.22)
project(alliance_auto_aim VERSION 0.0.1 LANGUAGES CXX)

# 编译选项
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 源码与头文件
include_directories(./include ./src)
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp src/*.cc)
add_library(alliance_auto_aim SHARED ${SOURCES})
target_include_directories(alliance_auto_aim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 第三方依赖
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# 链接依赖（使用目标名，自动传递）
target_link_libraries(alliance_auto_aim PUBLIC
  fmt::fmt
  spdlog::spdlog
  ${OpenCV_LIBS}
  Eigen3::Eigen
  TBB::tbb
  openvino::runtime
  Ceres::ceres
  yaml-cpp
)

# 安装规则
include(GNUInstallDirs)
install(TARGETS alliance_auto_aim
  EXPORT alliance_auto_aimTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 导出配置
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/alliance_auto_aimConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/alliance_auto_aim
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/alliance_auto_aimConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/alliance_auto_aim
)

install(EXPORT alliance_auto_aimTargets
  FILE alliance_auto_aimTargets.cmake
  NAMESPACE alliance_auto_aim::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/alliance_auto_aim
)


if(ENABLE_TESTS)
    message(STATUS "Tests are enabled")
    enable_testing()  # 启用 CTest
    add_subdirectory(tests)
else()
    message(STATUS "Tests are disabled. Use -DENABLE_TESTS=ON to enable.")
endif()