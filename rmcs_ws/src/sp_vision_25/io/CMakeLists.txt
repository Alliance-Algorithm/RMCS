cmake_minimum_required(VERSION 3.16)

find_package(hikcamera REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  sp_vision_25_io
  STATIC
  hikrobot/hikrobot.cpp
  usbcamera/usbcamera.cpp
  camera.cpp
)

target_include_directories(
  sp_vision_25_io
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    ${hikcamera_INCLUDE_DIRS}
)

target_link_libraries(sp_vision_25_io PUBLIC ${hikcamera_LIBRARIES} yaml-cpp)
