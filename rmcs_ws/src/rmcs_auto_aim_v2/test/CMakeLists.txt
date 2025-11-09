cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2_tests)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RMCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(RMCS_SRC_DIR "${RMCS_SOURCE_DIR}/src")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(hikcamera QUIET)

set(HIKCAMERA_AVAILABLE OFF)
if(hikcamera_FOUND)
    set(HIKCAMERA_AVAILABLE ON)
elseif(DEFINED hikcamera_LIBRARIES)
    set(HIKCAMERA_AVAILABLE ON)
endif()

include_directories(
    ${RMCS_SRC_DIR}
    ${hikcamera_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

ament_add_gtest(
    test_duck_type
    ${TEST_DIR}/duck_type.cpp
)

ament_add_gtest(
    test_pipeline
    ${TEST_DIR}/pipeline.cpp
)

ament_add_gtest(
    test_serializable
    ${TEST_DIR}/serializable.cpp
)
target_link_libraries(
    test_serializable
    rclcpp::rclcpp
    yaml-cpp::yaml-cpp
)

ament_add_gtest(
    test_interprocess
    ${TEST_DIR}/interprocess.cpp
)

ament_add_gtest(
    test_model_infer
    ${TEST_DIR}/model_infer.cpp
    ${RMCS_SRC_DIR}/modules/identifier/model.cpp
    ${RMCS_SRC_DIR}/utility/image.cpp
)
target_link_libraries(
    test_model_infer
    yaml-cpp::yaml-cpp
    openvino::runtime
    ${OpenCV_LIBS}
)

if(HIKCAMERA_AVAILABLE)
    add_executable(
        example_hikcamera
        ${TEST_DIR}/hikcamera.cpp
    )
    target_link_libraries(
        example_hikcamera
        ${OpenCV_LIBS}
        ${hikcamera_LIBRARIES}
    )

    add_executable(
        example_streaming
        ${TEST_DIR}/streaming.cpp
        ${RMCS_SRC_DIR}/modules/capturer/hikcamera.cpp
        ${RMCS_SRC_DIR}/modules/debug/visualization/stream_session.cpp
        ${RMCS_SRC_DIR}/modules/debug/visualization/stream_context.cpp
        ${RMCS_SRC_DIR}/utility/image.cpp
    )
    target_link_libraries(
        example_streaming
        rclcpp::rclcpp
        ${OpenCV_LIBS}
        ${hikcamera_LIBRARIES}
    )
else()
    message(
        STATUS
        "hikcamera SDK 未检测到，跳过 example_hikcamera 与 example_streaming 目标的构建")
endif()
