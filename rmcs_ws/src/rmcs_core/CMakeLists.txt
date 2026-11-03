cmake_minimum_required(VERSION 3.24)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)

include(FetchContent)
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

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk OpenSSL::Crypto nlohmann_json::nlohmann_json)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rmcs_rl REQUIRED)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    ament_add_gtest(wheel_leg_imu test/wheel_leg_imu.cpp)
    target_include_directories(wheel_leg_imu PRIVATE src)
    target_compile_definitions(wheel_leg_imu PRIVATE
        RMCS_TRADITIONAL_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-traditional.yaml"
        RMCS_PRONE_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-prone-v4.yaml"
        RMCS_WHEELBIPE_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-wheelbipe.yaml"
        RMCS_RECOVERY_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-wheelbipe-recovery.yaml")
    target_link_libraries(wheel_leg_imu ${PROJECT_NAME})
    ament_add_gtest(wheel_leg_feedback test/wheel_leg_feedback.cpp)
    target_include_directories(wheel_leg_feedback PRIVATE src)
    target_link_libraries(wheel_leg_feedback ${PROJECT_NAME})
    ament_add_gtest(wheel_leg_algorithms test/wheel_leg_algorithms.cpp)
    target_include_directories(wheel_leg_algorithms PRIVATE src)
    foreach(probe observation gas_spring traditional execution)
        add_executable(wheel_leg_${probe}_parity tools/wheel_leg_${probe}_parity.cpp)
        target_include_directories(wheel_leg_${probe}_parity PRIVATE src)
    endforeach()
    add_test(NAME wheel_leg_golden_parity COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/test/verify_wheel_leg_baseline.py
        $<TARGET_FILE:wheel_leg_observation_parity> $<TARGET_FILE:wheel_leg_gas_spring_parity> $<TARGET_FILE:wheel_leg_traditional_parity> $<TARGET_FILE:wheel_leg_execution_parity>
        ${PROJECT_SOURCE_DIR}/test/fixtures/wheel_leg)
    add_library(wheel_leg_self_righting_probe SHARED tools/wheel_leg_self_righting_abi.cpp)
    target_include_directories(wheel_leg_self_righting_probe PRIVATE src)
    foreach(probe core pose_routes abi)
        add_executable(self_righting_${probe}_test test/self_righting_${probe}_test.cpp)
        target_include_directories(self_righting_${probe}_test PRIVATE src)
        add_test(NAME self_righting_${probe} COMMAND self_righting_${probe}_test)
    endforeach()
    target_link_libraries(self_righting_abi_test wheel_leg_self_righting_probe)
    ament_add_gtest(wheel_leg_integration test/wheel_leg_integration.cpp TIMEOUT 45)
    target_include_directories(wheel_leg_integration PRIVATE src ${PROJECT_SOURCE_DIR}/../rmcs_executor/src)
    target_compile_definitions(wheel_leg_integration PRIVATE
        RMCS_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-prone-v4.yaml")
    target_link_libraries(wheel_leg_integration ${PROJECT_NAME})
    foreach(profile traditional wheelbipe)
        ament_add_gtest_test(wheel_leg_integration TEST_NAME wheel_leg_${profile}_integration
            TIMEOUT 45 ENV "RMCS_WHEEL_LEG_PROFILE=${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-${profile}.yaml")
    endforeach()
    foreach(profile prone-v4 wheelbipe)
        ament_add_gtest_test(wheel_leg_integration TEST_NAME wheel_leg_${profile}_pd_failure
            TIMEOUT 45 ENV "RMCS_MANUAL_INFERENCE=1"
            "RMCS_WHEEL_LEG_PROFILE=${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-${profile}.yaml")
    endforeach()
    ament_add_gtest(wheel_leg_bridge_epoch test/wheel_leg_bridge_epoch.cpp TIMEOUT 15)
    target_include_directories(wheel_leg_bridge_epoch PRIVATE ${PROJECT_SOURCE_DIR}/../rmcs_executor/src)
    target_compile_definitions(wheel_leg_bridge_epoch PRIVATE
        RMCS_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-prone-v4.yaml")
    target_link_libraries(wheel_leg_bridge_epoch ${PROJECT_NAME})
    ament_add_gtest(deformable_policy_bridge_integration test/deformable_policy_bridge_integration.cpp)
    target_include_directories(deformable_policy_bridge_integration PRIVATE
        ${PROJECT_SOURCE_DIR}/../rmcs_executor/src)
    target_compile_definitions(deformable_policy_bridge_integration PRIVATE
        RMCS_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/deformable-infantry-omni-rl.yaml")
    target_link_libraries(deformable_policy_bridge_integration ${PROJECT_NAME})
endif()

ament_auto_package()
