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

add_executable(wheel_leg_recording_preview tools/wheel_leg_recording_preview.cpp)
target_include_directories(wheel_leg_recording_preview PRIVATE ${PROJECT_SOURCE_DIR}/src)
install(TARGETS wheel_leg_recording_preview DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rmcs_rl REQUIRED)
    ament_add_gtest(wheel_leg_policy_integration test/wheel_leg_policy_integration.cpp)
    target_include_directories(wheel_leg_policy_integration PRIVATE
        src ${PROJECT_SOURCE_DIR}/../rmcs_executor/src)
    target_compile_definitions(wheel_leg_policy_integration PRIVATE
        RMCS_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/wheel-leg-infantry-rl.yaml")
    target_link_libraries(wheel_leg_policy_integration ${PROJECT_NAME})
    ament_add_gtest(deformable_policy_bridge_integration test/deformable_policy_bridge_integration.cpp)
    target_include_directories(deformable_policy_bridge_integration PRIVATE
        ${PROJECT_SOURCE_DIR}/../rmcs_executor/src)
    target_compile_definitions(deformable_policy_bridge_integration PRIVATE
        RMCS_PROFILE="${PROJECT_SOURCE_DIR}/../rmcs_bringup/config/deformable-infantry-omni-rl.yaml")
    target_link_libraries(deformable_policy_bridge_integration ${PROJECT_NAME})
endif()

ament_auto_package()
