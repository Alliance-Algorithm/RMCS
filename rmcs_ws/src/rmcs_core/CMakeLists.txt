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

# ---- Optional in-process MuJoCo simulation bridge ----
# wheelleg_simulation.cpp is only compiled when libmujoco is available on a
# native build; cross builds for the real robot keep it disabled.
list(FILTER PROJECT_SOURCE EXCLUDE REGEX "/wheelleg_simulation\\.cpp$")

# Archived wheel-leg experiments (torque-level task-space / LQR / test signal)
# are kept for reference but excluded from the default build.
list(FILTER PROJECT_SOURCE EXCLUDE REGEX "/wheelleg/legacy/")

set(RMCS_MUJOCO_ROOT "" CACHE PATH "MuJoCo root containing include/ and libmujoco.so")
if(NOT RMCS_MUJOCO_ROOT)
    if(DEFINED ENV{MUJOCO_ROOT})
        set(RMCS_MUJOCO_ROOT "$ENV{MUJOCO_ROOT}")
    else()
        file(GLOB _rmcs_mujoco_candidates
            "$ENV{HOME}/.venvs/mujoco/lib/python*/site-packages/mujoco")
        if(_rmcs_mujoco_candidates)
            list(GET _rmcs_mujoco_candidates 0 RMCS_MUJOCO_ROOT)
        endif()
        unset(_rmcs_mujoco_candidates)
    endif()
endif()

set(_rmcs_mujoco_library "")
if(RMCS_MUJOCO_ROOT AND EXISTS "${RMCS_MUJOCO_ROOT}/include/mujoco/mujoco.h")
    file(GLOB _rmcs_mujoco_libraries "${RMCS_MUJOCO_ROOT}/libmujoco.so*")
    if(_rmcs_mujoco_libraries)
        list(GET _rmcs_mujoco_libraries 0 _rmcs_mujoco_library)
    endif()
    unset(_rmcs_mujoco_libraries)
endif()

set(_rmcs_mujoco_source "${PROJECT_SOURCE_DIR}/src/hardware/wheelleg_simulation.cpp")

if(_rmcs_mujoco_library AND EXISTS "${_rmcs_mujoco_source}" AND NOT RMCS_TARGET_ARCH)
    set(_rmcs_mujoco_default ON)
else()
    set(_rmcs_mujoco_default OFF)
endif()
option(RMCS_ENABLE_MUJOCO_SIM "Build the in-process MuJoCo simulation bridge"
    ${_rmcs_mujoco_default})

if(RMCS_ENABLE_MUJOCO_SIM AND (NOT _rmcs_mujoco_library OR NOT EXISTS "${_rmcs_mujoco_source}"))
    message(WARNING
        "RMCS_ENABLE_MUJOCO_SIM requested but libmujoco or wheelleg_simulation.cpp is missing; disabling.")
    set(RMCS_ENABLE_MUJOCO_SIM OFF)
endif()

if(RMCS_ENABLE_MUJOCO_SIM)
    list(APPEND PROJECT_SOURCE "${_rmcs_mujoco_source}")
endif()

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk)

if(RMCS_ENABLE_MUJOCO_SIM)
    target_include_directories(${PROJECT_NAME} PRIVATE "${RMCS_MUJOCO_ROOT}/include")
    target_link_libraries(${PROJECT_NAME} "${_rmcs_mujoco_library}")
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "${RMCS_MUJOCO_ROOT}"
        INSTALL_RPATH "${RMCS_MUJOCO_ROOT}")

    install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)
if(RMCS_ENABLE_MUJOCO_SIM)
    pluginlib_export_plugin_description_file(rmcs_executor plugins_mujoco.xml)
endif()

add_executable(wheel_leg_recording_preview tools/wheel_leg_recording_preview.cpp)
target_include_directories(wheel_leg_recording_preview PRIVATE ${PROJECT_SOURCE_DIR}/src)
install(TARGETS wheel_leg_recording_preview DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
