cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(librmcs-sdk VERSION 3 LANGUAGES C CXX)

if(NOT DEFINED LIBRMCS_PROJECT_VERSION OR LIBRMCS_PROJECT_VERSION STREQUAL "")
    execute_process(
        COMMAND ".scripts/generate_version"
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
        OUTPUT_VARIABLE LIBRMCS_PROJECT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        COMMAND_ERROR_IS_FATAL ANY
    )
endif()
message(STATUS "Librmcs project version: ${LIBRMCS_PROJECT_VERSION}")

# Set C++ standard to C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set C standard to C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Disable GNU extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add compiler options based on compiler
if(MSVC)
    add_compile_options(/W4 /Zc:preprocessor)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
    # GCC/Clang
    add_compile_options(
        -g -Wall -Wextra -Wpedantic -fvisibility=hidden
        $<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>
    )
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

# Get project sources
file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

option(BUILD_STATIC_LIBRMCS "Build static librmcs" ON)
if(BUILD_STATIC_LIBRMCS)
    add_library(
        ${PROJECT_NAME} STATIC
        ${PROJECT_SOURCE}
    )
    target_compile_definitions(${PROJECT_NAME} PUBLIC STATIC_LINKING_LIBRMCS)
    message(STATUS "Building static librmcs")
else()
    add_library(
        ${PROJECT_NAME} SHARED
        ${PROJECT_SOURCE}
    )
    message(STATUS "Building shared librmcs")
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE LIBRMCS_PROJECT_VERSION_STRING="${LIBRMCS_PROJECT_VERSION}")
target_compile_definitions(${PROJECT_NAME} PRIVATE BUILDING_LIBRMCS)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../core/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../host/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Force optimization in debug mode
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:/O2>)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-O3>)
endif()

if(WIN32)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME} PUBLIC "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUSB_LIBRARIES})
elseif(UNIX)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUSB_LIBRARIES})
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
endif()

# Enable LTO optimization
set_target_properties(${PROJECT_NAME} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION TRUE
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY ../core/include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

# include(CTest)
# if(BUILD_TESTING)
#     include(FetchContent)
#     set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#     FetchContent_Declare(
#         googletest
#         URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
#         DOWNLOAD_EXTRACT_TIMESTAMP TRUE
#     )
#     FetchContent_MakeAvailable(googletest)

#     file(GLOB_RECURSE LIBRMCS_TEST_SOURCES CONFIGURE_DEPENDS
#         ${PROJECT_SOURCE_DIR}/tests/*.cpp
#     )

#     add_executable(librmcs_tests
#         ${LIBRMCS_TEST_SOURCES}
#     )
#     target_link_libraries(librmcs_tests PRIVATE gtest_main ${PROJECT_NAME})

#     add_test(NAME librmcs_tests COMMAND librmcs_tests)
# endif()

if(UNIX AND NOT APPLE)
    if(NOT DEFINED LIBRMCS_DEBIAN_VERSION OR LIBRMCS_DEBIAN_VERSION STREQUAL "")
        execute_process(
            COMMAND ".scripts/generate_version" "--format" "debian"
            WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
            OUTPUT_VARIABLE LIBRMCS_DEBIAN_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ERROR_IS_FATAL ANY
        )
    endif()
    message(STATUS "Librmcs Debian package version: ${LIBRMCS_DEBIAN_VERSION}")

    set(CPACK_GENERATOR "DEB")

    set(ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
    if(ARCHITECTURE STREQUAL "x86_64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    elseif(ARCHITECTURE STREQUAL "aarch64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
    else()
        message(FATAL_ERROR "Unsupported architecture: ${ARCHITECTURE}.")
    endif()

    set(CPACK_DEBIAN_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_DEBIAN_PACKAGE_VERSION "${LIBRMCS_DEBIAN_VERSION}")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Zihan Qin <zihanqin2048@gmail.com>")
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "A portable library implements the core functionality of RMCS")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/Alliance-Algorithm/librmcs")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0-dev")
    set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
    set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")

    string(TOLOWER "${CMAKE_BUILD_TYPE}" PACKAGE_BUILD_MODE)
    set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${LIBRMCS_PROJECT_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}-${PACKAGE_BUILD_MODE}")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE")

    include(CPack)
endif()
