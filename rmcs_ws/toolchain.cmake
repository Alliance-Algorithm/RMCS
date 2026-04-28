cmake_minimum_required(VERSION 3.16)

foreach(_rmcs_var IN ITEMS RMCS_TARGET_ARCH RMCS_SYSROOT RMCS_TARGET_TRIPLET)
  if((NOT DEFINED ${_rmcs_var} OR "${${_rmcs_var}}" STREQUAL "") AND DEFINED ENV{${_rmcs_var}})
    set(${_rmcs_var} "$ENV{${_rmcs_var}}")
  endif()
endforeach()
unset(_rmcs_var)

set(RMCS_TARGET_ARCH "${RMCS_TARGET_ARCH}" CACHE STRING "RMCS target architecture")
set(RMCS_SYSROOT "${RMCS_SYSROOT}" CACHE PATH "RMCS target sysroot path")
set(RMCS_TARGET_TRIPLET "${RMCS_TARGET_TRIPLET}" CACHE STRING "RMCS target compiler triplet")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
  RMCS_TARGET_ARCH
  RMCS_SYSROOT
  RMCS_TARGET_TRIPLET
)

if(NOT RMCS_TARGET_ARCH)
  message(FATAL_ERROR "RMCS_TARGET_ARCH is required. Supported values: arm64, amd64.")
endif()

if(NOT RMCS_TARGET_TRIPLET)
  if(RMCS_TARGET_ARCH STREQUAL "arm64")
    set(RMCS_TARGET_TRIPLET "aarch64-linux-gnu")
  elseif(RMCS_TARGET_ARCH STREQUAL "amd64")
    set(RMCS_TARGET_TRIPLET "x86_64-linux-gnu")
  else()
    message(FATAL_ERROR "Unsupported RMCS_TARGET_ARCH: ${RMCS_TARGET_ARCH}")
  endif()
endif()

if(NOT RMCS_SYSROOT)
  set(RMCS_SYSROOT "/opt/sysroots/${RMCS_TARGET_ARCH}")
endif()

if(NOT EXISTS "${RMCS_SYSROOT}")
  message(FATAL_ERROR "RMCS_SYSROOT does not exist: ${RMCS_SYSROOT}")
endif()

set(CMAKE_SYSTEM_NAME Linux)
if(RMCS_TARGET_ARCH STREQUAL "arm64")
  set(CMAKE_SYSTEM_PROCESSOR aarch64)
elseif(RMCS_TARGET_ARCH STREQUAL "amd64")
  set(CMAKE_SYSTEM_PROCESSOR x86_64)
else()
  message(FATAL_ERROR "Unsupported RMCS_TARGET_ARCH: ${RMCS_TARGET_ARCH}")
endif()

set(CMAKE_SYSROOT "${RMCS_SYSROOT}")
set(_rmcs_find_root_path "${RMCS_SYSROOT}")
foreach(_rmcs_prefix_env IN ITEMS AMENT_PREFIX_PATH CMAKE_PREFIX_PATH)
  if(DEFINED ENV{${_rmcs_prefix_env}} AND NOT "$ENV{${_rmcs_prefix_env}}" STREQUAL "")
    string(REPLACE ":" ";" _rmcs_env_prefixes "$ENV{${_rmcs_prefix_env}}")
    list(APPEND _rmcs_find_root_path ${_rmcs_env_prefixes})
  endif()
endforeach()
list(REMOVE_DUPLICATES _rmcs_find_root_path)
set(CMAKE_FIND_ROOT_PATH
  "${_rmcs_find_root_path}"
)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

function(rmcs_find_host_program output_var)
  set(options)
  set(oneValueArgs ENV_VAR)
  set(multiValueArgs NAMES)
  cmake_parse_arguments(RMCS_FIND_HOST_PROGRAM "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT RMCS_FIND_HOST_PROGRAM_NAMES)
    message(FATAL_ERROR "rmcs_find_host_program requires at least one candidate in NAMES.")
  endif()

  unset(_rmcs_resolved_program CACHE)
  unset(_rmcs_resolved_program)
  string(REPLACE ":" ";" _rmcs_host_path_entries "$ENV{PATH}")
  if(RMCS_FIND_HOST_PROGRAM_ENV_VAR
     AND DEFINED ENV{${RMCS_FIND_HOST_PROGRAM_ENV_VAR}}
     AND NOT "$ENV{${RMCS_FIND_HOST_PROGRAM_ENV_VAR}}" STREQUAL "")
    set(_rmcs_program_candidates "$ENV{${RMCS_FIND_HOST_PROGRAM_ENV_VAR}}")
    if(IS_ABSOLUTE "${_rmcs_program_candidates}")
      if(NOT EXISTS "${_rmcs_program_candidates}" OR IS_DIRECTORY "${_rmcs_program_candidates}")
        message(FATAL_ERROR
          "${RMCS_FIND_HOST_PROGRAM_ENV_VAR} points to a missing executable: ${_rmcs_program_candidates}"
        )
      endif()
      set(_rmcs_resolved_program "${_rmcs_program_candidates}")
    else()
      find_program(_rmcs_resolved_program
        NAMES "${_rmcs_program_candidates}"
        PATHS ${_rmcs_host_path_entries}
        NO_DEFAULT_PATH
      )
    endif()
  else()
    set(_rmcs_program_candidates ${RMCS_FIND_HOST_PROGRAM_NAMES})
    find_program(_rmcs_resolved_program
      NAMES ${_rmcs_program_candidates}
      PATHS ${_rmcs_host_path_entries}
      NO_DEFAULT_PATH
    )
  endif()

  if(NOT _rmcs_resolved_program)
    list(JOIN _rmcs_program_candidates ", " _rmcs_program_candidates_str)
    message(FATAL_ERROR "Cannot resolve host program. Tried: ${_rmcs_program_candidates_str}")
  endif()

  set(${output_var} "${_rmcs_resolved_program}" PARENT_SCOPE)
endfunction()

rmcs_find_host_program(RMCS_C_COMPILER ENV_VAR CC NAMES "${RMCS_TARGET_TRIPLET}-gcc")
rmcs_find_host_program(RMCS_CXX_COMPILER ENV_VAR CXX NAMES "${RMCS_TARGET_TRIPLET}-g++")
rmcs_find_host_program(RMCS_PYTHON_EXECUTABLE NAMES python3)
rmcs_find_host_program(RMCS_PKG_CONFIG_EXECUTABLE NAMES pkg-config pkgconf)

if(NOT RMCS_C_COMPILER)
  message(FATAL_ERROR "Cannot find C compiler: ${RMCS_TARGET_TRIPLET}-gcc")
endif()
if(NOT RMCS_CXX_COMPILER)
  message(FATAL_ERROR "Cannot find CXX compiler: ${RMCS_TARGET_TRIPLET}-g++")
endif()

set(CMAKE_C_COMPILER "${RMCS_C_COMPILER}")
set(CMAKE_CXX_COMPILER "${RMCS_CXX_COMPILER}")
set(Python3_EXECUTABLE "${RMCS_PYTHON_EXECUTABLE}" CACHE FILEPATH "Host Python interpreter" FORCE)
set(AMENT_PYTHON_EXECUTABLE "${RMCS_PYTHON_EXECUTABLE}" CACHE FILEPATH "Host Python interpreter for ament" FORCE)
set(PKG_CONFIG_EXECUTABLE "${RMCS_PKG_CONFIG_EXECUTABLE}" CACHE FILEPATH "Host pkg-config executable" FORCE)

if(CMAKE_GENERATOR MATCHES "Makefiles")
  rmcs_find_host_program(RMCS_MAKE_PROGRAM NAMES gmake make)
  set(CMAKE_MAKE_PROGRAM "${RMCS_MAKE_PROGRAM}" CACHE FILEPATH "Host make program" FORCE)
endif()

set(CMAKE_PREFIX_PATH
  "/opt/ros/jazzy"
  "/usr"
  CACHE STRING "Target-side prefix path for cross-compilation" FORCE
)

set(ENV{PKG_CONFIG_SYSROOT_DIR} "${RMCS_SYSROOT}")
set(_rmcs_pkg_config_libdirs
  "${RMCS_SYSROOT}/usr/lib/${RMCS_TARGET_TRIPLET}/pkgconfig"
  "${RMCS_SYSROOT}/usr/lib/pkgconfig"
  "${RMCS_SYSROOT}/usr/share/pkgconfig"
  "${RMCS_SYSROOT}/lib/${RMCS_TARGET_TRIPLET}/pkgconfig"
  "${RMCS_SYSROOT}/lib/pkgconfig"
  "${RMCS_SYSROOT}/opt/ros/jazzy/lib/${RMCS_TARGET_TRIPLET}/pkgconfig"
  "${RMCS_SYSROOT}/opt/ros/jazzy/lib/pkgconfig"
  "${RMCS_SYSROOT}/opt/ros/jazzy/share/pkgconfig"
)
list(JOIN _rmcs_pkg_config_libdirs ":" _rmcs_pkg_config_libdir)
set(ENV{PKG_CONFIG_LIBDIR} "${_rmcs_pkg_config_libdir}")
