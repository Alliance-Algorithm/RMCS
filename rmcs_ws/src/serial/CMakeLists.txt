cmake_minimum_required(VERSION 3.8)
project(serial)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

# if(UNIX AND NOT APPLE)
# # If Linux, add rt and pthread
# set(rt_LIBRARIES rt)
# set(pthread_LIBRARIES pthread)
# catkin_package(
# LIBRARIES ${PROJECT_NAME}
# INCLUDE_DIRS include
# DEPENDS rt pthread
# )
# else()
# # Otherwise normal call
# catkin_package(
# LIBRARIES ${PROJECT_NAME}
# INCLUDE_DIRS include
# )
# endif()

# Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(APPLE)
    # If OSX
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

# Add serial library
add_library(${PROJECT_NAME} SHARED ${serial_SRCS})

if(APPLE)
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} rt pthread)
else()
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# Include headers
target_include_directories(${PROJECT_NAME} PUBLIC include)

install(
    TARGETS
    ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY include/
    DESTINATION include
)

# Export libraries
ament_export_libraries(${PROJECT_NAME})

# Export headers
ament_export_include_directories(include)

ament_package()