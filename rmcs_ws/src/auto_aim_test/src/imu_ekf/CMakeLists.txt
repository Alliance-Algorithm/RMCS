cmake_minimum_required(VERSION 3.20)

project(imu_ekf_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(imu_ekf_demo main.cpp)
target_link_libraries(imu_ekf_demo PRIVATE Eigen3::Eigen)
