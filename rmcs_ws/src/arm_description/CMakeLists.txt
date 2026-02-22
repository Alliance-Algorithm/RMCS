cmake_minimum_required(VERSION 3.10)

project(arm_description)
find_package(ament_cmake REQUIRED)
ament_package()


install(DIRECTORY urdf
    meshes launch
    DESTINATION share/${PROJECT_NAME})

