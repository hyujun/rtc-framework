cmake_minimum_required(VERSION 3.16)
project(ur5e_tools)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


ament_package()
