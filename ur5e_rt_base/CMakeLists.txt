cmake_minimum_required(VERSION 3.16)
project(ur5e_rt_base VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# Header-only package — no executables, no libraries to build.
# Downstream packages get includes via ament_export_include_directories.

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
