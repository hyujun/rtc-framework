cmake_minimum_required(VERSION 3.16)
project(rtc_base VERSION 5.16.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
