cmake_minimum_required(VERSION 3.16)
project(rtc_communication VERSION 5.16.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rtc_base)
ament_package()
