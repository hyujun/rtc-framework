cmake_minimum_required(VERSION 3.16)
project(rtc_communication VERSION 5.16.0 LANGUAGES CXX)

# ── C++20 requirement (concepts, jthread, span, stop_token) ───────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
)

find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rtc_base)
ament_package()
