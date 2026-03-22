cmake_minimum_required(VERSION 3.16)
project(rtc_base VERSION 5.16.0 LANGUAGES CXX)

# ── C++20 requirement (concepts, jthread, numbers, etc.) ──────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Strict compiler warnings ─────────────────────────────────────────────────
add_compile_options(
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
