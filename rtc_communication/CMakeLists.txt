cmake_minimum_required(VERSION 3.16)
project(rtc_communication VERSION 5.17.0 LANGUAGES CXX)

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

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_udp_loopback test/test_udp_loopback.cpp)
  if(TARGET test_udp_loopback)
    target_include_directories(test_udp_loopback PRIVATE include ${rtc_base_INCLUDE_DIRS})
  endif()

  ament_add_gtest(test_transceiver test/test_transceiver.cpp)
  if(TARGET test_transceiver)
    target_include_directories(test_transceiver PRIVATE include ${rtc_base_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_transceiver pthread)
  endif()
endif()

ament_package()
