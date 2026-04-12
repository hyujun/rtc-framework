cmake_minimum_required(VERSION 3.16)
project(rtc_base VERSION 5.17.0 LANGUAGES CXX)

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

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_session_dir test/test_session_dir.cpp)
  if(TARGET test_session_dir)
    target_include_directories(test_session_dir PRIVATE include)
  endif()

  ament_add_gtest(test_seqlock test/test_seqlock.cpp)
  if(TARGET test_seqlock)
    target_include_directories(test_seqlock PRIVATE include)
    target_link_libraries(test_seqlock pthread)
  endif()

  ament_add_gtest(test_spsc_buffer test/test_spsc_buffer.cpp)
  if(TARGET test_spsc_buffer)
    target_include_directories(test_spsc_buffer PRIVATE include)
    target_link_libraries(test_spsc_buffer pthread)
  endif()

  ament_add_gtest(test_filters test/test_filters.cpp)
  if(TARGET test_filters)
    target_include_directories(test_filters PRIVATE include)
  endif()
endif()

ament_package()
