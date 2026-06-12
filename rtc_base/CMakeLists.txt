cmake_minimum_required(VERSION 3.16)
project(rtc_base VERSION 5.17.0 LANGUAGES CXX)

# ── C++20 requirement (concepts, jthread, numbers, etc.) ──────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

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

  ament_add_gtest(test_types test/test_types.cpp)
  if(TARGET test_types)
    target_include_directories(test_types PRIVATE include)
  endif()

  ament_add_gtest(test_filters test/test_filters.cpp)
  if(TARGET test_filters)
    target_include_directories(test_filters PRIVATE include)
  endif()

  ament_add_gtest(test_mpc_thread_config test/test_mpc_thread_config.cpp)
  if(TARGET test_mpc_thread_config)
    target_include_directories(test_mpc_thread_config PRIVATE include)
  endif()

  ament_add_gtest(test_cpu_topology test/test_cpu_topology.cpp)
  if(TARGET test_cpu_topology)
    target_include_directories(test_cpu_topology PRIVATE include)
  endif()

  ament_add_gtest(test_thread_timing test/test_thread_timing.cpp)
  if(TARGET test_thread_timing)
    target_include_directories(test_thread_timing PRIVATE include)
  endif()

  ament_add_gtest(test_thread_csv test/test_thread_csv.cpp)
  if(TARGET test_thread_csv)
    target_include_directories(test_thread_csv PRIVATE include)
  endif()

  ament_add_gtest(test_periodic_rt_thread test/test_periodic_rt_thread.cpp)
  if(TARGET test_periodic_rt_thread)
    target_include_directories(test_periodic_rt_thread PRIVATE include)
    target_link_libraries(test_periodic_rt_thread pthread)
  endif()

  ament_add_gtest(test_rt_utils test/test_rt_utils.cpp)
  if(TARGET test_rt_utils)
    target_include_directories(test_rt_utils PRIVATE include)
  endif()

  ament_add_gtest(test_drift_filters test/test_drift_filters.cpp)
  if(TARGET test_drift_filters)
    target_include_directories(test_drift_filters PRIVATE include)
    target_link_libraries(test_drift_filters pthread)
  endif()

  ament_add_gtest(test_timing_profiler_base test/test_timing_profiler_base.cpp)
  if(TARGET test_timing_profiler_base)
    target_include_directories(test_timing_profiler_base PRIVATE include)
  endif()

  ament_add_gtest(test_spsc_queue test/test_spsc_queue.cpp)
  if(TARGET test_spsc_queue)
    target_include_directories(test_spsc_queue PRIVATE include)
    target_link_libraries(test_spsc_queue pthread)
  endif()
endif()

ament_package()
