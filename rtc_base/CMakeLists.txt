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
  -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
)

find_package(ament_cmake REQUIRED)

# Only include/ ships. The test-only headers (rtc_base/testing/) live under
# test/include, NOT here — nothing there may reach the runtime install tree,
# and ament's symlink install ignores install(PATTERN EXCLUDE), so placing them
# under include/ would ship them:
#   - no_malloc_scope.hpp  — pulls in Eigen, which rtc_base deliberately does
#     not export (the runtime library stays Eigen-free, see the BUILD_TESTING
#     block), so it must never reach rtc_base's public include surface.
#   - wait_until.hpp       — sleep-poll wait shared by sibling test suites
#     (#356); test scaffolding with no runtime consumer.
# Sibling packages consume both via a source-tree include path
# (../rtc_base/test/include), the same layout the rtc_urdf_bridge/test headers
# use (see rtc_controllers/CMakeLists.txt).
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

# ── RT tracing (LTTng UST) — opt-in, compile-time gated ──────────────────────
# Default OFF: RTC_TRACE_SCOPE (rtc_base/tracing/trace_scope.hpp) expands to a
# no-op, no lttng-ust header/link is pulled in, and the RT hot path pays zero
# cost. Enable a tracing build with:
#   colcon build --cmake-args -DRTC_ENABLE_TRACING=ON
# which compiles the rtc:* UST probes into librtc_tracing. Consumers activate
# their RTC_TRACE_SCOPE call sites by BOTH linking the exported target
# `rtc_base::rtc_tracing` (lttng include path + probe .so) AND setting the
# RTC_TRACING_ENABLED define themselves (PRIVATE) in their own
# if(RTC_ENABLE_TRACING) block. The define is deliberately NOT on the exported
# target's interface: ament_target_dependencies(<pkg> rtc_base) propagates an
# exported target's INTERFACE_COMPILE_DEFINITIONS, so a PUBLIC define here would
# silently instrument EVERY rtc_base consumer whenever rtc_base itself is built
# ON — even ones built with RTC_ENABLE_TRACING=OFF — breaking the per-target
# opt-in and the zero-cost guarantee. Keeping it per-consumer-PRIVATE makes the
# opt-in strictly local.
# liblttng-ust-dev is provided by `install.sh --tracing` (optional apt dep,
# same contract as MuJoCo/aligator — intentionally not a package.xml <depend>).
option(RTC_ENABLE_TRACING "Compile rtc:* LTTng UST tracepoints (RT tracing)" OFF)
if(RTC_ENABLE_TRACING)
  find_package(LTTngUST REQUIRED)
  add_library(rtc_tracing SHARED src/tracing/rtc_tracepoints.cpp)
  target_include_directories(rtc_tracing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(rtc_tracing PUBLIC LTTng::UST)
  # RTC_TRACING_ENABLED for the probe TU itself (harmless — it does not gate on
  # it) but NOT exported: see the interface-leak note above. PRIVATE only.
  target_compile_definitions(rtc_tracing PRIVATE RTC_TRACING_ENABLED)
  # The probe TU is lttng-generated boilerplate; silence this repo's strict
  # -Wconversion/-Wpedantic wall on it only (never on call-site code).
  target_compile_options(rtc_tracing PRIVATE -w)
  install(TARGETS rtc_tracing EXPORT export_rtc_base
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
  # Consumers link the imported target `rtc_base::rtc_tracing` (full .so path +
  # PUBLIC RTC_TRACING_ENABLED). Deliberately NOT ament_export_libraries() — that
  # would leak a bare `-lrtc_tracing` into every rtc_base consumer via
  # ament_target_dependencies, unresolvable without a link dir.
  ament_export_targets(export_rtc_base)
  ament_export_dependencies(LTTngUST)
  message(STATUS "rtc_base: RT tracing ENABLED (rtc:* UST probes -> librtc_tracing)")
endif()

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Eigen is a TEST-only dependency: the zero-allocation gate (testing/) is the
  # sole Eigen user in rtc_base. The runtime library stays Eigen-free.
  find_package(Eigen3 REQUIRED)

  ament_add_gtest(test_no_malloc_scope test/test_no_malloc_scope.cpp)
  if(TARGET test_no_malloc_scope)
    # test/include holds the never-installed testing/ gate header (Eigen-coupled,
    # kept off the public include surface — see the install() note above).
    target_include_directories(test_no_malloc_scope PRIVATE include test/include)
    target_link_libraries(test_no_malloc_scope Eigen3::Eigen)
  endif()

  # Self-test for the shared sleep-poll wait, the sibling of the gate header
  # above under test/include. No Eigen, so it needs no extra link.
  ament_add_gtest(test_wait_until test/test_wait_until.cpp)
  if(TARGET test_wait_until)
    target_include_directories(test_wait_until PRIVATE include test/include)
  endif()

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
    # test/include also holds testing/wait_until.hpp (never-installed, shared
    # with rtc_mpc / rtc_communication / ur5e_bt_coordinator — see #356).
    target_include_directories(test_periodic_rt_thread PRIVATE include test/include)
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
