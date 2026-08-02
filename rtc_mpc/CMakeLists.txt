cmake_minimum_required(VERSION 3.16)
project(rtc_mpc VERSION 0.1.0 LANGUAGES CXX)

# ── C++20 requirement ────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Strict warning flags (CLAUDE.md compiler policy) ─────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wshadow -Wconversion -Wsign-conversion
    -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
  )
endif()

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
# Directory-scoped: instruments the STATIC lib + all tests when ON.
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── MPC deps (isolated to deps/install/ via scripts/setup_env.sh) ────────────
# pinocchio 4.0 은 충돌 라이브러리로 coal 을 사용 (hpp-fcl 폐기). coal 은 ROS
# distribution 판만 존재하고 pinocchioConfig 가 find_dependency(coal) 로 강제하므로
# 별도 *_DIR 강제 불필요 (구 hpp-fcl_DIR ABI-pin 코드 제거).
# fmt: deps/install (v11.1.4) 를 CMAKE_PREFIX_PATH 에서 찾음. v10 이상 요구.
# apt libfmt-dev (9.1.0) 가 ROS transitive 로 들어와도 HINT 는 필요 없음 —
# scripts/setup_env.sh 가 CMAKE_PREFIX_PATH 앞에 deps/install 을 둠.
find_package(fmt 10 REQUIRED)
find_package(pinocchio REQUIRED)
# aligator: deps/install (v0.19.0). CMAKE_PREFIX_PATH 로 찾음.
find_package(aligator REQUIRED)

# ── Library target (STATIC) ──────────────────────────────────────────────────
add_library(rtc_mpc STATIC
  src/interpolation/trajectory_interpolator.cpp
  src/feedback/riccati_feedback.cpp
  src/manager/mpc_solution_manager.cpp
  src/thread/mpc_thread.cpp
  src/thread/mock_mpc_thread.cpp
  src/thread/handler_mpc_thread.cpp
  src/model/robot_model_handler.cpp
  src/phase/phase_cost_config.cpp
  src/ocp/cost_factory.cpp
  src/ocp/contact_light_ocp.cpp
  src/ocp/contact_rich_ocp.cpp
  src/handler/internal/mpc_handler_core.cpp
  src/handler/contact_light_mpc.cpp
  src/handler/contact_rich_mpc.cpp
  src/handler/mpc_factory.cpp
)

target_include_directories(rtc_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Private build-tree include so TUs under src/ocp/ can #include
# "ocp/internal/..." shared helpers. Not installed or exported.
target_include_directories(rtc_mpc PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_include_directories(rtc_mpc SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(rtc_mpc PUBLIC
  yaml-cpp
  pinocchio::pinocchio
  fmt::fmt
  aligator::aligator
)

ament_target_dependencies(rtc_mpc PUBLIC
  rtc_base
)

# ── RT tracing (opt-in) ──────────────────────────────────────────────────────
# When rtc_base was built with -DRTC_ENABLE_TRACING=ON it exports the
# `rtc_tracing` target (lttng include path + probe .so). Linking it activates
# the RTC_TRACE_SCOPE spans in the MPC thread loop (mpc_tick → sub-phases).
# Default OFF → no-op, no lttng dependency. PRIVATE (this target uses keyword
# signatures): spans live only in this lib's TUs and the define below never
# leaks to consumers. See rtc_base/CMakeLists.txt.
option(RTC_ENABLE_TRACING "Compile rtc:* LTTng UST tracepoints (RT tracing)" OFF)
if(RTC_ENABLE_TRACING)
  target_link_libraries(rtc_mpc PRIVATE rtc_base::rtc_tracing)
  # Define set here (PRIVATE), not inherited from rtc_tracing — see the
  # interface-leak note in rtc_base/CMakeLists.txt. This keeps the opt-in local
  # so an ON rtc_base never silently instruments an OFF consumer.
  target_compile_definitions(rtc_mpc PRIVATE RTC_TRACING_ENABLED)
endif()

# ── Install & Export ─────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(TARGETS rtc_mpc EXPORT rtc_mpcTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT rtc_mpcTargets
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE rtc_mpc::
)

ament_export_targets(rtc_mpcTargets)
ament_export_include_directories(include)
ament_export_dependencies(rtc_base Eigen3 yaml-cpp pinocchio fmt aligator)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Path to the Panda URDF used by integration / OCP tests as a generic 7-DOF
  # fixture. Default resolves to the repo-vendored copy under robot_descriptions
  # (checked in, so present at build time both locally and in CI — unlike the old
  # deps/src or /usr/local paths that the deps artifact does not ship).
  # pinocchio::buildModel parses kinematics/dynamics only, so the package://
  # meshes are not needed. Override-able via -D or env.
  get_filename_component(_rtc_panda_default
      "${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots/panda/urdf/panda.urdf"
      ABSOLUTE)
  set(RTC_PANDA_URDF_PATH "${_rtc_panda_default}"
      CACHE FILEPATH "Path to panda.urdf used by rtc_mpc tests")
  if(DEFINED ENV{RTC_PANDA_URDF_PATH})
    set(RTC_PANDA_URDF_PATH "$ENV{RTC_PANDA_URDF_PATH}" CACHE FILEPATH
        "Path to panda.urdf used by rtc_mpc tests" FORCE)
  endif()

  ament_add_gtest(test_scaffold test/test_scaffold.cpp)
  if(TARGET test_scaffold)
    target_link_libraries(test_scaffold rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_types test/test_mpc_types.cpp)
  if(TARGET test_mpc_types)
    target_link_libraries(test_mpc_types rtc_mpc)
  endif()

  ament_add_gtest(test_triple_buffer test/test_triple_buffer.cpp)
  if(TARGET test_triple_buffer)
    target_link_libraries(test_triple_buffer rtc_mpc)
  endif()

  ament_add_gtest(test_trajectory_interpolator
    test/test_trajectory_interpolator.cpp)
  if(TARGET test_trajectory_interpolator)
    target_link_libraries(test_trajectory_interpolator rtc_mpc)
  endif()

  ament_add_gtest(test_riccati_feedback test/test_riccati_feedback.cpp)
  if(TARGET test_riccati_feedback)
    target_link_libraries(test_riccati_feedback rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_solution_manager
    test/test_mpc_solution_manager.cpp)
  if(TARGET test_mpc_solution_manager)
    target_link_libraries(test_mpc_solution_manager rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_timing_logger
    test/test_mpc_timing_logger.cpp)
  if(TARGET test_mpc_timing_logger)
    target_link_libraries(test_mpc_timing_logger rtc_mpc)
  endif()

  # rtc_base/testing/wait_until.hpp: the sleep-poll wait these thread tests use,
  # a test-only, never-installed header (#356 — promoted out of this package's
  # own test/wait_until.hpp). rtc_base owns the path and fails loudly if its
  # source tree is not reachable — see rtc_base/cmake/rtc_base-test-include.cmake.in.
  rtc_base_test_include_dir(rtc_base_test_include)

  ament_add_gtest(test_mpc_thread_skeleton
    test/test_mpc_thread_skeleton.cpp)
  if(TARGET test_mpc_thread_skeleton)
    target_include_directories(test_mpc_thread_skeleton PRIVATE
      ${rtc_base_test_include})
    target_link_libraries(test_mpc_thread_skeleton rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_thread_mock test/test_mpc_thread_mock.cpp)
  if(TARGET test_mpc_thread_mock)
    target_link_libraries(test_mpc_thread_mock rtc_mpc)
  endif()

  ament_add_gtest(test_robot_model_handler
    test/test_robot_model_handler.cpp)
  if(TARGET test_robot_model_handler)
    target_link_libraries(test_robot_model_handler rtc_mpc)
  endif()

  ament_add_gtest(test_phase_cost_config
    test/test_phase_cost_config.cpp)
  if(TARGET test_phase_cost_config)
    target_link_libraries(test_phase_cost_config rtc_mpc)
  endif()

  ament_add_gtest(test_cost_factory test/test_cost_factory.cpp)
  if(TARGET test_cost_factory)
    target_link_libraries(test_cost_factory rtc_mpc)
  endif()

  ament_add_gtest(test_contact_light_ocp test/test_contact_light_ocp.cpp)
  if(TARGET test_contact_light_ocp)
    target_link_libraries(test_contact_light_ocp rtc_mpc)
  endif()

  ament_add_gtest(test_contact_rich_ocp test/test_contact_rich_ocp.cpp)
  if(TARGET test_contact_rich_ocp)
    target_include_directories(test_contact_rich_ocp PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_contact_rich_ocp rtc_mpc)
  endif()

  ament_add_gtest(test_contact_light_mpc test/test_contact_light_mpc.cpp)
  if(TARGET test_contact_light_mpc)
    target_link_libraries(test_contact_light_mpc rtc_mpc)
  endif()

  ament_add_gtest(test_contact_rich_mpc test/test_contact_rich_mpc.cpp)
  if(TARGET test_contact_rich_mpc)
    target_link_libraries(test_contact_rich_mpc rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_factory test/test_mpc_factory.cpp)
  if(TARGET test_mpc_factory)
    target_link_libraries(test_mpc_factory rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_thread_integration
    test/test_mpc_thread_integration.cpp)
  if(TARGET test_mpc_thread_integration)
    target_include_directories(test_mpc_thread_integration PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test ${rtc_base_test_include})
    target_link_libraries(test_mpc_thread_integration rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_handler_alloc_tracer
    test/test_mpc_handler_alloc_tracer.cpp)
  if(TARGET test_mpc_handler_alloc_tracer)
    target_include_directories(test_mpc_handler_alloc_tracer PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_mpc_handler_alloc_tracer rtc_mpc)
  endif()

  # Apply PANDA_URDF_PATH macro to every test target that uses it.
  foreach(_t
      test_robot_model_handler test_phase_cost_config test_cost_factory
      test_contact_light_ocp test_contact_rich_ocp test_contact_light_mpc
      test_contact_rich_mpc test_mpc_factory test_mpc_thread_integration
      test_mpc_handler_alloc_tracer)
    if(TARGET ${_t})
      target_compile_definitions(${_t} PRIVATE
          RTC_PANDA_URDF_PATH="${RTC_PANDA_URDF_PATH}")
    endif()
  endforeach()

  # Aligator's ProxDDP solve path allocates via mimalloc but frees can route
  # through glibc (and vice-versa) across the pinocchio/aligator boundary; with
  # Pinocchio 4.0 this surfaces as `free(): invalid pointer` / `double free` in
  # the contact_rich solve tests. mimalloc must be the FIRST-resolved allocator
  # to interpose globally, which a plain link cannot guarantee. Preload it for
  # the aligator-solve tests under ctest (the sanctioned workaround — do not
  # deactivate isolation or run gtest binaries bare instead).
  find_library(RTC_MPC_MIMALLOC NAMES mimalloc
    HINTS $ENV{RTC_DEPS_PREFIX}/lib "${aligator_DIR}/../.."
    DOC "mimalloc lib for LD_PRELOAD in aligator-solve ctest runs")
  if(RTC_MPC_MIMALLOC)
    foreach(_solve_test
        test_contact_light_ocp test_contact_rich_ocp test_contact_light_mpc
        test_contact_rich_mpc test_mpc_factory test_mpc_thread_integration
        test_mpc_handler_alloc_tracer)
      if(TEST ${_solve_test})
        set_property(TEST ${_solve_test} APPEND PROPERTY
            ENVIRONMENT "LD_PRELOAD=${RTC_MPC_MIMALLOC}")
      endif()
    endforeach()
  else()
    message(WARNING
      "mimalloc not found — aligator-solve tests may double-free under Pinocchio 4.0. "
      "Set RTC_DEPS_PREFIX or build deps via repo_scripts/scripts/build_deps.sh.")
  endif()
endif()

ament_package()
