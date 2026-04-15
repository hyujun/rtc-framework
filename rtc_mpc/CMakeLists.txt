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
  )
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Library target (STATIC) ──────────────────────────────────────────────────
add_library(rtc_mpc STATIC
  src/interpolation/trajectory_interpolator.cpp
  src/feedback/riccati_feedback.cpp
  src/manager/mpc_solution_manager.cpp
  src/thread/mpc_thread.cpp
  src/thread/mock_mpc_thread.cpp
)

target_include_directories(rtc_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(rtc_mpc SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(rtc_mpc PUBLIC
  yaml-cpp
)

ament_target_dependencies(rtc_mpc PUBLIC
  rtc_base
)

# ── Install & Export ─────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)

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
ament_export_dependencies(rtc_base Eigen3 yaml-cpp)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

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

  ament_add_gtest(test_mpc_thread_skeleton
    test/test_mpc_thread_skeleton.cpp)
  if(TARGET test_mpc_thread_skeleton)
    target_link_libraries(test_mpc_thread_skeleton rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_thread_mock test/test_mpc_thread_mock.cpp)
  if(TARGET test_mpc_thread_mock)
    target_link_libraries(test_mpc_thread_mock rtc_mpc)
  endif()
endif()

ament_package()
