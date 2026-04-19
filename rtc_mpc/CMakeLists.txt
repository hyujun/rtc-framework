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

# ── MPC deps (Phase 0 dual-install workarounds — see
#    docs/mpc_implementation_progress.md §Phase 0 "CMake Constraints") ────────
# hpp-fcl ABI: force ROS Jazzy's hpp-fcl *before* pinocchio's transitive
# find_dependency, otherwise /usr/local's RowMajor-signature hpp-fcl wins and
# breaks pinocchio_parsers linking.
if(EXISTS /opt/ros/$ENV{ROS_DISTRO}/lib/x86_64-linux-gnu/cmake/hpp-fcl)
  set(hpp-fcl_DIR
    /opt/ros/$ENV{ROS_DISTRO}/lib/x86_64-linux-gnu/cmake/hpp-fcl
    CACHE PATH "ROS hpp-fcl (avoid /usr/local ABI break)" FORCE)
endif()
# fmt: apt libfmt-dev (9.1.0) is preferred by CMake's multi-arch search,
# but Aligator was built against source-installed fmt 11.1.4. Hint /usr/local.
find_package(fmt 10 REQUIRED HINTS /usr/local/lib/cmake/fmt)
find_package(pinocchio REQUIRED)
# aligator: source-installed at /usr/local (Phase 0). Not a rosdep — do NOT add
# to package.xml. CMake config lives at /usr/local/lib/cmake/aligator.
find_package(aligator REQUIRED)

# ── Library target (STATIC) ──────────────────────────────────────────────────
add_library(rtc_mpc STATIC
  src/interpolation/trajectory_interpolator.cpp
  src/feedback/riccati_feedback.cpp
  src/manager/mpc_solution_manager.cpp
  src/thread/mpc_thread.cpp
  src/thread/mock_mpc_thread.cpp
  src/model/robot_model_handler.cpp
  src/phase/phase_cost_config.cpp
  src/ocp/cost_factory.cpp
  src/ocp/light_contact_ocp.cpp
  src/ocp/contact_rich_ocp.cpp
  src/handler/internal/mpc_handler_core.cpp
  src/handler/light_contact_mpc.cpp
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

  ament_add_gtest(test_light_contact_ocp test/test_light_contact_ocp.cpp)
  if(TARGET test_light_contact_ocp)
    target_link_libraries(test_light_contact_ocp rtc_mpc)
  endif()

  ament_add_gtest(test_contact_rich_ocp test/test_contact_rich_ocp.cpp)
  if(TARGET test_contact_rich_ocp)
    target_include_directories(test_contact_rich_ocp PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_contact_rich_ocp rtc_mpc)
  endif()

  ament_add_gtest(test_light_contact_mpc test/test_light_contact_mpc.cpp)
  if(TARGET test_light_contact_mpc)
    target_link_libraries(test_light_contact_mpc rtc_mpc)
  endif()

  ament_add_gtest(test_contact_rich_mpc test/test_contact_rich_mpc.cpp)
  if(TARGET test_contact_rich_mpc)
    target_link_libraries(test_contact_rich_mpc rtc_mpc)
  endif()

  ament_add_gtest(test_mpc_factory test/test_mpc_factory.cpp)
  if(TARGET test_mpc_factory)
    target_link_libraries(test_mpc_factory rtc_mpc)
  endif()
endif()

ament_package()
