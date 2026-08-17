cmake_minimum_required(VERSION 3.16)
project(rtc_urdf_bridge VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
  )
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2 QUIET)
if(NOT tinyxml2_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(tinyxml2 REQUIRED IMPORTED_TARGET tinyxml2)
  add_library(tinyxml2::tinyxml2 ALIAS PkgConfig::tinyxml2)
endif()

# ── Library ───────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/urdf_analyzer.cpp
  src/kinematic_chain_extractor.cpp
  src/pinocchio_model_builder.cpp
  src/inertial_validation.cpp
  src/rt_model_handle.cpp
  src/xacro_processor.cpp
  # Unified kin&dyn 솔버 캐시 (rtc_tsid 에서 이관 — #unified-kindyn Phase 1)
  src/pinocchio_cache.cpp
  # Extended-URDF closed-chain (sidecar) — 로드 타임 계층
  src/closure_yaml_loader.cpp
  src/constraint_builder.cpp
  src/loop_verification.cpp
  src/loop_projection.cpp
  src/closed_chain_model.cpp
  src/closed_chain_handle.cpp
  src/rt_closed_chain_handle.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# sensor_msgs is intentionally NOT a core-lib dependency: only the off-RT
# closure visualization node (closure_state_publisher, an executable) touches
# JointState. Keeping it out of the shared lib avoids forcing sensor_msgs onto
# RT-only consumers that just need the model-building API.
ament_target_dependencies(${PROJECT_NAME} rclcpp pinocchio)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Eigen3::Eigen
  yaml-cpp
  tinyxml2::tinyxml2
)

# ── Examples ──────────────────────────────────────────────────────────────────
add_executable(example_basic_usage examples/example_basic_usage.cpp)
target_link_libraries(example_basic_usage ${PROJECT_NAME})

add_executable(example_submodel examples/example_submodel.cpp)
target_link_libraries(example_submodel ${PROJECT_NAME})

add_executable(example_tree_model examples/example_tree_model.cpp)
target_link_libraries(example_tree_model ${PROJECT_NAME})

add_executable(example_rt_integration examples/example_rt_integration.cpp)
target_link_libraries(example_rt_integration ${PROJECT_NAME})

# ── Nodes ─────────────────────────────────────────────────────────────────────
# The viz node (JointState I/O) lives in the executable, not the core lib, so
# sensor_msgs stays off RT-only consumers of ${PROJECT_NAME}.
add_executable(closure_state_publisher
  src/closure_state_publisher_main.cpp
  src/closure_state_publisher.cpp
)
ament_target_dependencies(closure_state_publisher sensor_msgs)
target_link_libraries(closure_state_publisher ${PROJECT_NAME})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS
    example_basic_usage
    example_submodel
    example_tree_model
    example_rt_integration
    closure_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# sensor_msgs is an executable-only dep (see closure_state_publisher target) —
# not exported, so downstream consumers of the core lib don't inherit it.
ament_export_dependencies(rclcpp pinocchio Eigen3 yaml-cpp tinyxml2)
ament_export_include_directories(include)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_urdf_analyzer test/test_urdf_analyzer.cpp)
  target_link_libraries(test_urdf_analyzer ${PROJECT_NAME})

  ament_add_gtest(test_joint_classification test/test_joint_classification.cpp)
  target_link_libraries(test_joint_classification ${PROJECT_NAME})

  ament_add_gtest(test_chain_extractor test/test_chain_extractor.cpp)
  target_link_libraries(test_chain_extractor ${PROJECT_NAME})

  ament_add_gtest(test_model_builder test/test_model_builder.cpp)
  target_link_libraries(test_model_builder ${PROJECT_NAME})

  ament_add_gtest(test_load_model_config test/test_load_model_config.cpp)
  target_link_libraries(test_load_model_config ${PROJECT_NAME})

  ament_add_gtest(test_rt_model_handle test/test_rt_model_handle.cpp)
  target_link_libraries(test_rt_model_handle ${PROJECT_NAME})

  ament_add_gtest(test_payload_regressor test/test_payload_regressor.cpp)
  target_link_libraries(test_payload_regressor ${PROJECT_NAME})

  # #316 V9: GetFrameJacobian 행/열 순서를 중심차분 oracle 로 고정. 별도 TU 인
  # 이유는 test_rt_model_handle 의 자기일치 대조와 판별 대상이 다르기 때문이다
  # (그쪽은 추출기가 행을 뒤집어도 green 으로 남는다).
  ament_add_gtest(test_frame_jacobian_fd_oracle test/test_frame_jacobian_fd_oracle.cpp)
  target_link_libraries(test_frame_jacobian_fd_oracle ${PROJECT_NAME})

  ament_add_gtest(test_inertial_validation test/test_inertial_validation.cpp)
  target_link_libraries(test_inertial_validation ${PROJECT_NAME})

  # 실모델 게이트 — robot_descriptions 를 ament 런타임 lookup 으로만 참조한다
  # (ARCH-5: build-time 의존 아님, test_depend + share dir 조회).
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_real_model_inertial_gate test/test_real_model_inertial_gate.cpp)
  target_link_libraries(test_real_model_inertial_gate ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)

  ament_add_gtest(test_xacro_processor test/test_xacro_processor.cpp)
  target_link_libraries(test_xacro_processor ${PROJECT_NAME})

  # ── Extended-URDF closed-chain 테스트 ──────────────────────────────────────
  ament_add_gtest(test_closure_yaml_loader test/test_closure_yaml_loader.cpp)
  target_link_libraries(test_closure_yaml_loader ${PROJECT_NAME})

  ament_add_gtest(test_constraint_builder test/test_constraint_builder.cpp)
  target_link_libraries(test_constraint_builder ${PROJECT_NAME})

  ament_add_gtest(test_loop_closed_chain test/test_loop_closed_chain.cpp)
  target_link_libraries(test_loop_closed_chain ${PROJECT_NAME})

  ament_add_gtest(test_pinocchio_builder_closure test/test_pinocchio_builder_closure.cpp)
  target_link_libraries(test_pinocchio_builder_closure ${PROJECT_NAME})

  ament_add_gtest(test_loop_projection_passive test/test_loop_projection_passive.cpp)
  target_link_libraries(test_loop_projection_passive ${PROJECT_NAME})

  ament_add_gtest(test_closed_chain_handle test/test_closed_chain_handle.cpp)
  target_link_libraries(test_closed_chain_handle ${PROJECT_NAME})

  ament_add_gtest(test_rt_closed_chain_handle test/test_rt_closed_chain_handle.cpp)
  target_link_libraries(test_rt_closed_chain_handle ${PROJECT_NAME})

  # RT-1 센서: Update() 힙 할당 0. alloc_counter.hpp 가 전역 new/delete 를 정의하므로 별도 TU.
  ament_add_gtest(test_rt_closed_chain_alloc test/test_rt_closed_chain_alloc.cpp)
  target_link_libraries(test_rt_closed_chain_alloc ${PROJECT_NAME})

  # #121 Phase 1: frozen-loop FK vs closed-chain FK 실측 하네스.
  ament_add_gtest(test_closed_chain_fk_measurement test/test_closed_chain_fk_measurement.cpp)
  target_link_libraries(test_closed_chain_fk_measurement ${PROJECT_NAME})

  # Compiles the viz node source directly (it is no longer in the core lib).
  # It brings up a ROS context, hence this package's dedicated test domain: on
  # the shared default 0 it would sit on the same discovery bus as every other
  # unclaimed package colcon runs in parallel (#401).
  ament_add_gtest(test_closure_state_publisher
    test/test_closure_state_publisher.cpp
    src/closure_state_publisher.cpp
    ENV ROS_DOMAIN_ID=57)
  ament_target_dependencies(test_closure_state_publisher sensor_msgs)
  target_link_libraries(test_closure_state_publisher ${PROJECT_NAME})

  # Phase 11 (MJCF 교차검증)
  ament_add_gtest(test_mjcf_comparison test/test_mjcf_comparison.cpp)
  target_link_libraries(test_mjcf_comparison ${PROJECT_NAME})
endif()

ament_package()
