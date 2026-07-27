cmake_minimum_required(VERSION 3.16)
project(rtc_controller_manager VERSION 5.17.0 LANGUAGES CXX)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)

# RTC packages
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_msgs REQUIRED)

# ── Static library ────────────────────────────────────────────────────────────
add_library(rtc_controller_manager_lib STATIC
  src/rt_controller_node.cpp
  src/rt_controller_node_params.cpp
  src/rt_controller_node_publishers.cpp
  src/rt_controller_node_switch.cpp
  src/rt_controller_node_services.cpp
  src/rt_controller_node_rt_loop.cpp
  src/rt_controller_node_publish.cpp
  src/rt_controller_node_estop.cpp
  src/rt_controller_node_device_config.cpp
  src/rt_controller_main_impl.cpp
  src/device_backend_registry.cpp
)
target_include_directories(rtc_controller_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_controller_manager_lib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  ament_index_cpp
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_communication
  rtc_msgs
  rtc_urdf_bridge
)
target_link_libraries(rtc_controller_manager_lib
  yaml-cpp
  pinocchio::pinocchio
)

# ── RT tracing (opt-in) ──────────────────────────────────────────────────────
# When rtc_base was built with -DRTC_ENABLE_TRACING=ON it exports the
# `rtc_tracing` target carrying RTC_TRACING_ENABLED (PUBLIC) + the lttng include
# path. Linking it PUBLIC activates the RTC_TRACE_SCOPE spans in this lib's RT
# loop and propagates the define to downstream bringup packages that link this
# lib. Default OFF → no-op, no lttng dependency. See rtc_base/CMakeLists.txt.
option(RTC_ENABLE_TRACING "Compile rtc:* LTTng UST tracepoints (RT tracing)" OFF)
if(RTC_ENABLE_TRACING)
  target_link_libraries(rtc_controller_manager_lib rtc_base::rtc_tracing)
  # Define set here (PRIVATE), not inherited from rtc_tracing — see the
  # interface-leak note in rtc_base/CMakeLists.txt. This keeps the opt-in local
  # so an ON rtc_base never silently instruments an OFF consumer.
  target_compile_definitions(rtc_controller_manager_lib PRIVATE RTC_TRACING_ENABLED)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
# rtc_controller_manager is library-only (no executable). Robot-specific
# bringup packages (e.g. integrated_bringup) link rtc_controller_manager_lib and
# expose their own executable via rtc::RtControllerMain(argc, argv, node_name).
# See agent_docs/design-principles.md.
install(TARGETS rtc_controller_manager_lib
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

# ── Export library for external bringup packages ─────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  ament_index_cpp
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_communication
  rtc_msgs
  rtc_urdf_bridge
  pinocchio
  yaml-cpp
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_controller_timing_profiler
    test/test_controller_timing_profiler.cpp)
  if(TARGET test_controller_timing_profiler)
    target_include_directories(test_controller_timing_profiler PRIVATE include)
    ament_target_dependencies(test_controller_timing_profiler
      rtc_controller_interface rtc_base rtc_urdf_bridge)
    target_link_libraries(test_controller_timing_profiler yaml-cpp)
  endif()

  ament_add_gtest(test_controller_lifecycle
    test/test_controller_lifecycle.cpp)
  if(TARGET test_controller_lifecycle)
    target_include_directories(test_controller_lifecycle PRIVATE include)
    ament_target_dependencies(test_controller_lifecycle
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_controller_lifecycle
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  ament_add_gtest(test_switch_service
    test/test_switch_service.cpp)
  if(TARGET test_switch_service)
    target_include_directories(test_switch_service PRIVATE include)
    ament_target_dependencies(test_switch_service
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_switch_service
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # #260 /rtc_cm/reset_fault: the only external exit from a latched
  # controller-local SAFE_STOP. Authority (name required, active-only), outcome
  # reporting, and E-8 separation from the global E-STOP in both directions.
  ament_add_gtest(test_reset_fault_service
    test/test_reset_fault_service.cpp)
  if(TARGET test_reset_fault_service)
    target_include_directories(test_reset_fault_service PRIVATE include)
    ament_target_dependencies(test_reset_fault_service
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_reset_fault_service
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  ament_add_gtest(test_device_backend_registry
    test/test_device_backend_registry.cpp)
  if(TARGET test_device_backend_registry)
    target_include_directories(test_device_backend_registry PRIVATE include)
    ament_target_dependencies(test_device_backend_registry
      rclcpp rclcpp_lifecycle rtc_base)
    target_link_libraries(test_device_backend_registry
      rtc_controller_manager_lib)
  endif()

  ament_add_gtest(test_device_backend_cb_group_injection
    test/test_device_backend_cb_group_injection.cpp)
  if(TARGET test_device_backend_cb_group_injection)
    target_include_directories(test_device_backend_cb_group_injection PRIVATE include)
    ament_target_dependencies(test_device_backend_cb_group_injection
      rclcpp rclcpp_lifecycle sensor_msgs std_msgs rtc_base)
    target_link_libraries(test_device_backend_cb_group_injection
      rtc_controller_manager_lib)
  endif()

  ament_add_gtest(test_estop
    test/test_estop.cpp)
  if(TARGET test_estop)
    target_include_directories(test_estop PRIVATE include test)
    ament_target_dependencies(test_estop
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs
      rtc_controller_interface rtc_base rtc_msgs)
    target_link_libraries(test_estop
      rtc_controller_manager_lib)
  endif()

  ament_add_gtest(test_eventfd_teardown_order
    test/test_eventfd_teardown_order.cpp)
  if(TARGET test_eventfd_teardown_order)
    target_include_directories(test_eventfd_teardown_order PRIVATE include test)
    ament_target_dependencies(test_eventfd_teardown_order
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs
      rtc_controller_interface rtc_base rtc_msgs)
    target_link_libraries(test_eventfd_teardown_order
      rtc_controller_manager_lib)
  endif()

  ament_add_gtest(test_on_configure
    test/test_on_configure.cpp)
  if(TARGET test_on_configure)
    target_include_directories(test_on_configure PRIVATE include)
    ament_target_dependencies(test_on_configure
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_on_configure
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  ament_add_gtest(test_create_device_backends
    test/test_create_device_backends.cpp)
  if(TARGET test_create_device_backends)
    target_include_directories(test_create_device_backends PRIVATE include test)
    ament_target_dependencies(test_create_device_backends
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs
      rtc_controller_interface rtc_base rtc_msgs)
    target_link_libraries(test_create_device_backends
      rtc_controller_manager_lib)
  endif()

  # Full-pipeline 테스트 2종은 실제 bring-up (다수 LifecycleNode + pub 생성/소멸
  # 반복, RT 루프 tick) 을 돌리므로 전용 ROS_DOMAIN_ID 로 격리한다 — CI 의
  # colcon test 는 패키지를 병렬 실행하고, 기본 도메인 공유 시 DDS discovery
  # churn 이 동시 실행 중인 타 패키지의 pub/sub 매칭 테스트 (ur5e_bt_coordinator
  # test_rewire_gate 가 PR #187 에서 실측 피해) 를 깨뜨린다.
  ament_add_gtest(test_cm_config_pipeline
    test/test_cm_config_pipeline.cpp
    ENV ROS_DOMAIN_ID=44)
  if(TARGET test_cm_config_pipeline)
    target_include_directories(test_cm_config_pipeline PRIVATE include test)
    ament_target_dependencies(test_cm_config_pipeline
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_cm_config_pipeline
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  ament_add_gtest(test_rt_loop_pipeline
    test/test_rt_loop_pipeline.cpp
    ENV ROS_DOMAIN_ID=45)
  if(TARGET test_rt_loop_pipeline)
    target_include_directories(test_rt_loop_pipeline PRIVATE include test)
    ament_target_dependencies(test_rt_loop_pipeline
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_rt_loop_pipeline
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # Separate binary: the controller registry is process-global, so its
  # two-group controller would reshape every node in the single-group pipeline
  # TU (issue #198 §1).
  ament_add_gtest(test_device_readiness_gate
    test/test_device_readiness_gate.cpp
    ENV ROS_DOMAIN_ID=50)
  if(TARGET test_device_readiness_gate)
    target_include_directories(test_device_readiness_gate PRIVATE include test)
    ament_target_dependencies(test_device_readiness_gate
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_device_readiness_gate
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # Separate binary: CM instantiates every registered controller, so a
  # torque-mode controller here would be command-type-checked against the
  # position-only stub backend in every node of the pipeline TU.
  ament_add_gtest(test_command_type_gate
    test/test_command_type_gate.cpp
    ENV ROS_DOMAIN_ID=51)
  if(TARGET test_command_type_gate)
    target_include_directories(test_command_type_gate PRIVATE include test)
    ament_target_dependencies(test_command_type_gate
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_command_type_gate
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # 자체 실행파일이 필수다 — ControllerRegistry 는 제거 API 가 없는 프로세스
  # 싱글턴이라, 이 테스트가 등록하는 중복 key 는 바이너리 수명 내내 남는다.
  ament_add_gtest(test_cm_duplicate_controller_key
    test/test_cm_duplicate_controller_key.cpp
    ENV ROS_DOMAIN_ID=46)
  if(TARGET test_cm_duplicate_controller_key)
    target_include_directories(test_cm_duplicate_controller_key PRIVATE include test)
    ament_target_dependencies(test_cm_duplicate_controller_key
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_cm_duplicate_controller_key
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # 위와 같은 이유로 또 하나의 전용 실행파일 — Name() 충돌을 등록하는 순간
  # registry 는 바이너리 수명 내내 오염되므로 duplicate-key 테스트와 섞을 수 없다.
  ament_add_gtest(test_cm_controller_name_collision
    test/test_cm_controller_name_collision.cpp
    ENV ROS_DOMAIN_ID=47)
  if(TARGET test_cm_controller_name_collision)
    target_include_directories(test_cm_controller_name_collision PRIVATE include test)
    ament_target_dependencies(test_cm_controller_name_collision
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_cm_controller_name_collision
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # 또 하나의 전용 실행파일 — config_key 가 앞선 등록의 Name() 을 가리는 형태
  # (issue #205 B-1). 위 name_collision 테스트의 fixture 는 Name() 과 config_key
  # 가 같은 문자열이라 Tier 2 의 config_key 쪽 arm 을 건드리지 못한다.
  ament_add_gtest(test_cm_config_key_collision
    test/test_cm_config_key_collision.cpp
    ENV ROS_DOMAIN_ID=48)
  if(TARGET test_cm_config_key_collision)
    target_include_directories(test_cm_config_key_collision PRIVATE include test)
    ament_target_dependencies(test_cm_config_key_collision
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_cm_config_key_collision
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()

  # initial_controller 가 config_key / Name() 양쪽으로 해석되는지 (issue #205 B-2).
  # 등록을 런타임에 추가하지 않으므로 registry 오염은 없지만, 두 컨트롤러를
  # static 등록하므로 다른 테스트와 섞으면 그쪽 registry 크기 단언이 깨진다.
  ament_add_gtest(test_cm_identifier_resolution
    test/test_cm_identifier_resolution.cpp
    ENV ROS_DOMAIN_ID=49)
  if(TARGET test_cm_identifier_resolution)
    target_include_directories(test_cm_identifier_resolution PRIVATE include test)
    ament_target_dependencies(test_cm_identifier_resolution
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_cm_identifier_resolution
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()
endif()

ament_package()
