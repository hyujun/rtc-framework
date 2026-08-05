cmake_minimum_required(VERSION 3.16)
project(repo_scripts VERSION 5.17.0 LANGUAGES)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  # RT system setup + verification
  scripts/setup_irq_affinity.sh
  scripts/setup_udp_optimization.sh
  scripts/setup_nvidia_rt.sh
  scripts/setup_grub_rt.sh
  scripts/setup_display_rt.sh
  scripts/setup_cpu_governor.sh
  scripts/build_rt_kernel.sh
  scripts/check_rt_setup.sh
  scripts/cpu_shield.sh
  scripts/verify_rt_runtime.sh
  scripts/timeline.sh
  scripts/enroll_lttng_mok.sh
  # Dependency isolation (see README.md "격리 환경 배포 가이드")
  scripts/setup_env.sh
  scripts/build_deps.sh
  scripts/verify_isolation.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/lib/bootstrap.sh
  scripts/lib/rt_common.sh
  # Generated CPU layout table (issue #153 M1). rt_common.sh sources it from its
  # own directory, so it must ship beside it or every installed RT script loses
  # get_rt_cores / get_nrt_cores / rtc_expected_threads. Generated from
  # config/thread_layout.yaml by scripts/gen_thread_layout.py -- never edit it.
  scripts/lib/thread_layout_generated.sh
  scripts/lib/rt_report.sh
  # install.sh modular helpers (sourced from <repo>/install.sh)
  scripts/lib/install_ros2.sh
  scripts/lib/install_python.sh
  scripts/lib/install_uv.sh
  scripts/lib/install_deps.sh
  scripts/lib/install_dev.sh
  scripts/lib/install_rt.sh
  DESTINATION lib/${PROJECT_NAME}/lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_rt_common
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_rt_common.sh
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(test_install_deps
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_install_deps.sh
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  # Stale-shield detection (issue #349 D14). Drives the pure comparison only —
  # cset is not installed on every dev box, so the mask *reader* is stubbed.
  ament_add_test(test_cpu_shield_mask
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpu_shield_mask.sh
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  # ── Thread layout manifest (issue #153 M1) ────────────────────────────────
  # The drift gate: fails if any checked-in artifact (C++ header, shell table,
  # Python mirror, the two README tables) is stale or was edited by hand. CI
  # runs the same command; registering it here means `colcon test` and the Stop
  # hook catch it too, without waiting for a push.
  # TIMEOUT covers the slow path, not the normal one: with a system clang-format
  # this runs in well under a second, but on a box without one the generator
  # falls back to the pinned clang-format through uvx (the same fallback the
  # PostToolUse hook uses) and the first run downloads it.
  ament_add_test(test_gen_thread_layout_check
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_thread_layout.py --check
    TIMEOUT 180
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  # The generator's own invariants: emitters are formatter-stable (so the
  # PostToolUse clang-format/ruff hook cannot make --check fail) and the layout
  # keeps its structural rules on every tier.
  ament_add_test(test_gen_thread_layout_self_test
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_thread_layout.py --self-test
    TIMEOUT 180
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  # Semantic equivalence, not just fresh files: manifest vs *executed* shell
  # helpers vs the imported Python mirror, for ncpu 1..17 and 24. The C++ axis
  # is rtc_base/test/test_thread_layout_tiers.cpp (it needs a compiler).
  ament_add_test(test_thread_layout_equivalence
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_thread_layout_equivalence.py
    TIMEOUT 120
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
