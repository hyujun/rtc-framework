cmake_minimum_required(VERSION 3.16)
project(rtc_scripts VERSION 5.17.0 LANGUAGES)

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
  # Dependency isolation (see README.md "격리 환경 배포 가이드")
  scripts/setup_env.sh
  scripts/build_deps.sh
  scripts/verify_isolation.sh
  scripts/uninstall_system_deps.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/lib/rt_common.sh
  DESTINATION lib/${PROJECT_NAME}/lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_rt_common
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_rt_common.sh
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
