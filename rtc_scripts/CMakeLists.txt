cmake_minimum_required(VERSION 3.16)
project(rtc_scripts VERSION 5.16.0 LANGUAGES)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/setup_irq_affinity.sh
  scripts/setup_udp_optimization.sh
  scripts/setup_nvidia_rt.sh
  scripts/build_rt_kernel.sh
  scripts/check_rt_setup.sh
  scripts/cpu_shield.sh
  scripts/verify_rt_runtime.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/lib/rt_common.sh
  DESTINATION lib/${PROJECT_NAME}/lib
)

ament_package()
