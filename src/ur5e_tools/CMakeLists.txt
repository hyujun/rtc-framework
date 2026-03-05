cmake_minimum_required(VERSION 3.16)
project(ur5e_tools)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables
install(PROGRAMS
  scripts/monitor_data_health.py
  scripts/plot_ur_trajectory.py
  scripts/motion_editor_gui.py
  scripts/hand_udp_sender_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
