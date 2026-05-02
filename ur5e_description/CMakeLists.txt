cmake_minimum_required(VERSION 3.16)
project(ur5e_description VERSION 5.17.0)

find_package(ament_cmake REQUIRED)

# ── Data-only package ─────────────────────────────────────────────────────────
#
# ur5e.urdf is pre-generated from the official UR description xacro.
# No build-time processing is needed.
#
# To regenerate ur5e.urdf (optional):
#   sudo apt install -y ros-jazzy-ur-description ros-jazzy-xacro
#   xacro <ur_description_path>/ur5e.urdf.xacro ...
#
# ur5e_with_hand.urdf.xacro is processed at runtime by xacro (exec_depend).
#
# Directory layout:
#   robots/ur5e/          -- UR5e arm (URDF, MJCF, meshes)
#   robots/hand_tmp/      -- Custom hand (URDF xacro, MJCF)
#   robots/ur5e_hand_tmp/ -- Combined arm+hand (URDF xacro, MJCF scene)
#

# ── Install robot model files ──────────────────────────────────────────────────
install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

ament_package()
