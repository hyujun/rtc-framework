cmake_minimum_required(VERSION 3.16)
project(ur5e_description VERSION 5.17.0)

find_package(ament_cmake REQUIRED)

# ── URDF generation (xacro → ur5e.urdf at build time) ─────────────────────────
#
# Requires:  ros-jazzy-ur-description  ros-jazzy-xacro
#   sudo apt install -y ros-jazzy-ur-description ros-jazzy-xacro
#
# The generated URDF uses the official UR kinematics / physical parameters
# (joint_limits.yaml, default_kinematics.yaml, physical_parameters.yaml) from
# the ur_description package.  Mesh references point to this package's own
# meshes/ur5e/{visual,collision}/ directory via package:// URI.
#

# ── Install robot model files ──────────────────────────────────────────────────
install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

ament_package()
