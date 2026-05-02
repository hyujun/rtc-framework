cmake_minimum_required(VERSION 3.16)
project(robot_descriptions VERSION 5.17.0)

find_package(ament_cmake REQUIRED)

# ── Robot-agnostic data hub ───────────────────────────────────────────────────
#
# This package holds raw robot/hand asset files (URDF, MJCF, meshes) only.
# Each robot or hand lives under its own robots/<name>/ subdirectory; no
# build-time processing is performed here.
#
# Asset categories per robot subdirectory (when applicable):
#   urdf/    — URDF and *.urdf.xacro  (Pinocchio / RViz / ros2_control)
#   mjcf/    — MuJoCo XML scenes      (rtc_mujoco_sim)
#   meshes/  — DAE / STL / OBJ        (RViz visual, MuJoCo collision/visual)
#
# Currently shipped:
#   robots/ur5e/         — UR5e 6-DoF arm (urdf pre-generated from
#                          ur_description xacro; regenerate with:
#                            sudo apt install -y ros-jazzy-ur-description \
#                                                ros-jazzy-xacro
#                            xacro <ur_description_path>/ur5e.urdf.xacro ...)
#   robots/assm_v1/      — 10-DoF custom hand "assm v1"
#   robots/ur5e_assm_v1/ — Combined arm+hand (URDF xacro processed at runtime)
#
# To add another robot or hand: drop robots/<name>/{urdf,mjcf,meshes}/ in
# place; downstream YAML/launch references it via the `package://` URL or
# the ament share path.
#

install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

ament_package()
