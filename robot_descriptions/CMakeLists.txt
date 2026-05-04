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
# ── Consumer rule (data-only invariant — see agent_docs/invariants.md ARCH-5) ─
#
# This package intentionally exports NO CMake targets, headers, or libraries —
# just an install(DIRECTORY) tree. Consumers MUST depend on it via:
#   - package.xml: <exec_depend>robot_descriptions</exec_depend>   (NOT <depend>)
#   - C++:        ament_index_cpp::get_package_share_directory("robot_descriptions")
#   - Python:     ament_index_python.packages.get_package_share_directory(...)
#   - URDF/MJCF/launch/YAML: package://robot_descriptions/robots/<name>/...
#                            or the bare string "robot_descriptions"
#                            (rtc_controller_manager resolves it at runtime)
#
# DO NOT add `find_package(robot_descriptions)` or list it in
# `ament_target_dependencies` / `ament_export_dependencies` — there is
# nothing to link, and doing so forces a spurious colcon build-topology
# edge that breaks the "drop this directory anywhere under <ws>/src and
# downstream finds it via install/share" workflow.
#
# If you ever need C++ utilities here, split them into a separate package
# (e.g. robot_descriptions_utils) so this one stays data-only.
#

install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

ament_package()
