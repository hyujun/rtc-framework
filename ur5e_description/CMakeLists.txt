cmake_minimum_required(VERSION 3.16)
project(ur5e_description VERSION 5.2.2)

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
find_program(XACRO_CMD xacro HINTS /opt/ros/jazzy/bin)

if(XACRO_CMD)
  set(URDF_OUT_DIR  "${CMAKE_CURRENT_BINARY_DIR}/robots/ur5e/urdf")
  set(URDF_OUT_FILE "${URDF_OUT_DIR}/ur5e.urdf")

  file(MAKE_DIRECTORY ${URDF_OUT_DIR})

  add_custom_command(
    OUTPUT "${URDF_OUT_FILE}"
    COMMAND bash -c
      "source /opt/ros/jazzy/setup.bash && \
       ${XACRO_CMD} $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro \
         ur_type:=ur5e \
         name:=ur5e \
         joint_limit_params:=$(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/joint_limits.yaml \
         kinematics_params:=$(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/default_kinematics.yaml \
         physical_params:=$(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/physical_parameters.yaml \
         visual_params:=$(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/visual_parameters.yaml \
         -o ${URDF_OUT_FILE}"
    COMMENT "Generating ur5e.urdf from ur_description xacro"
    VERBATIM
  )
  add_custom_target(generate_ur5e_urdf ALL DEPENDS "${URDF_OUT_FILE}")

  install(
    FILES "${URDF_OUT_FILE}"
    DESTINATION share/${PROJECT_NAME}/robots/ur5e/urdf
  )
  message(STATUS "ur5e_description: URDF will be generated from xacro at build time")
else()
  message(WARNING
    "ur5e_description: xacro not found — URDF will NOT be generated.\n"
    "  Install with:  sudo apt install -y ros-jazzy-ur-description ros-jazzy-xacro"
  )
endif()

# ── Install robot model files ──────────────────────────────────────────────────
install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
  PATTERN "urdf" EXCLUDE   # urdf/ comes from build directory above
)

ament_package()
