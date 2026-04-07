cmake_minimum_required(VERSION 3.16)
project(rtc_mujoco_sim VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(rtc_base REQUIRED)
find_package(tinyxml2_vendor QUIET)
find_package(tinyxml2 REQUIRED)

# ── mujoco_simulator_node (optional — requires MuJoCo 3.x C++ library) ────────
# Install MuJoCo (binary tarball — no cmake config included):
#   wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
#   sudo tar -xzf mujoco-*.tar.gz -C /opt/
#   colcon build --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x
#
# Two-stage detection:
#   1. find_package(mujoco) — succeeds only when built from source (cmake config present)
#   2. find_library + find_path — works with binary tarball (.so only)
find_package(mujoco QUIET)
find_package(glfw3 QUIET)

if(NOT mujoco_FOUND)
  # Fallback for binary tarball layout: /opt/mujoco-X.Y.Z/{lib/libmujoco.so, include/mujoco/mujoco.h}
  find_library(MUJOCO_LIBRARY
    NAMES mujoco
    HINTS "${mujoco_ROOT}" "$ENV{MUJOCO_DIR}"
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
  )
  find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${mujoco_ROOT}" "$ENV{MUJOCO_DIR}"
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )
  if(MUJOCO_LIBRARY AND MUJOCO_INCLUDE_DIR)
    message(STATUS "MuJoCo found via find_library: ${MUJOCO_LIBRARY}")
    add_library(mujoco::mujoco SHARED IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION             "${MUJOCO_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
    )
    set(mujoco_FOUND TRUE)
  endif()
endif()

if(mujoco_FOUND)
  message(STATUS "MuJoCo found — building mujoco_simulator_node")

  add_executable(mujoco_simulator_node
    src/mujoco_simulator_node.cpp
    src/mujoco_simulator.cpp
    src/mujoco_sim_loop.cpp
    src/ros2_resource_provider.cpp
    src/viewer/viewer_loop.cpp
    src/viewer/viewer_callbacks.cpp
    src/viewer/viewer_overlays.cpp
  )
  target_include_directories(mujoco_simulator_node PRIVATE include src)
  ament_target_dependencies(mujoco_simulator_node
    rclcpp
    std_msgs
    sensor_msgs
    ament_index_cpp
    rtc_msgs
    rtc_base
  )
  target_link_libraries(mujoco_simulator_node mujoco::mujoco tinyxml2::tinyxml2)

  if(glfw3_FOUND)
    message(STATUS "GLFW found — MuJoCo viewer enabled")
    target_compile_definitions(mujoco_simulator_node PRIVATE MUJOCO_HAVE_GLFW)
    target_link_libraries(mujoco_simulator_node glfw)
  else()
    message(STATUS "GLFW not found — MuJoCo viewer disabled (headless only)")
  endif()

  install(TARGETS mujoco_simulator_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "MuJoCo not found — mujoco_simulator_node will NOT be built.\n"
    "  Install MuJoCo 3.x binary tarball under /opt/, then re-run colcon with:\n"
    "    --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x\n"
    "  Or set environment variable MUJOCO_DIR=/opt/mujoco-3.x.x"
  )
endif()

# ── Install ────────────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
