cmake_minimum_required(VERSION 3.16)
project(ur5e_hand_udp VERSION 5.16.0 LANGUAGES CXX)

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
find_package(ur5e_rt_base REQUIRED)
find_package(ur5e_msgs REQUIRED)

# ── ONNX Runtime ─────────────────────────────────────────────────────────────
# find_package 우선, 실패 시 pkg-config fallback, 최후 수동 탐색
set(_ORT_FOUND FALSE)

find_package(onnxruntime QUIET)
if(onnxruntime_FOUND OR ONNXRUNTIME_FOUND)
  set(_ORT_FOUND TRUE)
endif()

if(NOT _ORT_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(ONNXRUNTIME QUIET libonnxruntime)
    if(ONNXRUNTIME_FOUND)
      set(_ORT_FOUND TRUE)
    endif()
  endif()
endif()

if(NOT _ORT_FOUND)
  # /opt/onnxruntime 수동 설치 경로 탐색
  find_library(ONNXRUNTIME_LIB onnxruntime
    PATHS /opt/onnxruntime/lib /usr/local/lib
  )
  find_path(ONNXRUNTIME_INCLUDE onnxruntime_cxx_api.h
    PATHS /opt/onnxruntime/include /usr/local/include/onnxruntime
  )
  if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE)
    set(_ORT_FOUND TRUE)
    set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIB})
    set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE})
    message(STATUS "ONNX Runtime found (manual): ${ONNXRUNTIME_LIB}")
  endif()
endif()

# 최종 검증: 어떤 경로로 찾았든 헤더가 실제 존재하는지 확인
if(_ORT_FOUND)
  include(CheckIncludeFileCXX)
  set(CMAKE_REQUIRED_INCLUDES ${ONNXRUNTIME_INCLUDE_DIRS})
  if(TARGET onnxruntime::onnxruntime)
    get_target_property(_ORT_INC onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
    if(_ORT_INC)
      list(APPEND CMAKE_REQUIRED_INCLUDES ${_ORT_INC})
    endif()
  endif()
  check_include_file_cxx("onnxruntime_cxx_api.h" _ORT_HEADER_OK)
  unset(CMAKE_REQUIRED_INCLUDES)
  if(NOT _ORT_HEADER_OK)
    message(WARNING "ONNX Runtime detected but onnxruntime_cxx_api.h not found — F/T inference disabled\n"
                    "  Install headers: sudo apt install libonnxruntime-dev")
    set(_ORT_FOUND FALSE)
  endif()
endif()

if(NOT _ORT_FOUND)
  message(WARNING "ONNX Runtime not found — F/T inference will not be available")
endif()

# ── Unified node: HandController (request-response polling) ──────────────────
add_executable(hand_udp_node
  src/hand_udp_node.cpp
)
target_include_directories(hand_udp_node PRIVATE include)
ament_target_dependencies(hand_udp_node
  rclcpp
  std_msgs
  ur5e_rt_base
  ur5e_msgs
)

# ONNX Runtime 링크 (발견된 경우)
if(_ORT_FOUND)
  target_compile_definitions(hand_udp_node PRIVATE HAS_ONNXRUNTIME)
  if(TARGET onnxruntime::onnxruntime)
    target_link_libraries(hand_udp_node onnxruntime::onnxruntime)
  else()
    target_include_directories(hand_udp_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
    target_link_libraries(hand_udp_node ${ONNXRUNTIME_LIBRARIES})
  endif()
endif()

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  hand_udp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

# ── Export headers for downstream packages (e.g. ur5e_rt_controller) ──────────
ament_export_include_directories(include)
ament_export_dependencies(ur5e_rt_base ur5e_msgs)

ament_package()
