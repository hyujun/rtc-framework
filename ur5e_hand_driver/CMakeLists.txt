cmake_minimum_required(VERSION 3.16)
project(ur5e_hand_driver VERSION 5.16.0 LANGUAGES CXX)

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
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_inference REQUIRED)
find_package(rtc_msgs REQUIRED)

# ── ONNX Runtime ─────────────────────────────────────────────────────────────
# find_package 우선, 실패 시 pkg-config fallback, 최후 수동 탐색
# F/T inference는 선택적 기능 — 미설치 시 STATUS 메시지만 출력
set(_ORT_FOUND FALSE)
set(ONNXRUNTIME_INCLUDE_DIRS "")
set(ONNXRUNTIME_LIBRARIES "")

# CMake 캐시 초기화 — 이전 빌드의 stale 경로 방지
unset(_ORT_HEADER_DIR CACHE)
unset(ONNXRUNTIME_LIB CACHE)

# 1) 헤더 경로 탐색 (어떤 방식이든 헤더가 있어야 빌드 가능)
find_path(_ORT_HEADER_DIR onnxruntime_cxx_api.h
  PATHS
    /usr/include/onnxruntime/core/session
    /usr/local/include/onnxruntime/core/session
    /usr/include/onnxruntime
    /usr/local/include/onnxruntime
    /opt/onnxruntime/include
  NO_DEFAULT_PATH
)
if(NOT _ORT_HEADER_DIR)
  # NO_DEFAULT_PATH에서 못 찾으면 시스템 기본 경로도 탐색
  find_path(_ORT_HEADER_DIR onnxruntime_cxx_api.h)
endif()

# 1b) 최종 검증: find_path 결과가 실제로 존재하는지 확인 (stale cache 방지)
if(_ORT_HEADER_DIR AND NOT EXISTS "${_ORT_HEADER_DIR}/onnxruntime_cxx_api.h")
  message(STATUS "ONNX Runtime header path stale (${_ORT_HEADER_DIR}) — resetting")
  unset(_ORT_HEADER_DIR CACHE)
  set(_ORT_HEADER_DIR "")
endif()

if(_ORT_HEADER_DIR)
  set(ONNXRUNTIME_INCLUDE_DIRS ${_ORT_HEADER_DIR})

  # 2a) find_package (cmake config)
  find_package(onnxruntime QUIET)
  if(onnxruntime_FOUND OR ONNXRUNTIME_FOUND)
    set(_ORT_FOUND TRUE)
  endif()

  # 2b) pkg-config fallback
  if(NOT _ORT_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(ONNXRUNTIME QUIET libonnxruntime)
      if(ONNXRUNTIME_FOUND)
        set(_ORT_FOUND TRUE)
      endif()
    endif()
  endif()

  # 2c) 수동 라이브러리 탐색
  if(NOT _ORT_FOUND)
    find_library(ONNXRUNTIME_LIB onnxruntime
      PATHS /opt/onnxruntime/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
    )
    if(ONNXRUNTIME_LIB)
      set(_ORT_FOUND TRUE)
      set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIB})
      message(STATUS "ONNX Runtime found (manual): ${ONNXRUNTIME_LIB}")
    endif()
  endif()
endif()

# 최종 게이트: 라이브러리를 찾았더라도 헤더가 실제로 존재해야 활성화
if(_ORT_FOUND)
  if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIRS}/onnxruntime_cxx_api.h")
    message(STATUS "ONNX Runtime library found but header missing at ${ONNXRUNTIME_INCLUDE_DIRS} — disabling")
    set(_ORT_FOUND FALSE)
  else()
    message(STATUS "ONNX Runtime: enabled (headers: ${ONNXRUNTIME_INCLUDE_DIRS})")
  endif()
else()
  message(STATUS "ONNX Runtime not found — F/T inference disabled (optional)")
  if(NOT _ORT_HEADER_DIR)
    message(STATUS "  Install: sudo apt install libonnxruntime-dev")
  endif()
endif()

# ── Unified node: HandController (request-response polling) ──────────────────
add_executable(hand_udp_node
  src/hand_udp_node.cpp
)
target_include_directories(hand_udp_node PRIVATE include)
ament_target_dependencies(hand_udp_node
  rclcpp
  std_msgs
  rtc_base
  rtc_communication
  rtc_inference
  rtc_msgs
)

# ONNX Runtime 링크 (발견된 경우)
if(_ORT_FOUND)
  target_compile_definitions(hand_udp_node PRIVATE HAS_ONNXRUNTIME)
  target_include_directories(hand_udp_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
  if(TARGET onnxruntime::onnxruntime)
    target_link_libraries(hand_udp_node onnxruntime::onnxruntime)
  else()
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

# ── Export headers for downstream packages ────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(rtc_base rtc_communication rtc_inference rtc_msgs)

ament_package()
