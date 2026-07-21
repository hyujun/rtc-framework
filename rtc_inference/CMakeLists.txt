cmake_minimum_required(VERSION 3.16)
project(rtc_inference VERSION 5.17.0 LANGUAGES CXX)

# ── C++20 requirement (concepts, span, etc.) ──────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
# Directory-scoped: header-only INTERFACE lib has no object of its own, so this
# instruments the test binaries that include the headers.
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

# ── enum switch exhaustiveness ───────────────────────────────────────────────
# 이 패키지는 위 표준 경고 블록(-Wall/-Wextra/...)이 없다 — 별건이다. 다만
# 배선 없는 enum 값이 조용히 통과하면 dead topic/dead branch 가 되므로
# (#205 A-2, #211) switch exhaustiveness 만은 에러로 잡는다. `-Werror=switch`
# 는 -Wall 없이도 해당 진단을 활성화한다.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror=switch)
endif()

find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)

# ── INTERFACE library target ──────────────────────────────────────────────────
add_library(rtc_inference INTERFACE)
target_include_directories(rtc_inference INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── Optional ONNX Runtime ─────────────────────────────────────────────────────
set(HAS_ONNXRUNTIME FALSE)

find_package(onnxruntime QUIET)
if(onnxruntime_FOUND)
  set(HAS_ONNXRUNTIME TRUE)
else()
  # Discover versioned tarball dirs (e.g. /opt/onnxruntime-linux-x64-1.17.1, or
  # nested under /opt/onnxruntime/). The bare /opt/onnxruntime symlink is also
  # covered by the HINTS below.
  file(GLOB _ORT_OPT_DIRS "/opt/onnxruntime-*" "/opt/onnxruntime/onnxruntime-*")
  find_library(ONNXRUNTIME_LIB onnxruntime
    HINTS ${_ORT_OPT_DIRS}
          /opt/onnxruntime /usr/local
          /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
    PATH_SUFFIXES lib)
  find_path(ONNXRUNTIME_INCLUDE onnxruntime_cxx_api.h
    HINTS ${_ORT_OPT_DIRS}
          /opt/onnxruntime /usr/local /usr
    PATH_SUFFIXES
      include/onnxruntime/core/session
      include/onnxruntime)
  if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE)
    set(HAS_ONNXRUNTIME TRUE)
  endif()
endif()

# ── Propagate ONNX Runtime to consumers via INTERFACE ─────────────────────────
if(HAS_ONNXRUNTIME)
  message(STATUS "ONNX Runtime found — building with inference support")
  target_compile_definitions(rtc_inference INTERFACE HAS_ONNXRUNTIME)
  if(TARGET onnxruntime::onnxruntime)
    target_link_libraries(rtc_inference INTERFACE onnxruntime::onnxruntime)
  else()
    target_include_directories(rtc_inference INTERFACE ${ONNXRUNTIME_INCLUDE})
    target_link_libraries(rtc_inference INTERFACE ${ONNXRUNTIME_LIB})
  endif()
else()
  message(STATUS "ONNX Runtime not found — building stub inference engine")
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linking the INTERFACE target propagates the include dir, the HAS_ONNXRUNTIME
  # definition, and the ONNX Runtime link (when present) into the test binary.
  ament_add_gtest(test_inference_engine test/test_inference_engine.cpp)
  if(TARGET test_inference_engine)
    target_link_libraries(test_inference_engine rtc_inference)
  endif()
endif()

# ── Install & Export ──────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(TARGETS rtc_inference EXPORT rtc_inferenceTargets)
install(EXPORT rtc_inferenceTargets
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE rtc_inference::
)

ament_export_targets(rtc_inferenceTargets)
ament_export_include_directories(include)
ament_export_dependencies(rtc_base)
ament_package()
