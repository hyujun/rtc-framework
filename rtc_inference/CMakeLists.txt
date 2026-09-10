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

# ── Offline model inspector ───────────────────────────────────────────────────
# Prints what a .onnx exposes, and optionally compares it against inline
# declarations using the SAME CompareModelIo/Format the engine calls at
# configure time — so it cannot drift into disagreeing with the real gate.
# Needs a session to read the model, hence ORT-only.
if(HAS_ONNXRUNTIME)
  # ARCH-7-exempt: offline inspector, not a runtime identity. It takes a model
  # path and shapes on argv, knows no robot, owns no RT loop, and appears in no
  # bringup chain (design-principles.md §Boundary Rules).
  add_executable(rtc_inference_check src/check_model.cpp)
  target_link_libraries(rtc_inference_check rtc_inference)
  install(TARGETS rtc_inference_check DESTINATION lib/${PROJECT_NAME})
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

  # shape_report.hpp is deliberately ONNX-Runtime-free, so this gate runs
  # identically whether or not ORT was found. It proves the comparison RULES;
  # test_onnx_engine below proves they are wired to a real session.
  ament_add_gtest(test_shape_report test/test_shape_report.cpp)
  if(TARGET test_shape_report)
    target_link_libraries(test_shape_report rtc_inference)
  endif()

  # End-to-end gate against a committed .onnx fixture. Its body is #ifdef'd out
  # without ONNX Runtime — with no session to bind to, a green would be
  # meaningless, so the honest state is "nothing to run" rather than a pass.
  # The fixture is committed rather than generated: building it needs the python
  # `onnx` package, which is not a dependency of this repo, and a gate that is
  # silently skipped wherever that package is missing is the one thing a shape
  # check cannot afford (see test/data/generate_fixtures.py).
  ament_add_gtest(test_onnx_engine test/test_onnx_engine.cpp)
  if(TARGET test_onnx_engine)
    target_link_libraries(test_onnx_engine rtc_inference)
    target_compile_definitions(test_onnx_engine PRIVATE
      RTC_INFERENCE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
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
