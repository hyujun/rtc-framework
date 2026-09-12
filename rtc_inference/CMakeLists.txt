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

# Explicit root, checked before anything under /opt. The seam for a runtime that
# is not where the installer puts it (a CI cache, a tarball unpacked without
# sudo to try a new version).
set(RTC_ONNXRUNTIME_ROOT "" CACHE PATH
    "ONNX Runtime root (contains lib/ and include/); searched before /opt")

# No find_package(onnxruntime). The release tarballs ship lib/cmake/onnxruntime,
# and 1.28.2's imported target points at lib64/ while the files are in lib/ — a
# config that is FOUND fails this configure hard, QUIET or not, and cannot be
# caught. It is found in practice: setup_env.sh puts /opt/onnxruntime on
# CMAKE_PREFIX_PATH. (1.17.1 shipped no config, which is why this never
# surfaced before the upgrade.) find_library/find_path below also walk
# CMAKE_PREFIX_PATH and the system dirs, so an apt or custom-prefix runtime is
# still found — only the imported target is lost, and a plain path links the same.
#
# Order matters once two runtimes coexist, which is exactly the state an upgrade
# leaves behind. `install_onnxruntime` points the bare /opt/onnxruntime symlink
# at the PINNED version, so it goes first; versioned dirs follow newest-first.
# The old order walked the glob lexically, where "1.17.1" sorts before "1.28.2" —
# the upgrade would have built against the runtime it was meant to replace, and
# that one cannot load IR-10 models.
#
# But HINTS is not what decides today, and saying otherwise would send the next
# reader to the wrong line: find_library searches CMAKE_PREFIX_PATH (variable AND
# environment) BEFORE HINTS, and setup_env.sh puts /opt/onnxruntime — the pinned
# symlink — on it. So the pin wins there first, and this list is the fallback
# that decides when that symlink is absent. An overlay or CI image that carries a
# DIFFERENT ORT prefix earlier on CMAKE_PREFIX_PATH would win over everything
# here; point RTC_ONNXRUNTIME_ROOT at the intended tree in that case, or drop the
# stray prefix. (Not forced with NO_CMAKE_PATH: that is also the path by which an
# apt or custom-prefix runtime is found on a box with no /opt install at all.)
file(GLOB _ORT_OPT_DIRS "/opt/onnxruntime-*" "/opt/onnxruntime/onnxruntime-*")
list(SORT _ORT_OPT_DIRS COMPARE NATURAL ORDER DESCENDING)
set(_ORT_HINTS ${RTC_ONNXRUNTIME_ROOT} /opt/onnxruntime ${_ORT_OPT_DIRS})
# A cached path would keep an existing build tree on the old runtime after the
# symlink moved, silently. NO_CACHE says so — but it is a CMake 3.21 keyword and
# this package declares 3.16, where it is not a keyword at all: it is swallowed
# as one more PATH_SUFFIXES entry and the variables become cache entries after
# all, with no warning. The explicit unset is what makes the guarantee hold at
# the declared minimum; on 3.21+ it is a no-op next to NO_CACHE.
unset(_RTC_ORT_LIB CACHE)
unset(_RTC_ORT_INCLUDE CACHE)
find_library(_RTC_ORT_LIB onnxruntime
  HINTS ${_ORT_HINTS}
        /usr/local /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
  PATH_SUFFIXES lib
  NO_CACHE)
find_path(_RTC_ORT_INCLUDE onnxruntime_cxx_api.h
  HINTS ${_ORT_HINTS} /usr/local /usr
  PATH_SUFFIXES
    include
    include/onnxruntime/core/session
    include/onnxruntime
  NO_CACHE)
if(_RTC_ORT_LIB AND _RTC_ORT_INCLUDE)
  set(HAS_ONNXRUNTIME TRUE)
endif()

# Say WHICH runtime was picked. A model's IR version is checked only when a
# session opens it, so building against a runtime too old for the shipped
# policy looks like a perfectly good build until on_configure.
# ORT_API_VERSION tracks the minor release; IR 10 needs 1.18 or newer.
if(HAS_ONNXRUNTIME AND _RTC_ORT_INCLUDE AND EXISTS "${_RTC_ORT_INCLUDE}/onnxruntime_c_api.h")
  file(STRINGS "${_RTC_ORT_INCLUDE}/onnxruntime_c_api.h" _ORT_API_LINE
       REGEX "^#define ORT_API_VERSION [0-9]+")
  string(REGEX MATCH "[0-9]+$" _ORT_API "${_ORT_API_LINE}")
  message(STATUS "ONNX Runtime API ${_ORT_API} at ${_RTC_ORT_LIB}")
  if(_ORT_API AND _ORT_API LESS 18)
    message(WARNING
      "ONNX Runtime API ${_ORT_API} (1.${_ORT_API}) cannot open IR-10 models such as the "
      "ur5e_p1b demo_inference policy. Install the version pinned in install.sh "
      "(install_onnxruntime re-points /opt/onnxruntime) and reconfigure.")
  endif()
endif()

# ── Propagate ONNX Runtime to consumers via INTERFACE ─────────────────────────
if(HAS_ONNXRUNTIME)
  message(STATUS "ONNX Runtime found — building with inference support")
  target_compile_definitions(rtc_inference INTERFACE HAS_ONNXRUNTIME)
  target_include_directories(rtc_inference INTERFACE ${_RTC_ORT_INCLUDE})
  target_link_libraries(rtc_inference INTERFACE ${_RTC_ORT_LIB})
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
