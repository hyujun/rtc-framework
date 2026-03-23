cmake_minimum_required(VERSION 3.16)
project(rtc_inference VERSION 5.16.0 LANGUAGES CXX)

# ── C++20 requirement (concepts, span, etc.) ──────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
  # Discover versioned subdirectories under /opt/onnxruntime (e.g. onnxruntime-linux-x64-1.17.1)
  file(GLOB _ORT_OPT_DIRS "/opt/onnxruntime/onnxruntime-*")
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
