cmake_minimum_required(VERSION 3.16)
project(rtc_inference VERSION 5.16.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)

# Optional ONNX Runtime
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_library(ONNXRUNTIME_LIB onnxruntime HINTS /opt/onnxruntime/lib)
  find_path(ONNXRUNTIME_INCLUDE onnxruntime_cxx_api.h
    HINTS /opt/onnxruntime/include/onnxruntime/core/session)
  if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE)
    set(HAS_ONNXRUNTIME TRUE)
  endif()
else()
  set(HAS_ONNXRUNTIME TRUE)
endif()

if(HAS_ONNXRUNTIME)
  message(STATUS "ONNX Runtime found — building with inference support")
  add_compile_definitions(HAS_ONNXRUNTIME)
else()
  message(STATUS "ONNX Runtime not found — building stub inference engine")
endif()

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rtc_base)
ament_package()
