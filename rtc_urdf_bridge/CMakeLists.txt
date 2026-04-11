cmake_minimum_required(VERSION 3.16)
project(rtc_urdf_bridge VERSION 5.17.0 LANGUAGES CXX)

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

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2 QUIET)
if(NOT tinyxml2_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(tinyxml2 REQUIRED IMPORTED_TARGET tinyxml2)
  add_library(tinyxml2::tinyxml2 ALIAS PkgConfig::tinyxml2)
endif()

# ── Library ───────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/urdf_analyzer.cpp
  src/kinematic_chain_extractor.cpp
  src/pinocchio_model_builder.cpp
  src/rt_model_handle.cpp
  src/xacro_processor.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp pinocchio)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
  tinyxml2::tinyxml2
)

# ── Examples ──────────────────────────────────────────────────────────────────
add_executable(example_basic_usage examples/example_basic_usage.cpp)
target_link_libraries(example_basic_usage ${PROJECT_NAME})

add_executable(example_submodel examples/example_submodel.cpp)
target_link_libraries(example_submodel ${PROJECT_NAME})

add_executable(example_tree_model examples/example_tree_model.cpp)
target_link_libraries(example_tree_model ${PROJECT_NAME})

add_executable(example_rt_integration examples/example_rt_integration.cpp)
target_link_libraries(example_rt_integration ${PROJECT_NAME})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS
    example_basic_usage
    example_submodel
    example_tree_model
    example_rt_integration
  DESTINATION lib/${PROJECT_NAME}
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp pinocchio Eigen3 yaml-cpp tinyxml2)
ament_export_include_directories(include)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_urdf_analyzer test/test_urdf_analyzer.cpp)
  target_link_libraries(test_urdf_analyzer ${PROJECT_NAME})

  ament_add_gtest(test_chain_extractor test/test_chain_extractor.cpp)
  target_link_libraries(test_chain_extractor ${PROJECT_NAME})

  ament_add_gtest(test_model_builder test/test_model_builder.cpp)
  target_link_libraries(test_model_builder ${PROJECT_NAME})

  ament_add_gtest(test_rt_model_handle test/test_rt_model_handle.cpp)
  target_link_libraries(test_rt_model_handle ${PROJECT_NAME})

  ament_add_gtest(test_xacro_processor test/test_xacro_processor.cpp)
  target_link_libraries(test_xacro_processor ${PROJECT_NAME})
endif()

ament_package()
