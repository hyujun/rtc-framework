cmake_minimum_required(VERSION 3.16)
project(rtc_math VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

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
# Numeric core is Eigen-only. Pinocchio is optional (adapter + tests only).
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Library (header-only, INTERFACE) ─────────────────────────────────────────
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# ── Examples ──────────────────────────────────────────────────────────────────
# SE(3) error-definition comparison experiment (S1–S5). Eigen-only — uses the
# numeric core, no Pinocchio. Pairs with scripts/plot_se3_compare.py.
add_executable(se3_error_compare examples/se3_error_compare.cpp)
target_link_libraries(se3_error_compare ${PROJECT_NAME})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS se3_error_compare
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  # test_se3_module cross-validates log6/Jlog6 against Pinocchio, so it is built
  # only when Pinocchio is available — the Eigen-only core itself has no such
  # dependency (optional-dep contract; mirrors the MuJoCo/aligator pattern).
  find_package(pinocchio QUIET)
  if(pinocchio_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_se3_module test/test_se3_module.cpp)
    target_link_libraries(test_se3_module ${PROJECT_NAME} pinocchio::pinocchio)
  else()
    message(STATUS
      "rtc_math: Pinocchio not found — skipping test_se3_module "
      "(Pinocchio cross-validation). Core is Eigen-only and unaffected.")
  endif()
endif()

ament_package()
