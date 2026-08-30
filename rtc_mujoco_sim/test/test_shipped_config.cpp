// ── test_shipped_config.cpp ───────────────────────────────────────────────────
// Every ros__parameters YAML this package ships must survive rclcpp's own
// parameter loader.
//
// Why this needs a test at all: a params YAML is not validated by anything the
// package builds. It is parsed by rcl_yaml_param_parser at *node construction*,
// so a malformed key does not fail a build, does not fail an existing test, and
// does not reach a single line of MuJoCoSimulator's own "failures are loud"
// validation — the node aborts before on_configure runs. The one construct that
// bites here is an empty sequence: `objects: []` gives the parser no element
// type to infer, so it yields PARAMETER_NOT_SET and rclcpp throws
// InvalidParameterValueException. mujoco_default.yaml shipped exactly that and
// nothing caught it, because that file is a copy-paste template no launch loads
// (README) — the abort would have landed in whichever robot bringup copied it.
//
// parameter_map_from_yaml_file is the same entry point a Node's constructor
// reaches through, so a green assertion here means the file can actually back a
// node, not merely that it is well-formed YAML.
// ──────────────────────────────────────────────────────────────────────────────
#include <rclcpp/parameter_map.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#ifndef MUJOCO_DEFAULT_YAML_PATH
#error "MUJOCO_DEFAULT_YAML_PATH must be defined by CMake"
#endif
#ifndef SOLVER_PARAM_YAML_PATH
#error "SOLVER_PARAM_YAML_PATH must be defined by CMake"
#endif
#ifndef SHIPPED_CONFIG_SCRATCH_DIR
#error "SHIPPED_CONFIG_SCRATCH_DIR must be defined by CMake"
#endif

namespace rtc {
namespace {

// The template every new robot bringup starts from. Its whole job is to be
// copied, so a construct that aborts a node here aborts it downstream too.
TEST(ShippedConfig, MujocoDefaultTemplateLoadsThroughRclcppParameterMap) {
  ASSERT_TRUE(std::filesystem::exists(MUJOCO_DEFAULT_YAML_PATH));
  EXPECT_NO_THROW({
    const auto params = rclcpp::parameter_map_from_yaml_file(MUJOCO_DEFAULT_YAML_PATH);
    EXPECT_FALSE(params.empty());
  });
}

// Loaded by every sim launch, so a break here is not latent at all.
TEST(ShippedConfig, SolverParamLoadsThroughRclcppParameterMap) {
  ASSERT_TRUE(std::filesystem::exists(SOLVER_PARAM_YAML_PATH));
  EXPECT_NO_THROW({
    const auto params = rclcpp::parameter_map_from_yaml_file(SOLVER_PARAM_YAML_PATH);
    EXPECT_FALSE(params.empty());
  });
}

// Positive control for the two tests above. Without it they are only evidence
// that the loader was *called*: if a future rclcpp accepted empty sequences, or
// the loader were swapped for a plain yaml-cpp parse, both would stay green
// while guarding nothing. This pins the failure mode itself — an empty sequence
// under ros__parameters throws, which is the whole reason `objects:` ships
// commented out rather than as `[]`.
TEST(ShippedConfig, EmptySequenceUnderRosParametersIsRejectedByTheLoader) {
  const std::filesystem::path dir{SHIPPED_CONFIG_SCRATCH_DIR};
  std::filesystem::create_directories(dir);
  const std::filesystem::path path = dir / "empty_sequence.yaml";
  {
    std::ofstream out(path);
    ASSERT_TRUE(out.is_open());
    out << "mujoco_simulator:\n"
        << "  ros__parameters:\n"
        << "    object_pool:\n"
        << "      enabled: false\n"
        << "      objects: []\n";
  }

  EXPECT_THROW(rclcpp::parameter_map_from_yaml_file(path.string()),
               rclcpp::exceptions::InvalidParameterValueException);

  // ... and the same file without the empty sequence is fine, so the throw is
  // attributable to that key and not to anything else in the fixture.
  const std::filesystem::path ok_path = dir / "no_empty_sequence.yaml";
  {
    std::ofstream out(ok_path);
    ASSERT_TRUE(out.is_open());
    out << "mujoco_simulator:\n"
        << "  ros__parameters:\n"
        << "    object_pool:\n"
        << "      enabled: false\n";
  }
  EXPECT_NO_THROW(rclcpp::parameter_map_from_yaml_file(ok_path.string()));
}

}  // namespace
}  // namespace rtc
