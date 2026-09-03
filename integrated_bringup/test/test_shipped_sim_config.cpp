// ── test_shipped_sim_config.cpp ──────────────────────────────────────────────
// Every shipped config/<profile>/mujoco_simulator.yaml must survive rclcpp's
// own parameter loader, and the keys the sim launches depend on must arrive
// with the types the node declares.
//
// WHY A TEST AT ALL. A params YAML is not validated by anything this package
// builds. It is parsed by rcl_yaml_param_parser at *node construction*, so a
// bad key fails no build, fails no existing test, and never reaches a line of
// MuJoCoSimulator's own "failures are loud" validation — the node aborts before
// on_configure. The repo's Stop hook does run a YAML gate over changed files,
// but it is a plain PyYAML parse: it accepts every construct below, because
// each is well-formed YAML that only rclcpp rejects.
//
// The two constructs that actually bite here, both silent until launch:
//   - an empty sequence (`objects: []`) gives the parser no element type to
//     infer, yields PARAMETER_NOT_SET, and rclcpp throws;
//   - a sequence written with integer literals (`position: [0, 0, 0.2]`, or
//     `initial_qpos: [0, -1, 1]`) loads as an INTEGER array and never matches
//     the node's declared DOUBLE array.
//
// rtc_mujoco_sim's own test_shipped_config covers that package's two template
// files and owns the positive control proving the loader really rejects an
// empty sequence. This one covers the robot bringup configs those templates get
// copied into — the files an abort would actually land in — and goes further
// than "no throw" by asserting the loaded types of the keys the launches read.
// ──────────────────────────────────────────────────────────────────────────────
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace integrated_bringup {
namespace {

// Enumerated rather than globbed: a profile that loses its mujoco_simulator.yaml
// should fail this test, not silently drop out of the matrix.
const std::vector<std::string> kProfiles{"ur5e_p1a", "ur5e_p1b", "iiwa7_leap"};

std::string SimConfigPath(const std::string& profile) {
  return std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/mujoco_simulator.yaml";
}

// Keys land under the node's fully qualified name ("/mujoco_simulator"), so
// this matches on a substring rather than assuming the leading slash — the
// files are also loaded by launches that could namespace the node.
std::vector<rclcpp::Parameter> ParamsForNode(const rclcpp::ParameterMap& map) {
  for (const auto& [node_name, params] : map) {
    if (node_name.find("mujoco_simulator") != std::string::npos) {
      return params;
    }
  }
  return {};
}

class ShippedSimConfig : public ::testing::TestWithParam<std::string> {};

TEST_P(ShippedSimConfig, LoadsThroughRclcppParameterMap) {
  const auto path = SimConfigPath(GetParam());
  ASSERT_TRUE(std::filesystem::exists(path)) << path;
  EXPECT_NO_THROW({
    const auto map = rclcpp::parameter_map_from_yaml_file(path);
    EXPECT_FALSE(map.empty()) << path << ": loaded but produced no parameters";
  });
}

// Without this the test above only proves the loader was *called*: a file whose
// keys all landed under the wrong node name, or whose numeric arrays came back
// as integers, still loads without throwing.
TEST_P(ShippedSimConfig, DeclaredArrayKeysLoadAsDoubleArrays) {
  const auto path = SimConfigPath(GetParam());
  const auto params = ParamsForNode(rclcpp::parameter_map_from_yaml_file(path));
  ASSERT_FALSE(params.empty()) << path << ": no mujoco_simulator parameters found";

  // Every key the node declares as a double array. An integer literal anywhere
  // in one of these makes the whole array an integer array, which throws at
  // node construction — before on_configure, so nothing else can catch it.
  const std::vector<std::string> double_array_keys{
      "servo_kp",
      "servo_kd",
      "object_pool.position",
      "object_pool.position_variation",
      "object_pool.rpy",
      "object_pool.rpy_variation",
      "object_pool.park_position",
  };

  int checked = 0;
  for (const auto& p : params) {
    const bool is_initial_qpos = p.get_name().find(".initial_qpos") != std::string::npos;
    bool is_listed = false;
    for (const auto& key : double_array_keys) {
      if (p.get_name() == key) {
        is_listed = true;
        break;
      }
    }
    if (!is_listed && !is_initial_qpos) {
      continue;
    }
    EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        << path << ": '" << p.get_name() << "' loaded as " << p.get_type_name()
        << ". Write every entry with a decimal point (0.0, not 0) — a ROS 2 params "
           "sequence takes its type from its entries.";
    ++checked;
  }
  EXPECT_GT(checked, 0) << path << ": no double-array key found — this assertion would be vacuous";
}

INSTANTIATE_TEST_SUITE_P(Profiles, ShippedSimConfig, ::testing::ValuesIn(kProfiles),
                         [](const ::testing::TestParamInfo<std::string>& profile_info) {
                           return profile_info.param;
                         });

}  // namespace
}  // namespace integrated_bringup
