// ── The shipped g1_p1b models, as a test rig ──────────────────────────────────
//
// The G1 + proto_1b model lives in `hand_description`, a package this
// workspace does not depend on: EVERY TEST THAT INCLUDES THIS HEADER STARTS
// WITH `RTC_SKIP_IF_PACKAGE_MISSING("hand_description")` (optional_package.hpp;
// validate_test_fixtures.py holds the rule). Where the package is absent the
// tests skip and what they cover is unverified there.
//
// The model topology is read from the SHIPPED profile (`g1_p1b/_base.yaml`'s
// `urdf:` block — the sub-models, the tree models, the closure sidecar), so a
// change there moves every suite that includes this. What the profile does not
// have yet is a catching configuration (the wait pose, the budgets and the
// hand's docking values are not shipped): the posture below is this fixture's.
#pragma once

#include "optional_package.hpp"
#include "shipped_catch_arm_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

inline constexpr int kG1Dof = 17;       // the `g1` device: waist 3, left arm 7, right arm 7
inline constexpr int kG1CatchDof = 10;  // `g1_catch`: waist 3, right arm 7

/// The model the catching search runs on, and the one segments are planned on.
inline constexpr const char* kG1SearchModel = "g1_catch";
inline constexpr const char* kG1SegmentModel = "g1";

/// A posture of the `g1` device (its `joint_state_names` order) with both
/// elbows bent and the right palm away from the trunk: no joint on a limit,
/// the right arm away from its straight (singular) pose, the two arms at
/// different angles. THIS FIXTURE'S — the profile ships no catching wait pose.
inline constexpr std::array<double, kG1Dof> kG1Home = {
    0.05,  0.0,   0.03,                            // waist yaw, roll, pitch
    0.25,  0.30,  0.10, 0.70, 0.10, -0.10, 0.0,    // left arm
    -0.45, -0.35, 0.20, 0.95, 0.15, -0.25, 0.10};  // right arm

/// The profile's `urdf:` block as a ModelConfig (the catch frame is added by
/// the loader that reads it).
inline rtc_urdf_bridge::ModelConfig MakeG1P1bModelConfig() {
  const YAML::Node urdf = ShippedNodeParameters("g1_p1b/_base.yaml")["urdf"];
  // The literal is what the fixture gate reads; the profile must agree with it.
  EXPECT_EQ(urdf["package"].as<std::string>(), "hand_description");
  const std::string share = ament_index_cpp::get_package_share_directory("hand_description");
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = share + "/" + urdf["path"].as<std::string>();
  cfg.root_joint_type = urdf["root_joint_type"].as<std::string>();
  cfg.closure_yaml_path = share + "/" + urdf["closure_path"].as<std::string>();
  for (const auto& kv : urdf["sub_models"]) {
    cfg.sub_models.push_back({kv.first.as<std::string>(), kv.second["root_link"].as<std::string>(),
                              kv.second["tip_link"].as<std::string>()});
  }
  for (const auto& kv : urdf["tree_models"]) {
    cfg.tree_models.push_back({kv.first.as<std::string>(), kv.second["root_link"].as<std::string>(),
                               kv.second["tip_links"].as<std::vector<std::string>>()});
  }
  return cfg;
}

/// The two planner models of the profile, on one builder.
struct G1CatchModels {
  ShippedArm search;   ///< `g1_catch`: 10 joints of the 17
  ShippedArm segment;  ///< `g1`: the whole device
};

inline G1CatchModels LoadG1CatchModels() {
  std::vector<ShippedArm> models = LoadShippedModels(
      "g1_p1b", "_base.yaml", MakeG1P1bModelConfig(), {kG1SearchModel, kG1SegmentModel}, "g1",
      std::vector<double>(kG1Home.begin(), kG1Home.end()));
  G1CatchModels out;
  if (models.size() == 2) {
    out.search = std::move(models[0]);
    out.segment = std::move(models[1]);
  }
  return out;
}

}  // namespace integrated_bringup::testfx
