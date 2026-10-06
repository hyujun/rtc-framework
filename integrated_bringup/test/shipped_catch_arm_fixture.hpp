// ── The shipped catch sub-models, as a test rig ───────────────────────────────
// (test-only; shared by the suites that record a numeric core or a search on
//  the models the planner is actually handed)
//
// rtc_controllers' own suites solve on the plain arm URDFs, because the
// hand-locked sub-models are xacro and only this package builds them. This
// loads `ur5e_catch` and `iiwa7_catch` exactly as the shipped profiles declare
// them — the sub-model's root and tip, the catch frame's parent, offset and
// rotation, the arm device's joint names and ratings, all read from the
// shipped YAML — so a change there moves every suite that includes this.
//
// The capture-set parameters are the docking fixture's synthetic ones
// (mpc_docking::BaseParams): the identified values are E1-F15's.
#pragma once

#include "iiwa7_leap_test_fixture.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

namespace mpc_docking = rtc::testing::mpc_docking;

struct ShippedArm {
  mpc_docking::Rig rig;
  double catch_mass{0.0};  // total mass of the sub-model the core computes on
  /// `device_of_model[m]` = the arm device's index of model joint m (the
  /// device's `joint_state_names` order against the sub-model's velocity
  /// order).
  std::vector<int> device_of_model;
};

// The profile's node parameters (`/**: ros__parameters:`), as shipped.
inline YAML::Node ShippedNodeParameters(const std::string& relative) {
  const YAML::Node root = YAML::LoadFile(std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + relative);
  EXPECT_TRUE(root.IsMap() && root.size() >= 1) << relative;
  return root.begin()->second["ros__parameters"];
}

// The catch sub-model exactly as the shipped profile declares it: the
// sub-model's root and tip, the catch frame's parent, offset and rotation, the
// arm device's joint names and ratings — all read from the shipped YAML, so a
// change there moves this test with it.
inline ShippedArm LoadShippedArm(const std::string& profile, const std::string& yaml,
                                 rtc_urdf_bridge::ModelConfig cfg, const std::string& arm_device,
                                 const std::vector<double>& home_device) {
  ShippedArm out;
  const YAML::Node params = ShippedNodeParameters(profile + "/" + yaml);
  const YAML::Node controller = ShippedControllerNode(profile, "demo_catching_controller");
  const std::string sub_model = rtc::catching::ParsePlannerParams(controller["catching"]).sub_model;
  EXPECT_FALSE(sub_model.empty());
  const YAML::Node sub = params["urdf"]["sub_models"][sub_model];
  EXPECT_TRUE(sub) << profile << ": urdf.sub_models." << sub_model;
  const auto declared = std::find_if(cfg.sub_models.begin(), cfg.sub_models.end(),
                                     [&sub_model](const auto& s) { return s.name == sub_model; });
  if (declared == cfg.sub_models.end()) {
    cfg.sub_models.push_back(
        {sub_model, sub["root_link"].as<std::string>(), sub["tip_link"].as<std::string>()});
  }
  const YAML::Node frame = params["urdf"]["extra_frames"]["catch_frame"];
  EXPECT_TRUE(frame) << profile << ": urdf.extra_frames.catch_frame";
  rtc_urdf_bridge::ExtraFrameConfig extra;
  extra.name = "catch_frame";
  extra.parent = frame["parent"].as<std::string>();
  const auto xyz = frame["xyz"].as<std::vector<double>>();
  extra.xyz = Eigen::Vector3d(xyz.at(0), xyz.at(1), xyz.at(2));
  if (frame["rpy"]) {
    const auto rpy = frame["rpy"].as<std::vector<double>>();
    extra.rpy = Eigen::Vector3d(rpy.at(0), rpy.at(1), rpy.at(2));
  }
  extra.provisional = false;
  cfg.extra_frames.push_back(extra);

  rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
  const std::shared_ptr<const pinocchio::Model> model = builder.GetReducedModel(sub_model);
  EXPECT_TRUE(model) << sub_model;
  if (!model) {
    return out;
  }
  out.catch_mass = pinocchio::computeTotalMass(*model);

  const YAML::Node device = params["devices"][arm_device];
  const auto names = device["joint_state_names"].as<std::vector<std::string>>();
  const auto max_velocity = device["joint_limits"]["max_velocity"].as<std::vector<double>>();
  const auto max_torque = device["joint_limits"]["max_torque"].as<std::vector<double>>();
  const Eigen::Index n = model->nv;
  EXPECT_EQ(static_cast<std::size_t>(n), names.size());
  EXPECT_EQ(static_cast<std::size_t>(n), home_device.size());

  out.rig.arm.model = model;
  out.rig.arm.frame = model->getFrameId("catch_frame");
  out.rig.arm.name = sub_model;
  out.rig.arm.q_nominal.resize(n);
  out.device_of_model.assign(static_cast<std::size_t>(n), -1);
  out.rig.model = *model;  // no extra armature: the shipped planner adds none
  out.rig.limits.q_min = model->lowerPositionLimit;
  out.rig.limits.q_max = model->upperPositionLimit;
  out.rig.limits.qd_max.resize(n);
  out.rig.limits.tau_max.resize(n);
  out.rig.limits.armature = Eigen::VectorXd::Zero(n);
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(model->njoints);
       ++jid) {
    const auto it = std::find(names.begin(), names.end(), model->names[jid]);
    EXPECT_NE(it, names.end()) << model->names[jid];
    if (it == names.end()) {
      continue;
    }
    const auto d = static_cast<std::size_t>(std::distance(names.begin(), it));
    const Eigen::Index m = model->joints[jid].idx_v();
    out.rig.arm.q_nominal[m] = home_device.at(d);
    out.device_of_model.at(static_cast<std::size_t>(m)) = static_cast<int>(d);
    out.rig.limits.qd_max[m] = max_velocity.at(d);
    out.rig.limits.tau_max[m] = max_torque.at(d);
  }
  out.rig.params = mpc_docking::BaseParams(static_cast<int>(n));
  out.rig.params.q_nom = out.rig.arm.q_nominal;
  return out;
}

inline std::vector<ShippedArm> ShippedArms() {
  std::vector<ShippedArm> arms;
  arms.push_back(LoadShippedArm("ur5e_p1b", "_base.yaml", MakeUr5eP1bModelConfig(), "ur5e",
                                std::vector<double>(kUr5eHome.begin(), kUr5eHome.end())));
  arms.push_back(LoadShippedArm("iiwa7_leap", "sim.yaml", MakeIiwa7LeapModelConfig(), "iiwa7",
                                std::vector<double>(kArmHome.begin(), kArmHome.end())));
  return arms;
}

}  // namespace integrated_bringup::testfx
