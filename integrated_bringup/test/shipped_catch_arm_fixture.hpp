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
// (mpc_docking::BaseParams): the identified values are not shipped.
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
  /// device's `joint_state_names` order against the model's velocity order).
  std::vector<int> device_of_model;
  /// The arm device's joint count: the model's when it is the whole device,
  /// more when the model is a chain of it (a search model).
  int n_dev{0};
};

// The profile's node parameters (`/**: ros__parameters:`), as shipped.
inline YAML::Node ShippedNodeParameters(const std::string& relative) {
  const YAML::Node root = YAML::LoadFile(std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + relative);
  EXPECT_TRUE(root.IsMap() && root.size() >= 1) << relative;
  return root.begin()->second["ros__parameters"];
}

// The models `names` of one shipped profile, each exactly as the profile
// declares it, all on ONE builder (one URDF parse): a name is the profile's
// `urdf.sub_models` entry (a chain, root → tip) or, failing that, its
// `urdf.tree_models` entry — the order a controller's configure looks a
// planner model up in. The catch frame's parent, offset and rotation and the
// arm device's joint names and ratings are read from the shipped YAML too, so
// a change there moves every suite that includes this.
//
// A model may have FEWER joints than the device (a chain of it):
// `device_of_model` is then an injection into the device and `home_device` —
// one entry per DEVICE joint — is read at the model's joints.
inline std::vector<ShippedArm> LoadShippedModels(const std::string& profile,
                                                 const std::string& yaml,
                                                 rtc_urdf_bridge::ModelConfig cfg,
                                                 const std::vector<std::string>& names,
                                                 const std::string& arm_device,
                                                 const std::vector<double>& home_device) {
  std::vector<ShippedArm> out;
  const YAML::Node params = ShippedNodeParameters(profile + "/" + yaml);
  std::vector<bool> is_sub;
  for (const std::string& name : names) {
    const YAML::Node sub = params["urdf"]["sub_models"][name];
    const YAML::Node tree = params["urdf"]["tree_models"][name];
    EXPECT_TRUE(sub || tree) << profile << ": neither urdf.sub_models nor urdf.tree_models has "
                             << name;
    is_sub.push_back(static_cast<bool>(sub));
    if (sub) {
      const auto declared = std::find_if(cfg.sub_models.begin(), cfg.sub_models.end(),
                                         [&name](const auto& s) { return s.name == name; });
      if (declared == cfg.sub_models.end()) {
        cfg.sub_models.push_back(
            {name, sub["root_link"].as<std::string>(), sub["tip_link"].as<std::string>()});
      }
    } else if (tree) {
      const auto declared = std::find_if(cfg.tree_models.begin(), cfg.tree_models.end(),
                                         [&name](const auto& t) { return t.name == name; });
      if (declared == cfg.tree_models.end()) {
        cfg.tree_models.push_back({name, tree["root_link"].as<std::string>(),
                                   tree["tip_links"].as<std::vector<std::string>>()});
      }
    }
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
  const YAML::Node device = params["devices"][arm_device];
  const auto joint_names = device["joint_state_names"].as<std::vector<std::string>>();
  const auto max_velocity = device["joint_limits"]["max_velocity"].as<std::vector<double>>();
  const auto max_torque = device["joint_limits"]["max_torque"].as<std::vector<double>>();
  EXPECT_EQ(joint_names.size(), home_device.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    const std::string& name = names[i];
    ShippedArm arm;
    const std::shared_ptr<const pinocchio::Model> model =
        is_sub[i] ? builder.GetReducedModel(name) : builder.GetTreeModel(name);
    EXPECT_TRUE(model) << name;
    if (!model) {
      out.push_back(std::move(arm));
      continue;
    }
    arm.catch_mass = pinocchio::computeTotalMass(*model);
    arm.n_dev = static_cast<int>(joint_names.size());
    const Eigen::Index n = model->nv;
    EXPECT_LE(static_cast<std::size_t>(n), joint_names.size()) << name;
    arm.rig.arm.model = model;
    arm.rig.arm.frame = model->getFrameId("catch_frame");
    arm.rig.arm.name = name;
    arm.rig.arm.q_nominal.resize(n);
    arm.device_of_model.assign(static_cast<std::size_t>(n), -1);
    arm.rig.model = *model;  // no extra armature: the shipped planner adds none
    arm.rig.limits.q_min = model->lowerPositionLimit;
    arm.rig.limits.q_max = model->upperPositionLimit;
    arm.rig.limits.qd_max.resize(n);
    arm.rig.limits.tau_max.resize(n);
    arm.rig.limits.armature = Eigen::VectorXd::Zero(n);
    for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(model->njoints);
         ++jid) {
      const auto it = std::find(joint_names.begin(), joint_names.end(), model->names[jid]);
      EXPECT_NE(it, joint_names.end()) << model->names[jid];
      if (it == joint_names.end()) {
        continue;
      }
      const auto d = static_cast<std::size_t>(std::distance(joint_names.begin(), it));
      const Eigen::Index m = model->joints[jid].idx_v();
      arm.rig.arm.q_nominal[m] = home_device.at(d);
      arm.device_of_model.at(static_cast<std::size_t>(m)) = static_cast<int>(d);
      arm.rig.limits.qd_max[m] = max_velocity.at(d);
      arm.rig.limits.tau_max[m] = max_torque.at(d);
    }
    arm.rig.params = mpc_docking::BaseParams(static_cast<int>(n));
    arm.rig.params.q_nom = arm.rig.arm.q_nominal;
    out.push_back(std::move(arm));
  }
  return out;
}

// The catch sub-model the profile's catching controller names
// (`planner.sub_model`) — one model for the search and the segments, every
// joint of the arm device.
inline ShippedArm LoadShippedArm(const std::string& profile, const std::string& yaml,
                                 const rtc_urdf_bridge::ModelConfig& cfg,
                                 const std::string& arm_device,
                                 const std::vector<double>& home_device) {
  const YAML::Node controller = ShippedControllerNode(profile, "demo_catching_controller");
  const std::string sub_model = rtc::catching::ParsePlannerParams(controller["catching"]).sub_model;
  EXPECT_FALSE(sub_model.empty());
  std::vector<ShippedArm> models =
      LoadShippedModels(profile, yaml, cfg, {sub_model}, arm_device, home_device);
  if (models.empty()) {
    return ShippedArm{};
  }
  if (models.front().rig.arm.model) {
    EXPECT_EQ(static_cast<std::size_t>(models.front().rig.arm.model->nv), home_device.size());
  }
  return std::move(models.front());
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
