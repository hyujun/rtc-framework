// mpc_docking numeric core on the SHIPPED catch sub-models (E1-F13, #739).
//
// rtc_controllers' own suite solves on the plain arm URDFs, because the
// hand-locked sub-models are xacro and only this package builds them. The
// hand's mass enters the torque rows, so "does it converge under a short lead
// / a real throw" is recorded here on the models the planner will actually
// hand the core: `ur5e_catch` and `iiwa7_catch`, each with its shipped catch
// frame and the shipped joint ratings.
//
// RECORDED, not judged: the capture-set parameters are the synthetic ones of
// the core's fixture (the identified values are E1-F15's), so a verdict here
// would be a verdict on made-up numbers. What IS asserted is only that the
// models are the shipped ones (name, frame, joint count, hand mass present)
// and that every solve returns a result.
#include "iiwa7_leap_test_fixture.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace dk = rtc::testing::mpc_docking;
namespace fx = rtc::testing::mpc_segment_core;

struct ShippedArm {
  dk::Rig rig;
  double catch_mass{0.0};  // total mass of the sub-model the core computes on
};

// The profile's node parameters (`/**: ros__parameters:`), as shipped.
YAML::Node ShippedNodeParameters(const std::string& relative) {
  const YAML::Node root = YAML::LoadFile(std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + relative);
  EXPECT_TRUE(root.IsMap() && root.size() >= 1) << relative;
  return root.begin()->second["ros__parameters"];
}

// The catch sub-model exactly as the shipped profile declares it: the
// sub-model's root and tip, the catch frame's parent, offset and rotation, the
// arm device's joint names and ratings — all read from the shipped YAML, so a
// change there moves this test with it.
ShippedArm LoadShippedArm(const std::string& profile, const std::string& yaml,
                          rtc_urdf_bridge::ModelConfig cfg, const std::string& arm_device,
                          const std::vector<double>& home_device) {
  ShippedArm out;
  const YAML::Node params = ShippedNodeParameters(profile + "/" + yaml);
  const YAML::Node controller =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
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
    out.rig.limits.qd_max[m] = max_velocity.at(d);
    out.rig.limits.tau_max[m] = max_torque.at(d);
  }
  out.rig.params = dk::BaseParams(static_cast<int>(n));
  out.rig.params.q_nom = out.rig.arm.q_nominal;
  return out;
}

std::vector<ShippedArm> ShippedArms() {
  using integrated_bringup::testfx::kArmHome;
  using integrated_bringup::testfx::kUr5eHome;
  std::vector<ShippedArm> arms;
  arms.push_back(LoadShippedArm("ur5e_p1b", "_base.yaml",
                                integrated_bringup::testfx::MakeUr5eP1bModelConfig(), "ur5e",
                                std::vector<double>(kUr5eHome.begin(), kUr5eHome.end())));
  arms.push_back(LoadShippedArm("iiwa7_leap", "sim.yaml",
                                integrated_bringup::testfx::MakeIiwa7LeapModelConfig(), "iiwa7",
                                std::vector<double>(kArmHome.begin(), kArmHome.end())));
  return arms;
}

TEST(DockingCoreShipped, SubModelsAreTheShippedHandLockedArms) {
  const std::vector<ShippedArm> arms = ShippedArms();
  ASSERT_EQ(arms.size(), 2U);
  EXPECT_EQ(arms[0].rig.arm.name, "ur5e_catch");
  EXPECT_EQ(arms[0].rig.model.nv, 6);
  EXPECT_EQ(arms[1].rig.arm.name, "iiwa7_catch");
  EXPECT_EQ(arms[1].rig.model.nv, 7);
  // The hand is IN the model the torque rows are computed on: the sub-model
  // locks the hand's joints, it does not drop the hand. Compared with the
  // plain arm URDF the core's own suite uses (a reduced `ur5e` / `iiwa7`
  // sub-model would not do — it carries the locked hand too).
  const std::array<double, 2> plain_arm_mass{pinocchio::computeTotalMass(*fx::RealArm6().model),
                                             pinocchio::computeTotalMass(*fx::RealArm7().model)};
  for (std::size_t i = 0; i < arms.size(); ++i) {
    const ShippedArm& a = arms[i];
    ASSERT_TRUE(a.rig.arm.model);
    EXPECT_LT(a.rig.arm.frame, a.rig.model.frames.size()) << a.rig.arm.name;
    EXPECT_GT(a.catch_mass, plain_arm_mass[i] + 0.1) << a.rig.arm.name;
    EXPECT_GT(a.rig.limits.tau_max.minCoeff(), 0.0);
    EXPECT_GT(a.rig.limits.qd_max.minCoeff(), 0.0);
    ::testing::Test::RecordProperty(a.rig.arm.name + "_mass_over_plain_arm_g",
                                    static_cast<int>((a.catch_mass - plain_arm_mass[i]) * 1e3));
  }
}

// A short lead, on the grids the plain-arm suite records: one coarse
// pre-catch interval, two, and four fine ones, before the shipped stop.
TEST(DockingCoreShipped, RecordsShortLeadGrids) {
  struct Grid {
    const char* tag;
    int n_pre;
    double dt_pre;
  };

  for (ShippedArm& a : ShippedArms()) {
    ASSERT_TRUE(a.rig.arm.model);
    for (const Grid& g : {Grid{"lead_0p10_x1", 1, 0.1}, Grid{"lead_0p10_x2", 2, 0.1},
                          Grid{"lead_0p04_x4", 4, 0.04}, Grid{"lead_0p05_x4", 4, 0.05}}) {
      dk::Rig rig = a.rig;
      dk::SetShortLeadGrid(rig.params, g.n_pre, g.dt_pre);
      const dk::SolveTally tally = dk::SolveGeneratedThrows(rig, 20);
      EXPECT_EQ(tally.cases, 20) << a.rig.arm.name << " " << g.tag;
      dk::RecordTally(a.rig.arm.name + "_" + g.tag, tally);
    }
  }
}

// The fixture's 0.3 s approach, for reference against the plain arms.
TEST(DockingCoreShipped, RecordsTheBaseGrid) {
  for (const ShippedArm& a : ShippedArms()) {
    ASSERT_TRUE(a.rig.arm.model);
    const dk::SolveTally tally = dk::SolveGeneratedThrows(a.rig, 20);
    EXPECT_EQ(tally.cases, 20) << a.rig.arm.name;
    dk::RecordTally(a.rig.arm.name + "_lead_0p05_x6", tally);
  }
}

// A real throw: 4–5 m/s in the world, a closing speed of at most 1.0 m/s
// allowed at the catch.
TEST(DockingCoreShipped, RecordsRealThrowCondition) {
  for (const ShippedArm& a : ShippedArms()) {
    ASSERT_TRUE(a.rig.arm.model);
    dk::Rig rig = a.rig;
    rig.params.c_cap_max = 1.0;
    rig.params.c_ent_max = 1.0;
    const dk::SolveTally tally = dk::SolveRealThrows(rig, 4.0, 5.0, 10);
    EXPECT_EQ(tally.cases, 10) << a.rig.arm.name;
    dk::RecordTally(a.rig.arm.name + "_real_throw_4p0_to_5p0", tally);
  }
}

}  // namespace
