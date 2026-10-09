// ── The reach bound on the shipped catch sub-models (L3 §4.1) ────────────────
//
// The searches refuse a candidate whose IK target is farther from the arm than
// the bound, without running the IK. That is only sound if the bound is a
// NECESSARY condition on the models the planner is actually handed:
//
//   1. no posture inside the joint limits puts the catch frame outside it;
//   2. a target the filter refuses is one the shipped catch-pose IK refuses
//      too — with the grid search's options and with the NLP search's, seeded
//      on the shipped wait pose as both searches seed it;
//   3. it is not idle: some posture comes within a few percent of the radius.
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/reach_bound.hpp"
#include "rtc_controllers/testing/reach_bound_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "shipped_catch_arm_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/data.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <random>
#include <string>
#include <vector>

namespace {

using integrated_bringup::testfx::LoadShippedArm;
using integrated_bringup::testfx::ShippedArm;
using integrated_bringup::testfx::ShippedControllerNode;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::ComputeReachBound;
using rtc::catching::ReachBound;
using rtc::catching::WithinReach;

struct Profile {
  std::string name;
  ShippedArm arm;
  YAML::Node catching;
};

std::vector<Profile> Profiles() {
  namespace fx = integrated_bringup::testfx;
  std::vector<Profile> out;
  out.push_back({"ur5e_p1b",
                 LoadShippedArm("ur5e_p1b", "_base.yaml", fx::MakeUr5eP1bModelConfig(), "ur5e",
                                std::vector<double>(fx::kUr5eHome.begin(), fx::kUr5eHome.end())),
                 ShippedControllerNode("ur5e_p1b", "demo_catching_controller")["catching"]});
  out.push_back({"iiwa7_leap",
                 LoadShippedArm("iiwa7_leap", "sim.yaml", fx::MakeIiwa7LeapModelConfig(), "iiwa7",
                                std::vector<double>(fx::kArmHome.begin(), fx::kArmHome.end())),
                 ShippedControllerNode("iiwa7_leap", "demo_catching_controller")["catching"]});
  return out;
}

// FK rounding only: the bound carries no margin of its own.
constexpr double kSlack = 1e-9;

TEST(CatchingReachBoundShipped, NoPostureInsideTheLimitsLeavesItAndSomeComeClose) {
  for (Profile& p : Profiles()) {
    SCOPED_TRACE(p.name);
    const pinocchio::Model& model = *p.arm.rig.arm.model;
    const pinocchio::FrameIndex frame = p.arm.rig.arm.frame;
    const ReachBound bound = ComputeReachBound(model, frame);
    ASSERT_TRUE(bound.Bounded()) << bound.unbounded_by;
    EXPECT_EQ(bound.joints, model.nv);
    RecordProperty("radius_um_" + p.name, static_cast<int>(bound.radius * 1e6));

    pinocchio::Data data(model);
    std::mt19937 rng(7933U);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const Eigen::VectorXd lo = model.lowerPositionLimit;
    const Eigen::VectorXd hi = model.upperPositionLimit;
    Eigen::VectorXd q(model.nq);
    double farthest = 0.0;
    for (int i = 0; i < 100000; ++i) {
      for (Eigen::Index j = 0; j < model.nq; ++j) {
        q[j] = lo[j] + (hi[j] - lo[j]) * unit(rng);
      }
      pinocchio::framesForwardKinematics(model, data, q);
      const Eigen::Vector3d at = data.oMf[frame].translation();
      ASSERT_TRUE(WithinReach(bound, at, kSlack))
          << "posture " << q.transpose() << " puts the catch frame "
          << (at - bound.centre).norm() - bound.radius << " m past the bound";
      farthest = std::max(farthest, (at - bound.centre).norm());
    }
    RecordProperty("farthest_um_" + p.name, static_cast<int>(farthest * 1e6));
    EXPECT_GT(farthest, 0.9 * bound.radius);
  }
}

TEST(CatchingReachBoundShipped, ATargetTheFilterRefusesIsOneTheShippedIkRefuses) {
  for (Profile& p : Profiles()) {
    const std::shared_ptr<const pinocchio::Model> model = p.arm.rig.arm.model;
    const pinocchio::FrameIndex frame = p.arm.rig.arm.frame;
    const int nv = model->nv;
    const ReachBound bound = ComputeReachBound(*model, frame);
    ASSERT_TRUE(bound.Bounded());
    // A handle of its own, without a device joint order (the IK refuses one).
    rtc_urdf_bridge::RtModelHandle handle(model);
    // The seed both searches start the IK from: the shipped wait pose, in
    // model order.
    const rtc::catching::PlannerParams planner = rtc::catching::ParsePlannerParams(p.catching);
    ASSERT_EQ(planner.wait_pose_n, nv);
    Eigen::VectorXd seed(nv);
    for (int m = 0; m < nv; ++m) {
      seed[m] = planner.wait_pose[static_cast<std::size_t>(
          p.arm.device_of_model[static_cast<std::size_t>(m)])];
    }
    for (const char* search : {"grid", "nlp"}) {
      SCOPED_TRACE(p.name + " / " + search);
      const CatchPoseIkOptions options =
          rtc::catching::ParseCatchPoseIkParams(p.catching, nullptr, search).options;
      rtc::testing::ExpectTheIkRefusesWhatTheFilterRefuses(
          handle, frame, bound, options, /*targets=*/200, /*ball_speed=*/4.0, 7934U,
          [&seed](std::mt19937&) { return seed; });
    }
  }
}

}  // namespace
