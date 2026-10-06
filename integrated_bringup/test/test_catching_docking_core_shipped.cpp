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
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "shipped_catch_arm_fixture.hpp"

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

using integrated_bringup::testfx::ShippedArm;
using integrated_bringup::testfx::ShippedArms;

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
