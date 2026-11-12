/// @file test_stage_reference_alloc_gate.cpp
/// @brief StageReferenceBuilder::Build runs on the MPC thread (SCHED_FIFO,
///        RT path): from the first Build after Init on — forward kinematics,
///        the swing plans, the knots, the slot swap — it must make no heap
///        allocation, and neither must GaitScheduler::Advance beside it. Two
///        gates in one TU (C-level malloc, operator new); three positive
///        controls prove they fire on library allocations: YAML::Load,
///        pinocchio::Data, and Init itself inside the gate.
///
/// malloc_gate.hpp may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"
#include "test_utils/loco_reference_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>

namespace rtc::mpc {
namespace {

namespace tu = test_utils;
using tu::AllocCounter;

TEST(StageReferenceAllocGate, PositiveControlsAreCounted) {
  if (!std::filesystem::exists(tu::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  const pinocchio::Model model = tu::BuildLocoPanda();
  RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, YAML::Load(tu::kLocoHandlerYaml)), RobotModelInitError::kNoError);
  ReferenceConfig cfg;
  const std::vector<double> qd = tu::RandomInsideLimits(model, 1U, 0.1);
  ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(model, qd, false, {}),
                                tu::ActuatedJointNames(model), cfg),
            ReferenceError::kNoError);
  std::size_t m_yaml = 0, m_data = 0, m_init = 0;
  std::int64_t n_yaml = 0, n_data = 0, n_init = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    const YAML::Node n = YAML::Load(tu::kLocoGaitYaml);
    m_yaml = gate.count();
  }
  n_yaml = AllocCounter::AllocCount();
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data data(model);
    m_data = gate.count();
  }
  n_data = AllocCounter::AllocCount();
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    StageReferenceBuilder b;
    EXPECT_EQ(b.Init(handler, cfg, tu::ShippedSwing(), FlatSurface(0.0), 51, 0.02),
              ReferenceError::kNoError);
    m_init = gate.count();
  }
  n_init = AllocCounter::AllocCount();
  AllocCounter::Disarm();
  EXPECT_GT(m_yaml, 0U);
  EXPECT_GT(n_yaml, 0);
  EXPECT_GT(m_data, 0U);
  EXPECT_GT(m_init, 0U);
  EXPECT_GT(n_init, 0);
  (void)n_data;  // pinocchio::Data allocates through malloc (Eigen aligned), new may be 0
}

TEST(StageReferenceAllocGate, AThousandTicksOfAdvanceAndBuildAllocateNothing) {
  if (!std::filesystem::exists(tu::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  const pinocchio::Model model = tu::BuildLocoPanda();
  RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, YAML::Load(tu::kLocoHandlerYaml)), RobotModelInitError::kNoError);
  const std::vector<double> qd = tu::RandomInsideLimits(model, 2U, 0.1);
  ReferenceConfig cfg;
  ASSERT_EQ(LoadReferenceConfig(
                tu::ReferenceYaml(model, qd, true, {{"panda_joint2", 1.0}, {"panda_joint4", -1.0}}),
                tu::ActuatedJointNames(model), cfg),
            ReferenceError::kNoError);
  StageReferenceBuilder b;
  ASSERT_EQ(b.Init(handler, cfg, tu::ShippedSwing(), FlatSurface(0.0), 51, 0.02),
            ReferenceError::kNoError);
  GaitScheduler s;
  const GaitTable table = tu::LocoGaitTable();
  ASSERT_EQ(s.Init(table, 1.0), GaitSchedulerError::kNoError);
  s.SetPolicyEnabled(false);
  const MPCStateSnapshot snap =
      tu::RegisteredSnapshot(model, tu::RandomInsideLimits(model, 3U, 0.05));
  MPCStateSnapshot bad = snap;
  bad.q[9] = std::numeric_limits<double>::quiet_NaN();
  FilteredCommand bad_cmd = tu::RegisteredCommand();
  bad_cmd.v_x = std::numeric_limits<double>::infinity();

  int errors = 0;
  int flight_nodes = 0;
  double sink = 0.0;
  std::size_t malloc_count = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    for (int k = 0; k < 1000; ++k) {
      const double t = 100.0 + 0.02 * k;
      // stance → walk (k 50) → hop with flight (k 400) → walk (k 700); an
      // invalid-command stretch in the middle; the failure set every 100 ticks.
      if (k == 50 || k == 700) {
        s.RequestGait(1);
      } else if (k == 400) {
        s.RequestGait(2);
      }
      const FilteredCommand cmd = tu::RegisteredCommand(!(k >= 550 && k < 600));
      errors += s.Advance(t, cmd, PlanarVelocity{}) != GaitSchedulerError::kNoError;
      errors += b.Build(s, snap, cmd) != ReferenceError::kNoError;
      const HorizonReference& h = b.active();
      sink += h.stages[0].q[0] + h.stages[h.n_stages - 1].contact_pose[2];
      for (std::uint32_t i = 0; i < h.n_stages; ++i) {
        flight_nodes += h.stages[i].mode == 0;
      }
      if (k % 100 == 0) {
        errors += b.Build(s, bad, cmd) != ReferenceError::kNonFiniteInput;
        errors += b.Build(s, snap, bad_cmd) != ReferenceError::kNonFiniteInput;
      }
    }
    malloc_count = gate.count();
  }
  AllocCounter::Disarm();
  EXPECT_EQ(malloc_count, 0U);
  EXPECT_EQ(AllocCounter::AllocCount(), 0);
  EXPECT_EQ(errors, 0);
  EXPECT_GT(flight_nodes, 0);
  EXPECT_TRUE(std::isfinite(sink));
  EXPECT_EQ(b.active().reference_id, 1000U);
}

}  // namespace
}  // namespace rtc::mpc
