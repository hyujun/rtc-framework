/// @file test_locomotion_phase_manager_alloc_gate.cpp
/// @brief LocomotionPhaseManager::Update runs on the MPC thread (SCHED_FIFO,
///        RT path) and SetCommand on the command filter's thread: from the
///        first Update after Load on — the command slot, the scheduler, the
///        builder, the context returned by value — neither may make a heap
///        allocation. Two gates in one TU (C-level malloc, operator new); the
///        positive controls prove they fire on a library allocation (YAML::Load)
///        and on Load itself.
///
/// malloc_gate.hpp may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/phase/locomotion_phase_manager.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"
#include "test_utils/shipped_walk_fixture.hpp"

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

class LocomotionPhaseManagerAllocGate : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)), RobotModelInitError::kNoError);
    stand_ = tu::G1Stand(model_);
  }

  pinocchio::Model model_;
  RobotModelHandler handler_;
  Eigen::VectorXd stand_;
};

TEST_F(LocomotionPhaseManagerAllocGate, PositiveControlsAreCounted) {
  const YAML::Node cfg = tu::ShippedLocomotionYaml();
  std::size_t m_yaml = 0;
  std::size_t m_load = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    const YAML::Node n = YAML::Load(tu::kShippedGaitYaml);
    m_yaml = gate.count();
  }
  const std::int64_t n_yaml = AllocCounter::AllocCount();
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
    EXPECT_EQ(m.Load(cfg), LocomotionPhaseError::kNoError);
    m_load = gate.count();
  }
  const std::int64_t n_load = AllocCounter::AllocCount();
  AllocCounter::Disarm();
  EXPECT_GT(m_yaml, 0U);
  EXPECT_GT(n_yaml, 0);
  EXPECT_GT(m_load, 0U);
  EXPECT_GT(n_load, 0);
}

TEST_F(LocomotionPhaseManagerAllocGate, UpdateAndSetCommandAllocateNothing) {
  LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  ASSERT_EQ(m.Load(tu::ShippedLocomotionYaml()), LocomotionPhaseError::kNoError);
  PhaseManagerBase& base = m;
  MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  MPCStateSnapshot bad = snap;
  bad.q[9] = std::numeric_limits<double>::quiet_NaN();
  const Eigen::VectorXd q = stand_;
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  const Eigen::VectorXd sensor;
  const pinocchio::SE3 tcp = pinocchio::SE3::Identity();
  FilteredCommand cmd = tu::RegisteredCommand();
  cmd.base_height = 0.7925;

  int ok = 0;
  int refused = 0;
  double sink = 0.0;
  std::size_t malloc_count = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    for (int k = 0; k < 1000; ++k) {
      const double t = 10.0 + 0.035 * k;
      // stance → walk (k 30) → policy back on (k 600); a reset at k 800.
      if (k == 30) {
        m.RequestGait(2);
      } else if (k == 600) {
        m.SetPolicyEnabled(true);
      } else if (k == 800) {
        m.RequestReset();
      }
      cmd.sequence = static_cast<std::uint64_t>(k + 1);
      cmd.valid = !(k >= 300 && k < 350);
      m.SetCommand(cmd);
      snap.sequence = static_cast<std::uint64_t>(k);
      const PhaseContext ctx = base.Update(snap, q, v, sensor, tcp, t);
      if (ctx.horizon != nullptr) {
        ++ok;
        sink += ctx.horizon->stages[0].q[2] + ctx.cost_config.dt;
      }
      if (k % 100 == 0) {
        // The refused ticks: a non-finite snapshot, the overload without one.
        refused += base.Update(bad, q, v, sensor, tcp, t + 1e-3).horizon == nullptr;
        refused += base.Update(q, v, sensor, tcp, t + 2e-3).horizon == nullptr;
      }
    }
    malloc_count = gate.count();
  }
  AllocCounter::Disarm();
  EXPECT_EQ(malloc_count, 0U);
  EXPECT_EQ(AllocCounter::AllocCount(), 0);
  EXPECT_EQ(ok, 1000);
  EXPECT_EQ(refused, 20);
  EXPECT_TRUE(std::isfinite(sink));
  EXPECT_EQ(m.command().sequence, 1000U);
}

}  // namespace
}  // namespace rtc::mpc
