/// @file test_mpc_layout_alloc_gate.cpp
/// @brief RT-path allocation gate for the whole-body layout and the solution
///        manager tick: `WbLayout::StateDifference` on a free-flyer model and
///        one `MPCSolutionManager::ComputeReference` tick (both the tick that
///        acquires a fresh plan and a steady one) and the state handoff
///        (`WriteState` on the RT side, `ReadState` on the MPC side) must
///        make no C-level heap allocation. The malloc gate replaces the allocator for the whole
///        executable, so pinocchio's own allocations count too; the positive
///        control proves the gate fires on exactly such a library allocation.
///
/// This header may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "test_utils/panda_fixture.hpp"

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <filesystem>

namespace rtc::mpc {
namespace {

using test_utils::kPandaUrdf;

TEST(MpcLayoutAllocGate, PositiveControlLibraryAllocationIsCounted) {
  if (!std::filesystem::exists(kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
  }
  const pinocchio::Model model = test_utils::BuildPandaFreeFlyer();
  std::size_t seen = 0;
  {
    const rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data data(model);  // allocates inside libpinocchio
    seen = gate.count();
  }
  EXPECT_GT(seen, 0u) << "the gate must observe an allocation made inside a library";
}

TEST(MpcLayoutAllocGate, StateDifferenceOnFreeFlyerAllocatesNothing) {
  if (!std::filesystem::exists(kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
  }
  const pinocchio::Model model = test_utils::BuildPandaFreeFlyer();
  const WbLayout layout = WbLayout::FromModel(model, 0);
  ASSERT_TRUE(layout.floating_base());

  std::srand(5);
  const Eigen::VectorXd q_ref = pinocchio::randomConfiguration(
      model, -Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq));
  const Eigen::VectorXd q_cur = pinocchio::randomConfiguration(
      model, -Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq));
  const Eigen::VectorXd v_ref = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd v_cur = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd dx(layout.ndx);

  // Warm once (first-use statics), then measure.
  ASSERT_TRUE(layout.StateDifference(model, q_ref, v_ref, q_cur, v_cur, dx));
  std::size_t seen = 0;
  bool ok = false;
  {
    const rtc::testing::ScopedMallocGate gate;
    ok = layout.StateDifference(model, q_ref, v_ref, q_cur, v_cur, dx);
    seen = gate.count();
  }
  EXPECT_TRUE(ok);
  EXPECT_EQ(seen, 0u) << "StateDifference is RT-path code";
}

/// A small hand-made fixed-base plan with an identity gain at node 0.
MPCSolution MakePlan(int nq, int nv, int horizon) {
  MPCSolution sol{};
  sol.horizon_length = horizon;
  sol.dt_node = 0.01;
  MarkSolved(sol, true);
  sol.nq = nq;
  sol.nv = nv;
  sol.nu = nv;
  sol.nx = nq + nv;
  sol.ndx = 2 * nv;
  sol.n_contact_vars = 0;
  sol.gain_nodes = 1;
  for (int k = 0; k <= horizon; ++k) {
    for (int i = 0; i < nq; ++i) {
      sol.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = 0.01 * k;
    }
    for (int i = 0; i < nv; ++i) {
      sol.v_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = 1.0;
    }
  }
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(nv, 2 * nv);
  K.topLeftCorner(nv, nv).setIdentity();
  EXPECT_TRUE(PackRiccatiGainRowMajor(K, sol.K_riccati[0].data(), sol.K_riccati[0].size()));
  return sol;
}

TEST(MpcLayoutAllocGate, FixedBaseComputeReferenceTickAllocatesNothing) {
  constexpr int kNq = 4;
  constexpr int kNv = 4;
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 100;

  MPCSolutionManager mgr;
  mgr.Init(cfg, WbLayout::FixedBase(kNq, kNv, 0));
  const MPCSolution plan = MakePlan(kNq, kNv, 10);

  const Eigen::VectorXd q = Eigen::VectorXd::Constant(kNq, 0.1);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  Eigen::VectorXd q_ref(kNq), v_ref(kNv), a_ff(kNv), u_fb(kNv);
  Eigen::VectorXd lambda_ref(1);
  InterpMeta meta{};

  // Warm: one publish / acquire cycle outside the gate.
  mgr.PublishSolution(plan);
  ASSERT_TRUE(mgr.ComputeReference(q, v, 0, q_ref, v_ref, a_ff, lambda_ref, u_fb, meta));

  // Tick that acquires a fresh plan (copy + gain install) and a steady tick.
  mgr.PublishSolution(plan);
  std::size_t seen_fresh = 0;
  std::size_t seen_steady = 0;
  bool ok_fresh = false;
  bool ok_steady = false;
  {
    const rtc::testing::ScopedMallocGate gate;
    ok_fresh = mgr.ComputeReference(q, v, 1'000'000, q_ref, v_ref, a_ff, lambda_ref, u_fb, meta);
    seen_fresh = gate.count();
  }
  {
    const rtc::testing::ScopedMallocGate gate;
    ok_steady = mgr.ComputeReference(q, v, 2'000'000, q_ref, v_ref, a_ff, lambda_ref, u_fb, meta);
    seen_steady = gate.count();
  }
  EXPECT_TRUE(ok_fresh);
  EXPECT_TRUE(ok_steady);
  EXPECT_EQ(seen_fresh, 0u) << "acquiring a plan on the RT tick must not allocate";
  EXPECT_EQ(seen_steady, 0u) << "a steady RT tick must not allocate";
  EXPECT_NE(u_fb(0), 0.0) << "the identity gain feeds the position error back";
}

TEST(MpcLayoutAllocGate, TheStateHandoffAllocatesNothing) {
  constexpr int kNq = 7;
  constexpr int kNv = 6;
  YAML::Node cfg;
  cfg["enabled"] = true;
  MPCSolutionManager mgr;
  mgr.Init(cfg, WbLayout::FixedBase(kNq, kNv, 0));
  const Eigen::VectorXd q = Eigen::VectorXd::Constant(kNq, 0.1);
  const Eigen::VectorXd v = Eigen::VectorXd::Constant(kNv, 0.2);
  mgr.SetStateEpoch(3);
  MPCStateSnapshot read{};
  std::size_t seen = 0;
  {
    const rtc::testing::ScopedMallocGate gate;
    mgr.WriteState(q, v, 123, 456);
    read = mgr.ReadState();
    seen = gate.count();
  }
  EXPECT_EQ(seen, 0u) << "the state handoff runs on the RT and the MPC tick";
  EXPECT_TRUE(read.valid);
  EXPECT_EQ(read.source_sequence, 456u);
  EXPECT_EQ(read.state_epoch, 3u);
}

}  // namespace
}  // namespace rtc::mpc
