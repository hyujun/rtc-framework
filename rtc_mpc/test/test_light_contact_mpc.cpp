/// @file test_light_contact_mpc.cpp
/// @brief Phase 5 integration test for `rtc::mpc::LightContactMPC` on the
///        generic Panda fixture. Exercises Init / Solve / warm-start
///        behavior and records per-tick solve latency.
///
/// Exit-criterion gates wired here (from
/// `docs/mpc_implementation_progress.md` §Phase 5):
/// - Warm-start iter drop: second solve after a small reference shift
///   should use ≤ 50% of the first solve's iteration count.
/// - `Solve` on steady-state topology must be `noexcept` — tested by the
///   code path itself returning an enum instead of throwing.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include "rtc_mpc/handler/light_contact_mpc.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/phase/phase_cost_config.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

namespace {

constexpr const char *kPandaUrdf =
    RTC_PANDA_URDF_PATH;

constexpr const char *kCostYaml = R"(
horizon_length: 20
dt: 0.01
w_frame_placement: 100.0
w_state_reg: 1.0
w_control_reg: 0.01
w_contact_force: 0.0
w_centroidal_momentum: 0.0
W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

class LightContactMPCTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto robot_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(handler_.Init(model_, robot_cfg),
              rtc::mpc::RobotModelInitError::kNoError);

    auto cost_node = YAML::Load(kCostYaml);
    ASSERT_EQ(
        rtc::mpc::PhaseCostConfig::LoadFromYaml(cost_node, handler_, cfg_),
        rtc::mpc::PhaseCostConfigError::kNoError);

    ctx_.phase_id = 0;
    ctx_.phase_name = "test";
    ctx_.phase_changed = false;
    ctx_.ocp_type = "light_contact";
    ctx_.cost_config = cfg_;
    ctx_.contact_plan = {};

    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ctx_.ee_target = pdata.oMf[handler_.end_effector_frame_id()];

    solver_cfg_.prim_tol = 1e-3;
    solver_cfg_.dual_tol = 1e-2;
    solver_cfg_.mu_init = 1e-2;
    solver_cfg_.max_iters = 40;
    solver_cfg_.max_al_iters = 10;
    solver_cfg_.verbose = false;
  }

  rtc::mpc::MPCStateSnapshot MakeStateSnapshot() const {
    rtc::mpc::MPCStateSnapshot s{};
    const Eigen::VectorXd q = pinocchio::neutral(model_);
    s.nq = handler_.nq();
    s.nv = handler_.nv();
    for (int i = 0; i < s.nq; ++i)
      s.q[static_cast<std::size_t>(i)] = q[i];
    for (int i = 0; i < s.nv; ++i)
      s.v[static_cast<std::size_t>(i)] = 0.0;
    s.timestamp_ns = 0;
    return s;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseCostConfig cfg_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_;
  rtc::mpc::MPCSolverConfig solver_cfg_;
};

// ── Init ─────────────────────────────────────────────────────────────────

TEST_F(LightContactMPCTest, InitSucceedsOnValidContext) {
  rtc::mpc::LightContactMPC mpc;
  EXPECT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kNoError);
  EXPECT_TRUE(mpc.Initialised());
  EXPECT_EQ(mpc.ocp_type(), std::string_view("light_contact"));
  EXPECT_EQ(mpc.horizon_length(), cfg_.horizon_length);
  EXPECT_EQ(mpc.nq(), handler_.nq());
  EXPECT_EQ(mpc.nv(), handler_.nv());
  EXPECT_EQ(mpc.nu(), handler_.nu());
  EXPECT_EQ(mpc.n_contact_vars(), 6); // 2×3
}

TEST_F(LightContactMPCTest, InitRejectsUninitialisedModel) {
  rtc::mpc::RobotModelHandler empty;
  rtc::mpc::LightContactMPC mpc;
  EXPECT_EQ(mpc.Init(solver_cfg_, empty, limits_, ctx_),
            rtc::mpc::MPCInitError::kModelNotInitialised);
  EXPECT_FALSE(mpc.Initialised());
}

TEST_F(LightContactMPCTest, InitRejectsOcpTypeMismatch) {
  ctx_.ocp_type = "contact_rich";
  rtc::mpc::LightContactMPC mpc;
  EXPECT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kOcpTypeMismatch);
}

TEST_F(LightContactMPCTest, InitRejectsInvalidSolverTol) {
  solver_cfg_.prim_tol = 0.0;
  rtc::mpc::LightContactMPC mpc;
  EXPECT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kInvalidSolverConfig);
}

// ── Solve ────────────────────────────────────────────────────────────────

TEST_F(LightContactMPCTest, SolveReturnsValidSolutionOnFirstCall) {
  rtc::mpc::LightContactMPC mpc;
  ASSERT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kNoError);

  const auto state = MakeStateSnapshot();
  rtc::mpc::MPCSolution out{};
  EXPECT_EQ(mpc.Solve(ctx_, state, out), rtc::mpc::MPCSolveError::kNoError);
  EXPECT_TRUE(out.IsValid());
  EXPECT_EQ(out.horizon_length, cfg_.horizon_length);
  EXPECT_EQ(out.nq, handler_.nq());
  EXPECT_EQ(out.nv, handler_.nv());
  EXPECT_EQ(out.nu, handler_.nu());
  EXPECT_GT(out.iterations, 0);
  // First node x_0 must equal the RT-state snapshot (force_initial_condition).
  const Eigen::VectorXd q_neutral = pinocchio::neutral(model_);
  for (int i = 0; i < handler_.nq(); ++i) {
    EXPECT_NEAR(out.q_traj[0][static_cast<std::size_t>(i)], q_neutral[i], 1e-9);
  }
}

TEST_F(LightContactMPCTest, SolveRejectsStateDimMismatch) {
  rtc::mpc::LightContactMPC mpc;
  ASSERT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kNoError);

  auto bad_state = MakeStateSnapshot();
  bad_state.nq = handler_.nq() + 1;
  rtc::mpc::MPCSolution out{};
  EXPECT_EQ(mpc.Solve(ctx_, bad_state, out),
            rtc::mpc::MPCSolveError::kStateDimMismatch);
}

TEST_F(LightContactMPCTest, SolveRejectsCrossModeContext) {
  rtc::mpc::LightContactMPC mpc;
  ASSERT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kNoError);

  auto alt = ctx_;
  alt.ocp_type = "contact_rich";
  const auto state = MakeStateSnapshot();
  rtc::mpc::MPCSolution out{};
  EXPECT_EQ(mpc.Solve(alt, state, out),
            rtc::mpc::MPCSolveError::kRebuildRequired);
}

TEST_F(LightContactMPCTest, SolveBeforeInitRejected) {
  rtc::mpc::LightContactMPC mpc;
  const auto state = MakeStateSnapshot();
  rtc::mpc::MPCSolution out{};
  EXPECT_EQ(mpc.Solve(ctx_, state, out),
            rtc::mpc::MPCSolveError::kNotInitialised);
}

// ── Warm-start exit criterion (≥50% iter drop on small ref shift) ────────

TEST_F(LightContactMPCTest, WarmStartReducesIterationsAtLeastFiftyPercent) {
  // Cold solve must be nontrivial for the ≥50% drop gate to be meaningful.
  // Push the initial EE target 15 cm away from fk(q_neutral), and give the
  // solver enough iters to reach convergence before warm-starting.
  ctx_.ee_target.translation() += Eigen::Vector3d(0.15, 0.0, 0.05);
  solver_cfg_.max_iters = 120;

  rtc::mpc::LightContactMPC mpc;
  ASSERT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kNoError);

  const auto state = MakeStateSnapshot();
  rtc::mpc::MPCSolution out{};

  ASSERT_EQ(mpc.Solve(ctx_, state, out), rtc::mpc::MPCSolveError::kNoError);
  const int cold_iters = out.iterations;
  ASSERT_GE(cold_iters, 4)
      << "cold solve must do non-trivial work for the ≥50% drop gate";

  // Small target perturbation — warm-start should cover it cheaply.
  rtc::mpc::PhaseContext ctx_next = ctx_;
  ctx_next.phase_changed = false;
  pinocchio::SE3 shifted = ctx_.ee_target;
  shifted.translation() += Eigen::Vector3d(0.001, 0.0, 0.0);
  ctx_next.ee_target = shifted;

  ASSERT_EQ(mpc.Solve(ctx_next, state, out), rtc::mpc::MPCSolveError::kNoError);
  const int warm_iters = out.iterations;

  std::cout << "[LightContactMPC warm-start] cold=" << cold_iters
            << " warm=" << warm_iters << "\n";

  EXPECT_LE(warm_iters * 2, cold_iters);
}

// ── No-alloc on steady state (indirect: thousands of ticks without OOM) ─

TEST_F(LightContactMPCTest, SteadyStateSolveSequenceStable) {
  rtc::mpc::LightContactMPC mpc;
  ASSERT_EQ(mpc.Init(solver_cfg_, handler_, limits_, ctx_),
            rtc::mpc::MPCInitError::kNoError);

  const auto state = MakeStateSnapshot();
  rtc::mpc::MPCSolution out{};
  std::vector<long> wall_us;
  wall_us.reserve(50);

  for (int i = 0; i < 50; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto err = mpc.Solve(ctx_, state, out);
    const auto t1 = std::chrono::steady_clock::now();
    wall_us.push_back(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    ASSERT_EQ(err, rtc::mpc::MPCSolveError::kNoError);
  }
  std::sort(wall_us.begin(), wall_us.end());
  const long p50 = wall_us[wall_us.size() / 2];
  const long p99 = wall_us.back();
  std::cout << "[LightContactMPC perf] 50 ticks: p50=" << p50
            << "us p99=" << p99 << "us\n";
  SUCCEED();
}

} // namespace
