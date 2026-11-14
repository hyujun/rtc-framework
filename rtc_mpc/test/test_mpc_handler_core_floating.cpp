/// @file test_mpc_handler_core_floating.cpp
/// @brief The handler core on a floating base, driven by a kinodynamic stub
///        OCP (`ControlModel::kKinodynamicWrench`) on the public G1: sizes by
///        `nu`, packs the wrench block as `lambda_traj` and the interval-mean
///        acceleration as `a_ff_traj`, packs the node modes and ids, keeps the
///        base quaternions on one hemisphere, starts cold on a new state epoch,
///        carries a warm start of size `nu`, refuses a non-unit base
///        quaternion; and the shipped joint-torque OCPs still refuse a
///        floating model. Zero cold seed; convergence is not asserted.

#include "handler/internal/mpc_handler_core.hpp"
#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/handler/contact_light_mpc.hpp"
#include "rtc_mpc/handler/contact_rich_mpc.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_stub_ocp.hpp"
#include "test_utils/panda_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>

namespace {

using rtc::mpc::MPCInitError;
using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::test_utils::KinodynamicStubOCP;

constexpr int kHorizon = 6;
constexpr double kNodeDt = 0.035;
/// Only contact 1 in contact: contact 0's wrench slots are inert.
constexpr std::uint32_t kMode = 0b10;

class MPCHandlerCoreFloatingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(rtc::mpc::test_utils::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << rtc::mpc::test_utils::kG1Urdf;
    }
    model_ = rtc::mpc::test_utils::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(rtc::mpc::test_utils::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ctx_.ocp_type = std::string{KinodynamicStubOCP::kType};
    ctx_.cost_config.dt = kNodeDt;
    solver_cfg_.prim_tol = 1e-4;
    solver_cfg_.dual_tol = 1e-4;
    solver_cfg_.mu_init = 1e-2;
    solver_cfg_.max_iters = 5;
    solver_cfg_.max_al_iters = 2;

    const Eigen::VectorXd q = pinocchio::neutral(model_);
    state_.nq = model_.nq;
    state_.nv = model_.nv;
    for (int i = 0; i < model_.nq; ++i) {
      state_.q[static_cast<std::size_t>(i)] = q[i];
    }
    state_.q[2] = 0.75;  // pelvis height
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_;
  rtc::mpc::MPCSolverConfig solver_cfg_;
  KinodynamicStubOCP ocp_{kHorizon, kMode};
  rtc::mpc::internal::MPCHandlerCore core_;
  MPCStateSnapshot state_{};
};

TEST_F(MPCHandlerCoreFloatingTest, PacksTheKinodynamicControl) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  const rtc::mpc::WbLayout& layout = handler_.layout();
  EXPECT_EQ(core_.nu(), layout.nu);

  MPCSolution out{};
  for (auto& row : out.lambda_traj) {
    row.fill(7.0);
  }
  for (auto& row : out.a_ff_traj) {
    row.fill(7.0);
  }
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  ASSERT_TRUE(out.IsValid());
  EXPECT_EQ(out.nu, layout.nu);
  EXPECT_EQ(out.nv, model_.nv);
  EXPECT_EQ(out.nq, model_.nq);
  EXPECT_EQ(out.ndx, 2 * model_.nv);
  EXPECT_EQ(out.n_contact_vars, layout.n_wrench_u);
  for (int k = 0; k < out.horizon_length; ++k) {
    const auto ks = static_cast<std::size_t>(k);
    for (int i = 0; i < layout.n_wrench_u; ++i) {
      const auto is = static_cast<std::size_t>(i);
      EXPECT_EQ(out.lambda_traj[ks][is], out.u_traj[ks][is]) << "node " << k << " slot " << i;
    }
    for (int i = 0; i < out.nv; ++i) {
      const auto is = static_cast<std::size_t>(i);
      EXPECT_EQ(out.a_ff_traj[ks][is], (out.v_traj[ks + 1][is] - out.v_traj[ks][is]) / kNodeDt)
          << "node " << k;
    }
  }
}

// A seeded control of size nu reaches the solver: contact 0's slots are
// inert here, so a value seeded there comes back unchanged.
TEST_F(MPCHandlerCoreFloatingTest, WarmStartOfSizeNuIsUsed) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCSolution first{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, first), MPCSolveError::kNoError);
  MPCSolution seed = first;
  for (int k = 0; k < kHorizon; ++k) {
    seed.u_traj[static_cast<std::size_t>(k)][0] = 123.0;
  }
  core_.SeedWarmStart(seed, ocp_);
  MPCSolution out{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  for (int k = 0; k < kHorizon; ++k) {
    EXPECT_EQ(out.u_traj[static_cast<std::size_t>(k)][0], 123.0) << "node " << k;
  }
}

TEST_F(MPCHandlerCoreFloatingTest, RefusesANonUnitBaseQuaternion) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCStateSnapshot bad = state_;
  for (int i = 3; i < 7; ++i) {
    bad.q[static_cast<std::size_t>(i)] *= 1.001;
  }
  MPCSolution out{};
  EXPECT_EQ(core_.Solve(ctx_, bad, ocp_, out), MPCSolveError::kStateDimMismatch);
  EXPECT_FALSE(out.IsValid());
  // Within the layout's tolerance it is accepted as is.
  MPCStateSnapshot close = state_;
  close.q[6] *= 1.0 + 1e-8;
  EXPECT_EQ(core_.Solve(ctx_, close, ocp_, out), MPCSolveError::kNoError);
}

// The horizon's node modes and schedule ids reach the plan, the slot's tail
// past the horizon is cleared, and a context without a horizon ships zeros;
// the snapshot's state epoch and source sequence come back as given.
TEST_F(MPCHandlerCoreFloatingTest, PacksTheNodeModesTheScheduleIdsAndTheStateProvenance) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  rtc::mpc::HorizonReference horizon{};
  horizon.n_stages = kHorizon + 1;
  horizon.schedule_id = 0xFEEDFACECAFEBEEFULL;
  horizon.schedule_epoch = 41;
  for (int k = 0; k <= kHorizon; ++k) {
    horizon.stages[static_cast<std::size_t>(k)].mode = static_cast<std::uint32_t>((k % 4) + 4 * k);
  }
  for (std::size_t k = static_cast<std::size_t>(kHorizon) + 1; k < horizon.stages.size(); ++k) {
    horizon.stages[k].mode = 0xA5A5A5A5u;  // past n_stages: must not be read
  }
  rtc::mpc::PhaseContext ctx = ctx_;
  ctx.horizon = &horizon;
  MPCStateSnapshot state = state_;
  state.state_epoch = 0xC0FFEEu;
  state.source_sequence = 0x1122334455667788ULL;

  MPCSolution out{};
  out.node_mode.fill(0xFFFFFFFFu);
  ASSERT_EQ(core_.Solve(ctx, state, ocp_, out), MPCSolveError::kNoError);
  EXPECT_EQ(out.schedule_id, 0xFEEDFACECAFEBEEFULL);
  EXPECT_EQ(out.schedule_epoch, 41u);
  EXPECT_EQ(out.state_epoch, 0xC0FFEEu);
  EXPECT_EQ(out.observation_source_sequence, 0x1122334455667788ULL);
  for (std::size_t k = 0; k < out.node_mode.size(); ++k) {
    const std::uint32_t want =
        k <= static_cast<std::size_t>(kHorizon) ? horizon.stages[k].mode : 0U;
    EXPECT_EQ(out.node_mode[k], want) << "node " << k;
  }

  // A horizon of fewer stages than the solve's nodes: the nodes past its
  // stages are 0, whatever its unused stages hold.
  horizon.n_stages = kHorizon - 1;
  ASSERT_EQ(core_.Solve(ctx, state, ocp_, out), MPCSolveError::kNoError);
  for (std::size_t k = 0; k < out.node_mode.size(); ++k) {
    const std::uint32_t want =
        k < static_cast<std::size_t>(kHorizon - 1) ? horizon.stages[k].mode : 0U;
    EXPECT_EQ(out.node_mode[k], want) << "node " << k;
  }

  // The reused slot under a context without a horizon: all modes and ids 0.
  // The state epoch changed, so this solve also starts cold.
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  EXPECT_EQ(out.schedule_id, 0u);
  EXPECT_EQ(out.schedule_epoch, 0u);
  EXPECT_EQ(out.state_epoch, 0u);
  EXPECT_EQ(out.observation_source_sequence, 0u);
  for (std::size_t k = 0; k < out.node_mode.size(); ++k) {
    EXPECT_EQ(out.node_mode[k], 0u) << "node " << k;
  }
}

// The kinodynamic feed-forward acceleration is the interval's mean,
// `(v_{k+1} − v_k) / dt`. Under semi-implicit Euler that is the dynamics'
// acceleration at node k — the input's joint accelerations on the joint
// rows, the forward dynamics' base acceleration on the base rows — up to the
// dynamics defect into node k + 1 (the solver's iterate need not be
// feasible).
TEST_F(MPCHandlerCoreFloatingTest, TheKinodynamicAccelerationIsTheIntervalMean) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCSolution out{};
  for (auto& row : out.a_ff_traj) {
    row.fill(7.0);
  }
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  const rtc::mpc::WbLayout& layout = handler_.layout();
  const int nv = out.nv;
  const int nq = out.nq;

  rtc::mpc::KinodynamicDynamics dyn;
  ASSERT_EQ(dyn.Init(handler_), rtc::mpc::KinodynamicError::kNoError);
  auto ode = dyn.MakeOde(kMode);
  ASSERT_TRUE(ode.has_value());
  auto data = ode->createData();
  Eigen::VectorXd x(nq + nv);
  Eigen::VectorXd u(out.nu);

  constexpr double kEps = std::numeric_limits<double>::epsilon();
  bool moved = false;
  for (int k = 0; k < out.horizon_length; ++k) {
    const auto ks = static_cast<std::size_t>(k);
    for (int i = 0; i < nq; ++i) {
      x[i] = out.q_traj[ks][static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < nv; ++i) {
      x[nq + i] = out.v_traj[ks][static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < out.nu; ++i) {
      u[i] = out.u_traj[ks][static_cast<std::size_t>(i)];
    }
    ode->forward(x, u, *data);
    const double defect = out.node_prim_inf[ks + 1] / kNodeDt;
    double v_inf = 1.0;
    for (int i = 0; i < nv; ++i) {
      v_inf = std::max({v_inf, std::abs(out.v_traj[ks][static_cast<std::size_t>(i)]),
                        std::abs(out.v_traj[ks + 1][static_cast<std::size_t>(i)])});
    }
    for (int i = 0; i < nv; ++i) {
      const auto is = static_cast<std::size_t>(i);
      const double a = out.a_ff_traj[ks][is];
      const double dv = out.v_traj[ks + 1][is] - out.v_traj[ks][is];
      ASSERT_TRUE(std::isfinite(a)) << "node " << k << " row " << i;
      // Arithmetic: the shipped value is that difference over dt.
      EXPECT_LE(std::abs(kNodeDt * a - dv), 1e-12 * v_inf) << "node " << k << " row " << i;
      const double model_a = i < layout.n_base_v ? data->xdot_[nv + i]
                                                 : u[layout.acc_u_offset + (i - layout.n_base_v)];
      // The defect and the two accelerations are each rounded once more.
      const double bound =
          defect + 4.0 * kEps * (std::abs(dv) / kNodeDt + std::abs(model_a) + defect);
      EXPECT_LE(std::abs(a - model_a), bound) << "node " << k << " row " << i;
      moved = moved || a != 0.0;
    }
  }
  EXPECT_TRUE(moved) << "a trajectory at rest would not tell the rows apart";
}

// Each node's free-flyer quaternion is put on the hemisphere of the node
// before it — the same rotation as the solver's iterate, its sign the only
// change — and nothing else in the state is touched. The warm start is
// seeded with sign-flipped quaternions so there is something to flip.
TEST_F(MPCHandlerCoreFloatingTest, FreeFlyerQuaternionsShareAHemisphereAlongTheHorizon) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCSolution first{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, first), MPCSolveError::kNoError);
  MPCSolution seed = first;
  for (int k = 2; k <= kHorizon; ++k) {
    for (std::size_t j = 3; j < 7; ++j) {
      seed.q_traj[static_cast<std::size_t>(k)][j] = -seed.q_traj[static_cast<std::size_t>(k)][j];
    }
  }
  core_.SeedWarmStart(seed, ocp_);
  MPCSolution out{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  const auto& xs = core_.solver()->results_.xs;
  int flipped = 0;
  for (int k = 0; k <= kHorizon; ++k) {
    const auto ks = static_cast<std::size_t>(k);
    const Eigen::VectorXd& xk = xs[ks];
    bool same = true;
    bool negated = true;
    for (int j = 3; j < 7; ++j) {
      same = same && out.q_traj[ks][static_cast<std::size_t>(j)] == xk[j];
      negated = negated && out.q_traj[ks][static_cast<std::size_t>(j)] == -xk[j];
    }
    EXPECT_TRUE(same || negated) << "node " << k << ": not ± the iterate";
    flipped += (!same && negated) ? 1 : 0;
    for (int i = 0; i < model_.nq; ++i) {
      if (i < 3 || i >= 7) {
        EXPECT_EQ(out.q_traj[ks][static_cast<std::size_t>(i)], xk[i]) << "node " << k;
      }
    }
    if (k > 0) {
      double dot = 0.0;
      for (std::size_t j = 3; j < 7; ++j) {
        dot += out.q_traj[ks - 1][j] * out.q_traj[ks][j];
      }
      EXPECT_GE(dot, 0.0) << "node " << k;
    }
  }
  EXPECT_EQ(out.q_traj[0][6], state_.q[6]) << "node 0 stays as measured";
  EXPECT_GT(flipped, 0) << "the seed put iterates on the other hemisphere";
}

// A snapshot of another state epoch is not a continuation of the iterates:
// that solve starts cold (no shift); the next one of the same epoch shifts.
TEST_F(MPCHandlerCoreFloatingTest, AStateEpochChangeStartsTheSolveCold) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCSolution out{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  EXPECT_TRUE(out.cold_start) << "the first solve after the build";
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  EXPECT_FALSE(out.cold_start) << "pass control: same epoch, warm";
  EXPECT_EQ(core_.LastNodeShift(), 1);

  MPCStateSnapshot reset = state_;
  reset.state_epoch = 1;
  ASSERT_EQ(core_.Solve(ctx_, reset, ocp_, out), MPCSolveError::kNoError);
  EXPECT_TRUE(out.cold_start);
  EXPECT_TRUE(out.initial_solve) << "the standing budget, as after a build";
  EXPECT_EQ(core_.LastNodeShift(), 0);
  ASSERT_EQ(core_.Solve(ctx_, reset, ocp_, out), MPCSolveError::kNoError);
  EXPECT_FALSE(out.cold_start);
}

TEST_F(MPCHandlerCoreFloatingTest, ControlModelMustMatchTheBase) {
  // The shipped joint-torque OCPs refuse a floating model.
  rtc::mpc::PhaseContext light = ctx_;
  light.ocp_type = "contact_light";
  rtc::mpc::ContactLightMPC light_mpc;
  EXPECT_EQ(light_mpc.Init(solver_cfg_, handler_, limits_, light),
            MPCInitError::kFloatingBaseUnsupported);
  rtc::mpc::PhaseContext rich = ctx_;
  rich.ocp_type = "contact_rich";
  rtc::mpc::ContactRichMPC rich_mpc;
  EXPECT_EQ(rich_mpc.Init(solver_cfg_, handler_, limits_, rich),
            MPCInitError::kFloatingBaseUnsupported);

  // A kinodynamic OCP refuses a fixed base.
  if (!std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  pinocchio::Model fixed;
  pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, fixed);
  rtc::mpc::RobotModelHandler fixed_handler;
  ASSERT_EQ(fixed_handler.Init(fixed, YAML::Load(rtc::mpc::test_utils::kPandaTwoFingerYaml)),
            rtc::mpc::RobotModelInitError::kNoError);
  rtc::mpc::internal::MPCHandlerCore core;
  KinodynamicStubOCP ocp(kHorizon, kMode);
  EXPECT_EQ(core.Init(solver_cfg_, fixed_handler, limits_, ctx_, ocp),
            MPCInitError::kFixedBaseUnsupported);
  EXPECT_FALSE(core.Initialised());
}

}  // namespace
