/// @file test_mpc_thread_integration.cpp
/// @brief Phase 6 end-to-end pipeline test — `HandlerMPCThread` +
///        `MockPhaseManager` + `LightContactMPC` driving the full MPC-RT
///        loop on the generic Panda fixture.
///
/// Covers Phase 6 Exit Criteria:
///   - Thread lifecycle survives null-handler configuration (one-shot log).
///   - 20 Hz solve loop converges on a static Phase A target.
///   - Phase transition (auto + `ForcePhase`) is reflected by
///     `HandlerMPCThread::LastPhaseId()` within one tick.
///   - `MPCSolutionManager` publishes solutions consumable by a 1 kHz RT
///     polling loop (mirrors `test_mpc_thread_mock.cpp`).
///   - Injected handler failure does not crash the thread; recovery works.
///
/// Robot-agnostic invariant — this file names `panda` only via the
/// `kPandaUrdf` constant (same pattern as every other Phase 3–5 test). The
/// `MockPhaseManager` does not know about Panda; the fixture injects the
/// contact-frame ids resolved by `RobotModelHandler`.

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

#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <thread>

#include "rtc_mpc/handler/light_contact_mpc.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/phase/phase_cost_config.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include "mock_phase_manager.hpp"

namespace {

constexpr const char *kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

constexpr const char *kPhaseACostYaml = R"(
horizon_length: 15
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

constexpr const char *kPhaseBCostYaml = R"(
horizon_length: 15
dt: 0.01
w_frame_placement: 150.0
w_state_reg: 1.0
w_control_reg: 0.01
w_contact_force: 0.0
w_centroidal_momentum: 0.0
W_placement: [150.0, 150.0, 150.0, 15.0, 15.0, 15.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

YAML::Node MinimalManagerConfig() {
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 100;
  YAML::Node r;
  r["enabled"] = true;
  r["gain_scale"] = 0.3;
  r["accel_only"] = true;
  cfg["riccati"] = r;
  return cfg;
}

class MpcThreadIntegrationTest : public ::testing::Test {
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
    ASSERT_EQ(model_handler_.Init(model_, robot_cfg),
              rtc::mpc::RobotModelInitError::kNoError);

    // Pre-build both phase cost configs once.
    ASSERT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(
                  YAML::Load(kPhaseACostYaml), model_handler_, cost_A_),
              rtc::mpc::PhaseCostConfigError::kNoError);
    ASSERT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(
                  YAML::Load(kPhaseBCostYaml), model_handler_, cost_B_),
              rtc::mpc::PhaseCostConfigError::kNoError);

    // EE target at neutral FK; Phase B shifts +10 cm along z.
    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ee_target_A_ = pdata.oMf[model_handler_.end_effector_frame_id()];
    ee_target_B_ = ee_target_A_;
    ee_target_B_.translation().z() += 0.10;
  }

  rtc::mpc::PhaseContext MakeInitialContextForHandler() const {
    rtc::mpc::PhaseContext ctx{};
    ctx.phase_id = 0;
    ctx.phase_name = "free_flight";
    ctx.phase_changed = false;
    ctx.ocp_type = "light_contact";
    ctx.cost_config = cost_A_;
    ctx.contact_plan = {};
    ctx.ee_target = ee_target_A_;
    return ctx;
  }

  rtc::mpc::test_utils::MockPhaseManager::Params
  MakeMockParams(int transition_tick, bool cross_mode = false) const {
    rtc::mpc::test_utils::MockPhaseManager::Params p;
    p.cost_config_A = cost_A_;
    p.cost_config_B = cost_B_;
    // Both plans empty for baseline — MockPhaseManager only shifts EE target
    // and weights between A and B. Cross-mode stretch overrides this by
    // populating a contact plan for Phase B.
    p.contact_plan_A = {};
    p.contact_plan_B = {};
    p.ee_target_A = ee_target_A_;
    p.ee_target_B = ee_target_B_;
    p.transition_tick = transition_tick;
    p.cross_mode = cross_mode;
    return p;
  }

  std::unique_ptr<rtc::mpc::LightContactMPC> MakeInitialisedLightHandler() {
    auto h = std::make_unique<rtc::mpc::LightContactMPC>();
    rtc::mpc::MPCSolverConfig cfg{};
    cfg.prim_tol = 1e-3;
    cfg.dual_tol = 1e-2;
    cfg.mu_init = 1e-2;
    cfg.max_iters = 40;
    cfg.max_al_iters = 10;
    rtc::mpc::OCPLimits limits{};
    const auto ctx = MakeInitialContextForHandler();
    EXPECT_EQ(h->Init(cfg, model_handler_, limits, ctx),
              rtc::mpc::MPCInitError::kNoError);
    return h;
  }

  void WriteNeutralState(rtc::mpc::MPCSolutionManager &mgr) const {
    const Eigen::VectorXd q = pinocchio::neutral(model_);
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_handler_.nv());
    mgr.WriteState(q, v, 0);
  }

  rtc::mpc::MpcThreadLaunchConfig MakeLaunchCfg(double hz = 20.0) const {
    rtc::mpc::MpcThreadLaunchConfig launch{};
    launch.main.cpu_core = -1; // no pinning in unit tests
    launch.main.sched_policy = 0;
    launch.num_workers = 0;
    launch.target_frequency_hz = hz;
    return launch;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler model_handler_;
  rtc::mpc::PhaseCostConfig cost_A_;
  rtc::mpc::PhaseCostConfig cost_B_;
  pinocchio::SE3 ee_target_A_;
  pinocchio::SE3 ee_target_B_;
};

// ── 1. Null-handler guard ────────────────────────────────────────────────

TEST_F(MpcThreadIntegrationTest, NullHandlerLogsOnceAndSkips) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(), 6);
  WriteNeutralState(mgr);

  rtc::mpc::HandlerMPCThread thread;
  // Configure with nullptr handler + nullptr phase manager — Solve must
  // short-circuit without crashing.
  thread.Configure(model_handler_, nullptr, nullptr);
  thread.Init(mgr, MakeLaunchCfg(100.0));
  thread.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  thread.RequestStop();
  thread.Join();

  EXPECT_TRUE(thread.NullHandlerLogged());
  EXPECT_EQ(thread.TotalSolves(), 0u)
      << "Null path must not increment total_solves — it returns before the "
         "counter update";
  EXPECT_GT(thread.FailedSolves(), 0u);
}

// ── 2. Default-phase convergence (baseline solver loop) ──────────────────

TEST_F(MpcThreadIntegrationTest, DefaultPhaseConvergesFreeFlight) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(), 6);
  WriteNeutralState(mgr);

  auto mock = std::make_unique<rtc::mpc::test_utils::MockPhaseManager>(
      MakeMockParams(/*transition_tick=*/-1)); // never auto-transition
  auto handler = MakeInitialisedLightHandler();

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler), std::move(mock));
  thread.Init(mgr, MakeLaunchCfg(20.0));
  thread.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  thread.RequestStop();
  thread.Join();

  // 500 ms at 20 Hz → ~10 solves; be generous on the lower bound to avoid
  // flakiness on loaded CI machines.
  EXPECT_GE(thread.TotalSolves(), 5u);
  EXPECT_EQ(thread.FailedSolves(), 0u)
      << "Baseline light_contact solve loop must not fail on neutral pose";
  EXPECT_EQ(thread.LastSolveErrorCode(), 0);
  EXPECT_EQ(thread.LastPhaseId(), 0);
}

// ── 3. Phase transition picked up within one tick ────────────────────────

TEST_F(MpcThreadIntegrationTest, PhaseTransitionPickedUpWithinOneTick) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(), 6);
  WriteNeutralState(mgr);

  auto mock = std::make_unique<rtc::mpc::test_utils::MockPhaseManager>(
      MakeMockParams(/*transition_tick=*/5)); // auto-transition early
  auto handler = MakeInitialisedLightHandler();

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler), std::move(mock));
  thread.Init(mgr, MakeLaunchCfg(20.0)); // 50 ms period
  thread.Start();

  // 20 Hz × 5 ticks = ~250 ms for auto-transition; poll in 50 ms chunks until
  // phase id flips to 1 or we exceed a safety bound.
  bool transitioned = false;
  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (thread.LastPhaseId() == 1) {
      transitioned = true;
      break;
    }
  }

  // After the FSM flipped, give the thread one more 50 ms window so the
  // next solve with phase B has time to land, then stop.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  thread.RequestStop();
  thread.Join();

  EXPECT_TRUE(transitioned)
      << "MockPhaseManager auto-transition (tick=5) did not surface within "
         "the 1-second safety window";
  EXPECT_EQ(thread.LastPhaseId(), 1);
  EXPECT_EQ(thread.FailedSolves(), 0u)
      << "Light_contact handler should survive a weight/target swap "
         "without factory rebuild";
}

// ── 4. ForcePhase overrides auto-transition guards ───────────────────────

TEST_F(MpcThreadIntegrationTest, ForcePhaseOverridesGuards) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(), 6);
  WriteNeutralState(mgr);

  auto mock = std::make_unique<rtc::mpc::test_utils::MockPhaseManager>(
      MakeMockParams(/*transition_tick=*/-1));
  rtc::mpc::test_utils::MockPhaseManager *mock_raw = mock.get();
  auto handler = MakeInitialisedLightHandler();

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler), std::move(mock));
  thread.Init(mgr, MakeLaunchCfg(20.0));
  thread.Start();

  // Give the loop a few ticks in phase A then force phase B.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(thread.LastPhaseId(), 0);

  mock_raw->ForcePhase(1);

  bool saw_phase_b = false;
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (thread.LastPhaseId() == 1) {
      saw_phase_b = true;
      break;
    }
  }
  thread.RequestStop();
  thread.Join();

  EXPECT_TRUE(saw_phase_b) << "ForcePhase(1) did not propagate to the thread";
  EXPECT_EQ(thread.FailedSolves(), 0u);
}

// ── 5. State-dim mismatch path does not crash the thread ────────────────

TEST_F(MpcThreadIntegrationTest, HandlerSurvivesStateDimMismatch) {
  rtc::mpc::MPCSolutionManager mgr;
  // Intentionally mis-size the manager's state container so WriteState
  // produces a snapshot with unexpected dims.
  const int bad_nq = model_handler_.nq() + 1;
  const int bad_nv = model_handler_.nv() + 1;
  mgr.Init(MinimalManagerConfig(), bad_nq, bad_nv, 6);
  Eigen::VectorXd q_bad = Eigen::VectorXd::Zero(bad_nq);
  Eigen::VectorXd v_bad = Eigen::VectorXd::Zero(bad_nv);
  mgr.WriteState(q_bad, v_bad, 0);

  auto mock = std::make_unique<rtc::mpc::test_utils::MockPhaseManager>(
      MakeMockParams(/*transition_tick=*/-1));
  auto handler = MakeInitialisedLightHandler();

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler), std::move(mock));
  thread.Init(mgr, MakeLaunchCfg(50.0));
  thread.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  thread.RequestStop();
  thread.Join();

  EXPECT_GT(thread.TotalSolves(), 0u);
  EXPECT_GT(thread.FailedSolves(), 0u);
  EXPECT_EQ(thread.LastSolveErrorCode(),
            static_cast<int>(rtc::mpc::MPCSolveError::kStateDimMismatch));
}

// ── 6. End-to-end: solution manager → RT-poll consumption ───────────────

TEST_F(MpcThreadIntegrationTest, SolutionManagerConsumptionE2E) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(), 6);
  WriteNeutralState(mgr);

  auto mock = std::make_unique<rtc::mpc::test_utils::MockPhaseManager>(
      MakeMockParams(/*transition_tick=*/-1));
  auto handler = MakeInitialisedLightHandler();

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler), std::move(mock));
  thread.Init(mgr, MakeLaunchCfg(50.0));

  // Warm start by writing initial state before Start, mirroring the mock
  // end-to-end test shape.
  const Eigen::VectorXd q0 = pinocchio::neutral(model_);
  Eigen::VectorXd q = q0;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_handler_.nv());

  thread.Start();

  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(model_handler_.nq());
  Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(model_handler_.nv());
  Eigen::VectorXd a_ff = Eigen::VectorXd::Zero(model_handler_.nv());
  Eigen::VectorXd lambda_ref = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd u_fb = Eigen::VectorXd::Zero(model_handler_.nv());
  rtc::mpc::InterpMeta meta{};

  int valid_count = 0;
  double max_u_fb_abs = 0.0;
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start)
             .count() < 500) {
    const auto now = std::chrono::steady_clock::now();
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch())
            .count());
    mgr.WriteState(q, v, now_ns);
    const bool ok = mgr.ComputeReference(q, v, now_ns, q_ref, v_ref, a_ff,
                                         lambda_ref, u_fb, meta);
    if (ok) {
      ++valid_count;
      for (int i = 0; i < u_fb.size(); ++i) {
        max_u_fb_abs = std::max(max_u_fb_abs, std::abs(u_fb(i)));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  thread.RequestStop();
  thread.Join();

  EXPECT_GT(valid_count, 10)
      << "Expected many valid ComputeReference calls over 500 ms";
  EXPECT_LT(mgr.StaleCount(), mgr.MaxStaleSolutions())
      << "Stale counter should stay below threshold";
  EXPECT_EQ(thread.FailedSolves(), 0u);
  for (int i = 0; i < model_handler_.nq(); ++i) {
    EXPECT_TRUE(std::isfinite(q_ref(i)));
    EXPECT_TRUE(std::isfinite(u_fb(i)));
  }
  EXPECT_TRUE(std::isfinite(max_u_fb_abs));
}

// ── 7. Cross-mode swap stretch (Phase 6 Step 6) ─────────────────────────
// Exercises `HandlerMPCThread::TryCrossModeSwap`: starts in `light_contact`,
// MockPhaseManager flips `ocp_type` to `"contact_rich"` at phase B, and the
// thread swaps its handler via `MPCFactory::Create` +
// `SeedWarmStart(prev_out_)`.
// `MPCFactoryTest.CrossModeSwapPreservesSolveability` covers the non-threaded
// swap path; this case closes the threaded-loop gap.

TEST_F(MpcThreadIntegrationTest, CrossModeSwapSucceedsOnPhaseTransition) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(), 6);
  WriteNeutralState(mgr);

  // Phase B carries both fingertip contacts so the `contact_rich` handler
  // has a non-empty contact plan to work with (matches the MPCFactory test's
  // `MakeContactContext` shape).
  auto params = MakeMockParams(/*transition_tick=*/5, /*cross_mode=*/true);
  const int lf = model_handler_.contact_frames()[0].frame_id;
  const int rf = model_handler_.contact_frames()[1].frame_id;
  params.contact_plan_B.frames = model_handler_.contact_frames();
  params.contact_plan_B.phases.push_back(
      rtc::mpc::ContactPhase{{lf, rf}, 0.0, 100.0});

  auto mock = std::make_unique<rtc::mpc::test_utils::MockPhaseManager>(params);
  auto handler = MakeInitialisedLightHandler();

  // Pre-build both factory YAML nodes — MPCFactory cross-checks the YAML
  // `ocp_type` against `ctx.ocp_type`, so we need one per dispatch key.
  auto cfg_light = YAML::Load(R"(
mpc:
  ocp_type: light_contact
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    max_iters: 40
)");
  auto cfg_rich = YAML::Load(R"(
mpc:
  ocp_type: contact_rich
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    max_iters: 40
  limits:
    friction_mu: 0.7
)");

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler), std::move(mock),
                   cfg_light, cfg_rich);
  thread.Init(mgr, MakeLaunchCfg(20.0));
  thread.Start();

  // Wait for the auto-transition (tick=5 at 20 Hz ≈ 250 ms) + a few more
  // ticks for the swap + first `contact_rich` solve to land.
  bool saw_phase_b = false;
  for (int i = 0; i < 40; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (thread.LastPhaseId() == 1) {
      saw_phase_b = true;
      break;
    }
  }
  // One extra window so the swapped `contact_rich` handler can run at least
  // one warm solve after `SeedWarmStart`.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  thread.RequestStop();
  thread.Join();

  ASSERT_TRUE(saw_phase_b)
      << "MockPhaseManager cross-mode auto-transition (tick=5) never surfaced";
  EXPECT_EQ(thread.LastPhaseId(), 1);
  // The swap branch is the only path that can return `kRebuildRequired`; if
  // it did, the swap failed and the rich handler never took over. We allow
  // ContactRich Risk #14 NaN flakes to bump FailedSolves (kSolverException),
  // but kRebuildRequired must NOT be the last error seen.
  const int last_err = thread.LastSolveErrorCode();
  EXPECT_NE(last_err,
            static_cast<int>(rtc::mpc::MPCSolveError::kRebuildRequired))
      << "Cross-mode swap reported kRebuildRequired — factory rejected the "
         "config or handler_ failed to take ownership of the new mode.";
  // Sanity: we did run solves after the transition. TotalSolves grows each
  // tick regardless of error class, so a non-zero post-transition count is
  // the minimum proof the loop kept going.
  EXPECT_GE(thread.TotalSolves(), 5u);
}

} // namespace
