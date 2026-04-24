/// @file test_grasp_pipeline.cpp
/// @brief Phase 7c integration test — `GraspPhaseManager` + `MPCFactory` +
///        `HandlerMPCThread` wired as the ur5e_bringup handler-mode stack.
///
/// Complements `test_grasp_phase_manager.cpp` (FSM-only, 7a) by driving the
/// full handler pipeline: the concrete grasp FSM now feeds a real MPC
/// handler over a 20 Hz thread, with the WBC FSM bridge simulated via
/// `ForcePhase` — the same surface `DemoWbcController::OnPhaseEnter` calls
/// in production.
///
/// Robot-agnostic: Panda (9-DoF, 2 × 3-dim contacts) is the fixture, in line
/// with every Phase 3–6 rtc_mpc test. The production ur5e_bringup YAMLs
/// target UR5e + 10-DoF hand; this test validates the *wiring*, not the
/// specific UR5e parameterisation (that is covered by the MuJoCo E2E in
/// Phase 7c).

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
#include <filesystem>
#include <memory>
#include <thread>

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"

#include "ur5e_bringup/phase/grasp_phase_manager.hpp"

namespace {

constexpr const char *kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

using ur5e_bringup::phase::GraspCommand;
using ur5e_bringup::phase::GraspPhaseId;
using ur5e_bringup::phase::GraspPhaseManager;

// Panda-sized equivalent of config/controllers/phase_config.yaml. Kept
// minimal — only the dimensions the real YAML varies are exercised here.
constexpr const char *kPandaPhaseConfig = R"(
transition:
  approach_tolerance: 0.05
  pregrasp_tolerance: 0.01
  force_threshold: 0.5
  max_failures: 100
phases:
  idle:
    ocp_type: "light_contact"
    active_contact_indices: []
    cost: &light
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
  approach:    { ocp_type: "light_contact", active_contact_indices: [], cost: *light }
  pre_grasp:   { ocp_type: "light_contact", active_contact_indices: [], cost: *light }
  closure:
    ocp_type: "contact_rich"
    active_contact_indices: [0, 1]
    cost: &rich
      horizon_length: 15
      dt: 0.01
      w_frame_placement: 50.0
      w_state_reg: 0.1
      w_control_reg: 0.01
      w_contact_force: 10.0
      w_centroidal_momentum: 0.0
      W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
      q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
      F_target: [0, 0, 2, 0, 0, 2]
  hold:        { ocp_type: "contact_rich", active_contact_indices: [0, 1], cost: *rich }
  manipulate:  { ocp_type: "contact_rich", active_contact_indices: [0, 1], cost: *rich }
  retreat:     { ocp_type: "light_contact", active_contact_indices: [], cost: *light }
  release:     { ocp_type: "light_contact", active_contact_indices: [], cost: *light }
)";

// Panda-flavoured analogue of config/controllers/mpc/light_contact.yaml.
// The `mpc.model` block is built by the test fixture before this YAML is
// consumed — MPCFactory only reads `mpc.ocp_type`, `mpc.solver`, `mpc.limits`.
constexpr const char *kLightFactoryYaml = R"(
mpc:
  ocp_type: "light_contact"
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    mu_init: 1.0e-2
    max_iters: 40
    max_al_iters: 10
    verbose: false
  limits:
    friction_mu: 0.7
    n_friction_facets: 4
)";

constexpr const char *kRichFactoryYaml = R"(
mpc:
  ocp_type: "contact_rich"
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    mu_init: 1.0e-2
    max_iters: 40
    max_al_iters: 15
    verbose: false
  limits:
    friction_mu: 0.7
    n_friction_facets: 4
)";

YAML::Node MinimalManagerConfig() {
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 200;
  YAML::Node r;
  r["enabled"] = true;
  r["gain_scale"] = 0.3;
  r["accel_only"] = true;
  cfg["riccati"] = r;
  return cfg;
}

rtc::mpc::MpcThreadLaunchConfig MakeLaunchCfg(double hz) {
  rtc::mpc::MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1; // no pinning in unit tests
  launch.main.sched_priority = 0;
  launch.num_workers = 0;
  launch.target_frequency_hz = hz;
  return launch;
}

class GraspPipelineTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto model_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(model_handler_.Init(model_, model_cfg),
              rtc::mpc::RobotModelInitError::kNoError);

    phase_manager_ = std::make_unique<GraspPhaseManager>(model_handler_);
    ASSERT_EQ(phase_manager_->Load(YAML::Load(kPandaPhaseConfig)),
              ur5e_bringup::phase::GraspPhaseInitError::kNoError);
    ASSERT_TRUE(phase_manager_->Initialised());

    // Neutral-pose EE target so the grasp FSM can hand a non-identity pose
    // to the OCP builder from tick 1.
    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    const auto tcp_now = pdata.oMf[model_handler_.end_effector_frame_id()];
    ur5e_bringup::phase::GraspTarget target{};
    target.grasp_pose = tcp_now;
    target.pregrasp_pose = tcp_now;
    target.approach_start = tcp_now;
    phase_manager_->SetTaskTarget(target);
  }

  rtc::mpc::PhaseContext BuildIdleContext() {
    Eigen::VectorXd q = pinocchio::neutral(model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_handler_.nv());
    Eigen::VectorXd sensor = Eigen::VectorXd::Zero(0);
    pinocchio::Data pdata(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q);
    const auto tcp = pdata.oMf[model_handler_.end_effector_frame_id()];
    return phase_manager_->Update(q, v, sensor, tcp, 0.0);
  }

  void WriteNeutralState(rtc::mpc::MPCSolutionManager &mgr) const {
    const auto q = pinocchio::neutral(model_);
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_handler_.nv());
    mgr.WriteState(q, v, /*timestamp_ns=*/1);
  }

  pinocchio::Model model_{};
  rtc::mpc::RobotModelHandler model_handler_{};
  std::unique_ptr<GraspPhaseManager> phase_manager_{};
};

// ── 1. Factory build path ─────────────────────────────────────────────────

TEST_F(GraspPipelineTest, MPCFactoryBuildsLightContactFromIdleContext) {
  const auto ctx = BuildIdleContext();
  ASSERT_EQ(ctx.ocp_type, "light_contact");

  std::unique_ptr<rtc::mpc::MPCHandlerBase> handler;
  const auto status = rtc::mpc::MPCFactory::Create(
      YAML::Load(kLightFactoryYaml), model_handler_, ctx, handler);
  ASSERT_EQ(status.error, rtc::mpc::MPCFactoryError::kNoError)
      << "MPCFactory rejected the phase_config.yaml-shaped context";
  ASSERT_TRUE(handler);
  EXPECT_TRUE(handler->Initialised());
  EXPECT_EQ(handler->ocp_type(), "light_contact");
  EXPECT_EQ(handler->nq(), model_handler_.nq());
}

// ── 2. End-to-end 20 Hz loop (LightContact) ───────────────────────────────

TEST_F(GraspPipelineTest, HandlerThreadLoopSolvesWithGraspPhaseManager) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(),
           /*n_contact_vars=*/6);
  WriteNeutralState(mgr);

  const auto ctx = BuildIdleContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> handler;
  ASSERT_EQ(rtc::mpc::MPCFactory::Create(YAML::Load(kLightFactoryYaml),
                                         model_handler_, ctx, handler)
                .error,
            rtc::mpc::MPCFactoryError::kNoError);

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler),
                   std::move(phase_manager_), YAML::Load(kLightFactoryYaml),
                   YAML::Load(kRichFactoryYaml));
  thread.Init(mgr, MakeLaunchCfg(20.0));
  thread.Start();

  // Keep feeding neutral state so ComputeReference stays within the
  // MaxStaleSolutions window for the duration of the test.
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start)
             .count() < 400) {
    WriteNeutralState(mgr);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  thread.RequestStop();
  thread.Join();

  EXPECT_GT(thread.TotalSolves(), 3u)
      << "20 Hz thread should have solved several times over 400 ms";
  EXPECT_EQ(thread.FailedSolves(), 0u)
      << "LightContact steady-state path must not fail on the neutral pose";
  EXPECT_EQ(thread.LastSolveErrorCode(), 0);
  EXPECT_EQ(thread.LastPhaseId(), static_cast<int>(GraspPhaseId::kIdle));
}

// ── 3. Cross-mode swap via ForcePhase (WBC bridge analogue) ───────────────
//
// `DemoWbcController::OnPhaseEnter` calls `phase_manager_->ForcePhase(...)`
// when the WBC FSM crosses into `kClosure`. That path flips `ctx.ocp_type`
// from `light_contact` to `contact_rich`, which `HandlerMPCThread`
// translates into a `MPCFactory::Create`-based handler swap. This test
// reproduces the same sequence to prove the bridge works end-to-end.

TEST_F(GraspPipelineTest, ForcePhaseClosureTriggersCrossModeSwap) {
  rtc::mpc::MPCSolutionManager mgr;
  mgr.Init(MinimalManagerConfig(), model_handler_.nq(), model_handler_.nv(),
           /*n_contact_vars=*/6);
  WriteNeutralState(mgr);

  const auto ctx = BuildIdleContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> handler;
  ASSERT_EQ(rtc::mpc::MPCFactory::Create(YAML::Load(kLightFactoryYaml),
                                         model_handler_, ctx, handler)
                .error,
            rtc::mpc::MPCFactoryError::kNoError);

  // Capture a raw pointer so the test thread can call ForcePhase after
  // ownership moves into HandlerMPCThread::Configure.
  auto *pm_raw = phase_manager_.get();

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(model_handler_, std::move(handler),
                   std::move(phase_manager_), YAML::Load(kLightFactoryYaml),
                   YAML::Load(kRichFactoryYaml));
  thread.Init(mgr, MakeLaunchCfg(20.0));
  thread.Start();

  // A few IDLE ticks first so the initial solve lands.
  for (int i = 0; i < 5; ++i) {
    WriteNeutralState(mgr);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // WBC bridge analogue: force the grasp FSM into CLOSURE.
  pm_raw->ForcePhase(static_cast<int>(GraspPhaseId::kClosure));

  bool saw_closure = false;
  for (int i = 0; i < 40; ++i) {
    WriteNeutralState(mgr);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (thread.LastPhaseId() == static_cast<int>(GraspPhaseId::kClosure)) {
      saw_closure = true;
      break;
    }
  }
  // A small extra window for the swapped contact_rich handler to run at
  // least one warm solve (mirrors the rtc_mpc integration test pattern).
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  thread.RequestStop();
  thread.Join();

  ASSERT_TRUE(saw_closure)
      << "ForcePhase(kClosure) did not reach the HandlerMPCThread within "
         "2 seconds — bridge or swap is broken";
  EXPECT_NE(thread.LastSolveErrorCode(),
            static_cast<int>(rtc::mpc::MPCSolveError::kRebuildRequired))
      << "Cross-mode swap returned kRebuildRequired — factory rejected the "
         "contact_rich config or handler_ failed to take ownership";
  EXPECT_GE(thread.TotalSolves(), 5u);
}

} // namespace
