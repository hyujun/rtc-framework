// ── DemoWbcController TSID-path unit tests (#189) ────────────────────────────
//
// Why this file exists: every pre-existing DemoWbcController test runs with
// `tsid_initialized_ == false` — none of them calls LoadConfig with a `tsid:`
// section — so the entire TSID routing subtree of src/controllers/wbc/compute.cpp
// (ComputeTSIDPosition → ComputeWbcCommon / ComputeKinematicWbc /
// ComputeDynamicWbc) and everything downstream of it (FillTaskPosePods,
// ComputeHandFingertipFk, the TSID branches of UpdatePullEstimate /
// FillPublishOutput / FillDeviceWbcLogPod / FillWbcDiagLogPod) was never
// executed. This file drives the real bring-up on the iiwa7_leap serial profile
// with a full TSID stack so those paths run.
//
// Two things distinguish it from test_demo_task_controller's URDF fixture:
//
//   1. It builds a LifecycleNode and calls on_configure(). InitClik() lives
//      there (wbc/lifecycle.cpp), NOT in LoadConfig — without it
//      clik_tcp_frame_idx_ stays -1 and the CLIK position backbone, the arm-TCP
//      pose pods and the fingertip FK are all unreachable. The base
//      RTControllerInterface::on_configure explicitly supports this entry
//      ("Direct callers (unit tests, legacy paths) skip PreConfigure").
//   2. It therefore touches DDS, so the binary runs under a dedicated
//      ROS_DOMAIN_ID (CMakeLists) — the isolation idiom
//      rtc_controller_manager's full-pipeline tests use.
//
// Numerics are deliberately loose: the QP solve is not bit-reproducible across
// solver builds, so the assertions pin BRANCHES and CONTRACTS (which phase ran,
// which flags/shapes/valid-bits came out, whether the command stayed finite and
// inside the joint box) rather than solved values.

#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <span>
#include <string>

namespace {

using integrated_bringup::DemoWbcController;
using integrated_bringup::WbcPhase;
using rtc::ControllerOutput;
using rtc::ControllerState;

using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kHandDof;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::SetFingertipForce;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;

// ── Controller YAML ─────────────────────────────────────────────────────────
// Structurally the shipped config/iiwa7_leap/controllers/demo_wbc_controller.yaml
// (same task/constraint/contact/preset set, same frame names), trimmed to the
// keys the compute path reads and with the shared `pull_estimator` block folded
// in: node_ carries no `config_variant` parameter here, so
// LoadDemoSharedYamlFile finds no demo_shared.yaml and the per-controller cfg is
// the only route to a wired pull estimator (ApplyDemoSharedConfig(cfg, shared)).
// MPC stays disabled — the MPC bindings already have test_demo_wbc_mpc_integration.
const char* const kWbcYaml = R"(
arm_dof: 7
command_type: "position"

arm_trajectory_speed: 0.5
hand_trajectory_speed: 1.0
arm_max_traj_velocity: 1.71
hand_max_traj_velocity: 4.0
tcp_trajectory_speed: 0.1
tcp_trajectory_angular_speed: 0.5
tcp_max_traj_velocity: 0.5
tcp_max_traj_angular_velocity: 1.0
pi_rotation_margin: 0.15

topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
    publish:
      - topic: "transforms"
        role: "robot_transforms"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"

logs:
  - msg_type: integrated_bringup/DeviceWbcLog
    instance: iiwa7_state
  - msg_type: integrated_bringup/DeviceWbcLog
    instance: leap_state
  - msg_type: integrated_bringup/WbcDiagLog
    instance: wbc_diag
  - msg_type: integrated_bringup/PullEstimatorLog
    instance: pull_estimator

tsid:
  formulation_type: "wqp"
  wqp:
    solver:
      max_iter: 20
      max_iter_in: 100
      eps_abs: 1.0e-6
      eps_rel: 0.0
      verbose: false

  tasks:
    posture:
      type: "posture"
      weight: 1.0
      priority: 0
      arm:  { kp: 36.0, kd: 12.0 }
      hand: { kp: 16.0, kd: 8.0 }
    se3_tcp:
      type: "se3"
      name: "se3_tcp"
      frame: "ee_link"
      base_frame: "link_0"
      mask: [1, 1, 1, 1, 1, 1]
      kp: [36, 36, 36, 16, 16, 16]
      kd: [12.5, 12.5, 12.5, 8.2, 8.2, 8.2]
      weight: 100.0
      priority: 0
    force:
      type: "force"
      weight: 10.0
      priority: 1
    contact_consistency:
      type: "contact_consistency"
      weight: 100.0
      priority: 1
    object_wrench:
      type: "object_wrench"
      weight: 100.0
      priority: 1
    internal_force:
      type: "internal_force"
      weight: 50.0
      priority: 1
    object_se3:
      type: "object_se3"
      mask: [1, 1, 1, 1, 1, 1]
      kp: [50, 50, 50, 20, 20, 20]
      kd: [2.5, 2.5, 2.5, 1, 1, 1]
      weight: 50.0
      priority: 0

  constraints:
    eom:
      type: "eom"
    joint_limit:
      type: "joint_limit"
      dt: 0.002
      position_margin: 0.05
      velocity_margin: 0.1
    friction_cone:
      type: "friction_cone"
    torque_limit:
      type: "torque_limit"
      tau_scale: 1.0

  force_pi:
    kp: 0.5
    ki: 2.0
    i_max: 2.0
    lambda_min: 0.0
    lambda_max: 20.0
    f_des_default: 2.0

  object_frame:
    p_w: [0.0, 0.0, 0.0]
    mass: 0.0

  contacts_default_ramp_sec: 0.02

  contacts:
    - { name: "thumb_tip",  frame: "thumb_tip_head",  type: "point", mu: 0.5, n_faces: 8 }
    - { name: "index_tip",  frame: "index_tip_head",  type: "point", mu: 0.5, n_faces: 8 }
    - { name: "middle_tip", frame: "middle_tip_head", type: "point", mu: 0.5, n_faces: 8 }
    - { name: "ring_tip",   frame: "ring_tip_head",   type: "point", mu: 0.5, n_faces: 8 }

  phase_presets:
    idle:
      tasks:
        posture:             { active: true,  weight: 1.0,   priority: 0 }
        se3_tcp:             { active: false, weight: 50.0,  priority: 0 }
        force:               { active: false, weight: 0.0,   priority: 1 }
        contact_consistency: { active: false, weight: 0.0,   priority: 1 }
        object_wrench:       { active: false, weight: 0.0,   priority: 1 }
        internal_force:      { active: false, weight: 0.0,   priority: 1 }
        object_se3:          { active: false, weight: 0.0,   priority: 0 }
      constraints:
        eom:           { active: true }
        joint_limit:   { active: true }
        friction_cone: { active: false }
        torque_limit:  { active: true }
    approach:
      tasks:
        posture:             { active: true,  weight: 1.0,   priority: 0 }
        se3_tcp:             { active: true,  weight: 200.0, priority: 0 }
        force:               { active: false, weight: 0.0,   priority: 1 }
        contact_consistency: { active: false, weight: 0.0,   priority: 1 }
        object_wrench:       { active: false, weight: 0.0,   priority: 1 }
        internal_force:      { active: false, weight: 0.0,   priority: 1 }
        object_se3:          { active: false, weight: 0.0,   priority: 0 }
      constraints:
        eom:           { active: true }
        joint_limit:   { active: true }
        friction_cone: { active: false }
        torque_limit:  { active: true }
    closure:
      tasks:
        posture:             { active: true,  weight: 0.1,   priority: 0 }
        se3_tcp:             { active: true,  weight: 50.0,  priority: 0 }
        force:               { active: true,  weight: 10.0,  priority: 1 }
        contact_consistency: { active: true,  weight: 100.0, priority: 1 }
        object_wrench:       { active: true,  weight: 100.0, priority: 1 }
        internal_force:      { active: true,  weight: 50.0,  priority: 1 }
        object_se3:          { active: true,  weight: 50.0,  priority: 0 }
      constraints:
        eom:           { active: true }
        joint_limit:   { active: true }
        friction_cone: { active: true }
        torque_limit:  { active: true }
    hold:
      tasks:
        posture:             { active: true,  weight: 0.1,   priority: 0 }
        se3_tcp:             { active: true,  weight: 50.0,  priority: 0 }
        force:               { active: true,  weight: 20.0,  priority: 1 }
        contact_consistency: { active: true,  weight: 100.0, priority: 1 }
        object_wrench:       { active: true,  weight: 100.0, priority: 1 }
        internal_force:      { active: true,  weight: 50.0,  priority: 1 }
        object_se3:          { active: true,  weight: 50.0,  priority: 0 }
      constraints:
        eom:           { active: true }
        joint_limit:   { active: true }
        friction_cone: { active: true }
        torque_limit:  { active: true }
    release:
      tasks:
        posture:             { active: true,  weight: 1.0,   priority: 0 }
        se3_tcp:             { active: true,  weight: 50.0,  priority: 0 }
        force:               { active: false, weight: 0.0,   priority: 1 }
        contact_consistency: { active: false, weight: 0.0,   priority: 1 }
        object_wrench:       { active: false, weight: 0.0,   priority: 1 }
        internal_force:      { active: false, weight: 0.0,   priority: 1 }
        object_se3:          { active: false, weight: 0.0,   priority: 0 }
      constraints:
        eom:           { active: true }
        joint_limit:   { active: true }
        friction_cone: { active: true }
        torque_limit:  { active: true }

integration:
  position_margin: 0.02
  velocity_scale: 0.95
  force_rate_alpha: 0.1

clik:
  damping_sq: 1.0e-4
  v_limit: 1.5
  kx_pos: 5.0
  kx_rot: 2.0
  ka: 1.0
  kh: 1.0
  anchor_drift_max: 0.5

hand_tauff_enable: false
hand_tauff_source: "gravity_comp"
hand_tauff_gravity_gain: 1.0
hand_tauff_closure_bias: 0.0
hand_tauff_max: 5.0

fsm:
  epsilon_pregrasp: 0.005
  force_contact_threshold: 0.2
  min_contacts_for_hold: 2
  slip_rate_threshold: 5.0
  deformation_threshold: 0.015
  max_qp_fail_before_fallback: 5
  approach_speed: 0.5
  release_ramp_sec: 0.03

estop:
  arm_safe_position: [0.0, 0.7, 0.0, -1.4, 0.0, 0.7, 0.0]

# Shared-config block (see the file header): the WBC controller resolves the
# pull estimator's tip links against tsid.contacts, so the four LEAP tips here
# map 1:1 onto the four contacts declared above.
pull_estimator:
  enabled: true
  filter_cutoff_hz: 5.0
  min_valid_contacts: 2
  decay_time_constant_s: 0.1
  slip_risk_threshold: 1.0
  alignment_error_rad: 0.035
  gravity_force: [0.0, 0.0, 0.0]
  inplane_x_reference: [0.0, 0.0, 1.0]
  use_baseline_subtraction: true
  plane_normal_source: "pinch_geometry"
  plane_normal: [0.0, 0.0, 1.0]
  required_roles: ["thumb"]
  tips:
    tip_names: ["thumb", "index", "middle", "ring"]
    thumb:
      link: "thumb_tip_head"
      friction_coeff: 0.7
      force_saturation: 25.0
      contact_on_threshold: 0.5
      contact_off_threshold: 0.2
    index:
      link: "index_tip_head"
      friction_coeff: 0.7
      force_saturation: 25.0
      contact_on_threshold: 0.5
      contact_off_threshold: 0.2
    middle:
      link: "middle_tip_head"
      friction_coeff: 0.7
      force_saturation: 25.0
      contact_on_threshold: 0.5
      contact_off_threshold: 0.2
    ring:
      link: "ring_tip_head"
      friction_coeff: 0.7
      force_saturation: 25.0
      contact_on_threshold: 0.5
      contact_off_threshold: 0.2

mpc:
  enabled: false
)";

// ── Fixture ─────────────────────────────────────────────────────────────────

class WbcTsidPathTest : public ::testing::Test {
 protected:
  /// YAML LoadConfig receives. Overridden by fixtures needing a variant.
  virtual std::string ControllerYaml() const { return kWbcYaml; }

  void SetUp() override {
    ctrl_ = std::make_unique<DemoWbcController>("");
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("wbc_tsid_path_test");

    // Production bring-up sequence (rt_controller_node_params.cpp), then the
    // lifecycle transition. on_configure is where InitClik() runs, and CLIK is
    // the sole position backbone — a controller configured without it would
    // hold last forever and cover none of ComputeKinematicWbc.
    ctrl_->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
    ctrl_->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
    ctrl_->SetControlRate(1.0 / kDt);
    const YAML::Node cfg = YAML::Load(ControllerYaml());
    ctrl_->LoadConfig(cfg);
    ctrl_->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
    configured_ = ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg) ==
                  DemoWbcController::CallbackReturn::SUCCESS;

    state_ = MakeIiwa7LeapState();
    last_out_ = ctrl_->Compute(state_);  // first tick: seed + cache Update
  }

  void TearDown() override {
    ctrl_.reset();
    node_.reset();
  }

  /// Closed-loop tick: the plant is a perfect position tracker on both axes,
  /// so the measured state follows the command and the model cache stays
  /// consistent with what the controller believes it commanded. `feedback_hand`
  /// off pins the fingers (an object blocking closure).
  ControllerOutput RunTicks(int n, bool feedback_hand = true) {
    for (int i = 0; i < n; ++i) {
      state_.iteration += 1;
      last_out_ = ctrl_->Compute(state_);
      if (last_out_.devices[0].num_channels == kArmDof) {
        for (std::size_t j = 0; j < static_cast<std::size_t>(kArmDof); ++j) {
          state_.devices[0].positions[j] = last_out_.devices[0].commands[j];
        }
      }
      if (feedback_hand && last_out_.num_devices > 1 &&
          last_out_.devices[1].num_channels == kHandDof) {
        for (std::size_t j = 0; j < static_cast<std::size_t>(kHandDof); ++j) {
          state_.devices[1].positions[j] = last_out_.devices[1].commands[j];
        }
      }
    }
    return last_out_;
  }

  /// Per-tip sensor-frame sign: the thumb opposes the other three, because a
  /// pinch is fingers pressing against each other rather than four fingers
  /// shoving the same way. NOTE this is not enough to satisfy the estimator's
  /// signed contact gate at the fixture's open-hand configuration — see
  /// PullEstimateAssemblyDeliversEveryActiveContact for why that gate is the
  /// estimator's business and not this file's.
  static constexpr std::array<float, 4> kPinchSign{-1.0F, 1.0F, 1.0F, 1.0F};

  /// Ramp all four fingertip forces up to `target` newtons, one tick per
  /// increment, and leave them pressed.
  ///
  /// Two things here are load-bearing, both learned from the FSM rejecting the
  /// obvious shortcut:
  ///
  ///   * `num_inference_groups` is what DeriveFingertipCounts actually counts.
  ///     LEAP is the force-only lane (inference groups populated, sensor
  ///     channels empty), so without it the per-fingertip payload
  ///     SetFingertipForce writes is invisible and num_active_fingertips_ stays 0.
  ///   * The ramp is not politeness. A step injection is 0→target in one 2 ms
  ///     tick = target·500 N/s raw, and kHold's slip guard
  ///     (fsm.slip_rate_threshold) trips straight to kFallback on the tick it
  ///     lands — observed at 135 N/s after EMA smoothing. `per_tick` keeps the
  ///     steady-state df/dt below that guard, so what this models is a finger
  ///     closing on an object rather than an object hitting a finger.
  void RampFingertipForce(float target, float per_tick = 0.008F) {
    state_.devices[1].num_inference_groups = 4;
    state_.devices[1].num_sensor_channels = 0;
    float f = 0.0F;
    while (f < target) {
      f = std::min(target, f + per_tick);
      for (int k = 0; k < 4; ++k) {
        SetFingertipForce(state_, k, kPinchSign[static_cast<std::size_t>(k)] * f);
      }
      RunTicks(1, /*feedback_hand=*/false);
    }
  }

  /// Drive the real FSM: grasp command + a joint target puts kIdle→kApproach,
  /// and the TCP hold goal is already at the measured pose so the pregrasp
  /// epsilon is met immediately → kClosure. The fingertip ramp then crosses
  /// fsm.force_contact_threshold on ≥2 tips → kHold, and keeps going to 2 N so
  /// the grasp gate (grasp_force_threshold, 1 N) is cleared too. No phase is
  /// forced — every transition here is the production one.
  void DriveToHold() {
    std::array<double, kArmDof> arm_target{};
    for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
      arm_target[i] = kArmHome[i];
    }
    ctrl_->SetGraspCmdForTesting(1);
    ctrl_->SetDeviceTarget(0, std::span<const double>(arm_target));
    RunTicks(4);
    RampFingertipForce(2.0F);
  }

  std::unique_ptr<DemoWbcController> ctrl_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  ControllerState state_;
  ControllerOutput last_out_;
  bool configured_{false};
};

// ── Bring-up ────────────────────────────────────────────────────────────────

// The whole file rests on this: if on_configure did not succeed, TSID and CLIK
// are both off and every test below would pass vacuously against the fallback
// path. DEC-1 ⓐ makes the two inseparable — on_configure FAILS a
// TSID-initialised controller that could not enable CLIK — so a successful
// transition is itself the proof that both are live.
TEST_F(WbcTsidPathTest, ConfiguresTsidAndClikStack) {
  ASSERT_TRUE(configured_) << "on_configure must succeed — TSID + CLIK bring-up";
  EXPECT_TRUE(last_out_.valid);
  EXPECT_EQ(last_out_.num_devices, 2);
  EXPECT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kIdle);
  // clik_tcp_frame_idx_ >= 0 is not directly observable; the arm-tip pose is
  // filled only under that gate, so a valid one is the observable proxy.
  EXPECT_TRUE(last_out_.arm_tip_pose_valid);
}

// kIdle regulates toward the seeded hold snapshot through the full TSID stack
// (ComputeWbcCommon → ComputeKinematicWbc → ComputeDynamicWbc). With the plant
// tracking perfectly the arm must stay where it started rather than drift.
TEST_F(WbcTsidPathTest, IdleHoldsMeasuredPoseThroughTsidStack) {
  ASSERT_TRUE(configured_);
  auto out = RunTicks(50);
  ASSERT_TRUE(out.valid);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i])) << "arm joint " << i;
    EXPECT_NEAR(out.devices[0].commands[i], kArmHome[i], 1e-2)
        << "arm joint " << i << " drifted while holding under TSID";
  }
  EXPECT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kIdle);
}

// The TSID solve health is reported through WbcState. n_lambda_active_ is 0 in
// idle (no contacts activated), which is the "grasp cache empty, Rank()=0"
// branch of ComputeWbcCommon — the counterpart of the closure case below.
TEST_F(WbcTsidPathTest, IdleReportsSolvedTsidWithNoActiveContacts) {
  ASSERT_TRUE(configured_);
  RunTicks(10);
  const auto& ws = ctrl_->GetWbcStateForTesting();
  EXPECT_TRUE(ws.tsid_solver_ok) << "idle TSID solve must converge";
  EXPECT_EQ(ws.qp_fail_count, 0);
  EXPECT_EQ(ws.num_active_contacts, 0);
  // The staged body and the SeqLock body must agree on a normal tick.
  EXPECT_EQ(ctrl_->GetPublishedWbcStateForTesting().phase, ws.phase);
}

// ── FSM through the TSID stack ──────────────────────────────────────────────

// The production transition chain with TSID live. This is what activates the
// contacts, so it is the only route to ComputeWbcCommon's n_lambda_active_ > 0
// branch (grasp matrix + GraspCache::Compute) and to the ForceReferenceUpdater
// tick that pushes λ_des into the ForceTask.
TEST_F(WbcTsidPathTest, GraspCommandDrivesIdleToHoldWithContactsActive) {
  ASSERT_TRUE(configured_);
  DriveToHold();

  EXPECT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kHold)
      << "grasp + pregrasp epsilon + 4 pressed fingertips must reach kHold";
  const auto& ws = ctrl_->GetWbcStateForTesting();
  EXPECT_EQ(ws.num_fingertips, 4);
  EXPECT_GE(ws.num_active_contacts, 2);
  EXPECT_TRUE(ws.grasp_detected);
  EXPECT_GT(ws.max_force, 0.0F);
  EXPECT_TRUE(last_out_.valid);
}

// A held grasp must keep producing a finite, in-box command on every device
// while the friction-cone / object-level tasks are active. Values are not
// pinned (solver non-determinism); the contract is that the QP stayed feasible
// and the command lane stayed honest.
TEST_F(WbcTsidPathTest, HoldPhaseKeepsCommandsFiniteAndBounded) {
  ASSERT_TRUE(configured_);
  DriveToHold();
  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kHold);

  auto out = RunTicks(20, /*feedback_hand=*/false);
  ASSERT_TRUE(out.valid);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i])) << "arm joint " << i;
    // iiwa7 position limits from the fixture's device config, with the
    // configured 0.02 rad margin folded in generously.
    EXPECT_LT(std::abs(out.devices[0].commands[i]), 3.1) << "arm joint " << i;
  }
  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[1].commands[i])) << "hand joint " << i;
  }
  EXPECT_EQ(ctrl_->GetWbcStateForTesting().qp_fail_count, 0);
}

// kRelease under a live TSID stack takes the ComputeReleaseMode branch that
// calls ComputeTSIDPosition (the arm stays on SE3 hold) rather than the
// model-less fresh-hold branch the existing tests exercise, then overlays the
// finger-open ramp once the contact-deactivation window elapses.
TEST_F(WbcTsidPathTest, ReleaseRunsTsidArmHoldAndOpensFingers) {
  ASSERT_TRUE(configured_);
  DriveToHold();
  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kHold);

  ctrl_->SetGraspCmdForTesting(2);  // RELEASE
  RunTicks(2);
  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kRelease);
  EXPECT_EQ(ctrl_->GetReleaseStageForTesting(), 0) << "ramp window first";

  // release_ramp_sec 0.03 s = 15 ticks at 2 ms; then the hand trajectory runs.
  RunTicks(40);
  EXPECT_EQ(ctrl_->GetReleaseStageForTesting(), 1)
      << "contact ramp window elapsed → finger-open trajectory initialised";
  EXPECT_TRUE(last_out_.valid);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_TRUE(std::isfinite(last_out_.devices[0].commands[i]));
  }
}

// ── Publish / log surface fed by the TSID tick ──────────────────────────────

// FillTaskPosePods + ComputeHandFingertipFk: with the arm TCP taken from the
// shared cache oMf and the hand FK composed onto it, the arm tip and all four
// fingertip TFs must be published, and the task pods must carry a real pose.
TEST_F(WbcTsidPathTest, PublishSurfaceCarriesArmTcpAndFingertipPoses) {
  ASSERT_TRUE(configured_);
  auto out = RunTicks(5);

  ASSERT_TRUE(out.arm_tip_pose_valid);
  const auto& p = out.arm_tip_pose.position;
  EXPECT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  // A serial iiwa7 at the fixture home pose has a TCP well away from its base.
  EXPECT_GT(std::hypot(std::hypot(p[0], p[1]), p[2]), 0.1);

  // actual_task_positions comes from the same oMf; with no commanded SE3 the
  // goal pods mirror the actual pose (tcp_goal_ seeded from measured FK).
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(out.actual_task_positions[i])) << "task pod " << i;
    EXPECT_TRUE(std::isfinite(out.task_goal_positions[i])) << "goal pod " << i;
  }
  EXPECT_NEAR(out.actual_task_positions[0], out.task_goal_positions[0], 1e-2);

  for (std::size_t f = 0; f < 4; ++f) {
    EXPECT_TRUE(out.task_link_pose_valid[f]) << "fingertip " << f << " TF withheld";
    EXPECT_TRUE(std::isfinite(out.task_link_poses[f].position[0])) << "fingertip " << f;
  }
}

// The WBC CSV pods are only meaningful on a tick that actually solved. The
// device pod carries the TSID a_opt slice and the SE3 ramp setpoint; assert it
// fills without NaN and reports the per-device command type.
TEST_F(WbcTsidPathTest, DeviceWbcLogPodReflectsSolvedTick) {
  ASSERT_TRUE(configured_);
  auto out = RunTicks(10);

  const auto arm_pod = ctrl_->FillDeviceWbcLogPodForTesting(state_, out, /*device_idx=*/0,
                                                            /*role=*/0);
  EXPECT_EQ(arm_pod.num_joints, kArmDof);
  EXPECT_EQ(arm_pod.role, 0);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_TRUE(std::isfinite(arm_pod.commands[idx])) << "arm pod command " << i;
    EXPECT_TRUE(std::isfinite(arm_pod.accelerations[idx])) << "arm pod a_opt " << i;
  }
  // Arm role carries the task-space block from the same cache oMf the publish
  // path used, so the two must agree on this tick.
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(arm_pod.actual_task_positions[i], out.actual_task_positions[i], 1e-9)
        << "task pod " << i << " disagrees with the publish surface";
  }

  const auto hand_pod = ctrl_->FillDeviceWbcLogPodForTesting(state_, out, /*device_idx=*/1,
                                                             /*role=*/1);
  EXPECT_EQ(hand_pod.num_joints, kHandDof);
  EXPECT_EQ(hand_pod.role, 1);
  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_TRUE(std::isfinite(hand_pod.commands[static_cast<std::size_t>(i)]))
        << "hand pod command " << i;
  }
}

// ── Pull estimate on a TSID tick ────────────────────────────────────────────

// What wbc/compute.cpp owns here is the HAND-OFF, not the verdict: gate on
// contact_geometry_fresh_ (set only by ComputeWbcCommon, i.e. only on a
// TSID-routing tick) and the per-contact active flag, then copy each contact
// frame's oMf rotation + the measured fingertip force into the wiring and mirror
// the estimator's answer back through FillPullEstimateData.
//
// touch_mask is the observable for exactly that hand-off: bit k is set from
// |f_obj| for contact k, so all four bits can only be reached by way of this
// function's assembly. The signed contact gate (f_n = n̂·f_obj vs
// contact_on_threshold) and therefore contact_mask / valid_contact_count are the
// ESTIMATOR's judgement, and they are deliberately not asserted here: those live
// in rtc_controllers and have their own targeted tests (#231/#234) that build
// pinch geometry under control. A uniform sensor-frame push — with or without an
// opposing thumb — is not a pinch at the fixture's open-hand configuration, and
// making it one would mean encoding the estimator's sign convention in a file
// whose subject is a different translation unit.
TEST_F(WbcTsidPathTest, PullEstimateAssemblyDeliversEveryActiveContact) {
  ASSERT_TRUE(configured_);
  DriveToHold();
  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kHold);
  RunTicks(20, /*feedback_hand=*/false);

  const auto& pull = ctrl_->GetWbcStateForTesting().pull;
  EXPECT_EQ(pull.touch_mask, 0b1111)
      << "all four active contacts must reach the estimator with a live force";
  // Whatever the estimator decides, it decided it on real input: these two
  // reasons mean it never received wiring or never had a well-formed plane —
  // i.e. that the hand-off above silently did nothing.
  EXPECT_NE(pull.invalid_reason, 1) << "kNotInitialized — wiring never reached the estimator";
  EXPECT_NE(pull.invalid_reason, 2) << "kDegenerateNormal — no usable plane from the geometry";
  EXPECT_TRUE(std::isfinite(pull.magnitude));
  EXPECT_TRUE(std::isfinite(pull.force_inplane[0]));
  EXPECT_TRUE(std::isfinite(pull.force_inplane[1]));
  EXPECT_TRUE(std::isfinite(pull.friction_utilization));
}

// The negative control for the gate above, and the reason contact_geometry_fresh_
// exists: a non-TSID tick refreshes contact_frames[].oMf just the same (the
// Stage-1 cache Update runs every tick), so oMf freshness cannot be what decides.
// Only ComputeWbcCommon sets the flag, so kFallback — which routes to
// ComputeFallback and never reaches it — must starve the estimator even though
// the fingertips are still pressed and the frames are still fresh.
TEST_F(WbcTsidPathTest, NonTsidTickWithholdsContactGeometryFromPullEstimate) {
  ASSERT_TRUE(configured_);
  DriveToHold();
  ASSERT_EQ(ctrl_->GetWbcStateForTesting().pull.touch_mask, 0b1111);

  ctrl_->ForcePhaseForTesting(WbcPhase::kFallback);
  RunTicks(5, /*feedback_hand=*/false);

  const auto& pull = ctrl_->GetWbcStateForTesting().pull;
  EXPECT_EQ(pull.touch_mask, 0) << "kFallback runs no ComputeWbcCommon — geometry is untrusted";
  EXPECT_EQ(pull.contact_mask, 0);
  EXPECT_FALSE(pull.valid);
  // Still fingertips reporting force — the withholding is the geometry gate,
  // not a lost sensor stream.
  EXPECT_EQ(ctrl_->GetNumActiveFingertipsForTesting(), 4);
}

// The mirror image: on an E-STOP tick no TSID solve runs, so the same fields
// must report "not solved" instead of replaying the last good body. This pins
// the FillEstopPublishState / StageEstopPullTick pair against the TSID-live
// baseline the rest of this file establishes (the pre-existing E-STOP tests run
// with no TSID stack at all, so they cannot tell a stale replay from a reset).
TEST_F(WbcTsidPathTest, EstopAfterTsidTickReportsNotSolvedNotStale) {
  ASSERT_TRUE(configured_);
  DriveToHold();
  ASSERT_TRUE(ctrl_->GetWbcStateForTesting().tsid_solver_ok);

  ctrl_->TriggerEstop();
  RunTicks(2, /*feedback_hand=*/false);

  const auto ws = ctrl_->GetPublishedWbcStateForTesting();
  EXPECT_FALSE(ws.tsid_solver_ok) << "no solve ran this tick";
  EXPECT_FLOAT_EQ(ws.tsid_solve_us, 0.0F);
  EXPECT_FALSE(ws.pull.valid) << "pull estimate must be invalidated under E-STOP";
}

// ── Hand feedforward torque overlay ─────────────────────────────────────────

// ComputeDynamicWbc's τ_ff overlay is opt-in and closure/hold-only. With it on,
// a held grasp must route the hand device to kPdFeedforward and carry finite,
// clamped torques — the branch that is dead whenever TSID is not initialised.
TEST_F(WbcTsidPathTest, HandTauffOverlayEngagesInHoldAndStaysClamped) {
  ASSERT_TRUE(configured_);
  {
    auto g = ctrl_->get_gains();
    g.hand_tauff_enable = true;
    ctrl_->set_gains(g);
  }
  DriveToHold();
  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kHold);
  auto out = RunTicks(5, /*feedback_hand=*/false);

  ASSERT_TRUE(ctrl_->GetHandTauffActiveForTesting())
      << "hold + enable + converged QP must activate the τ_ff overlay";
  EXPECT_EQ(out.devices[1].command_type, rtc::CommandType::kPdFeedforward);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[1].feedforward[i])) << "hand τ_ff " << i;
    EXPECT_LE(std::abs(out.devices[1].feedforward[i]), 5.0 + 1e-9)
        << "hand τ_ff " << i << " exceeds hand_tauff_max";
  }
}

// The overlay is phase-scoped: idle must leave the hand on a plain position
// hold with zero feedforward even while the enable flag is set.
TEST_F(WbcTsidPathTest, HandTauffOverlayStaysOffInIdle) {
  ASSERT_TRUE(configured_);
  {
    auto g = ctrl_->get_gains();
    g.hand_tauff_enable = true;
    ctrl_->set_gains(g);
  }
  auto out = RunTicks(10);

  ASSERT_EQ(ctrl_->GetPhaseForTesting(), WbcPhase::kIdle);
  EXPECT_FALSE(ctrl_->GetHandTauffActiveForTesting());
  EXPECT_NE(out.devices[1].command_type, rtc::CommandType::kPdFeedforward);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    EXPECT_DOUBLE_EQ(out.devices[1].feedforward[i], 0.0) << "hand τ_ff " << i;
  }
}

// ── #340: `arm_dof` is the DoF source, and the cross-check is finally live ────
//
// arm_dof_ used to BE the length of estop.arm_safe_position, so the two could
// not disagree and every throw inside ParseArmSafePosition was unreachable from
// production. Each config below is one the old code accepted in silence.
//
// The message is asserted, not just the throw: a LoadConfig this large has many
// ways to fail, and a bare EXPECT_THROW would go green on any of them.

// Applies `mutate` to kWbcYaml and returns LoadConfig's error message, or "" on
// success. Same bring-up order as WbcTsidPathTest::SetUp up to LoadConfig.
std::string WbcLoadConfigError(const std::function<void(YAML::Node&)>& mutate) {
  DemoWbcController ctrl{""};
  ctrl.SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl.SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl.SetControlRate(1.0 / kDt);
  YAML::Node cfg = YAML::Load(kWbcYaml);
  mutate(cfg);
  try {
    ctrl.LoadConfig(cfg);
  } catch (const std::exception& e) {
    return e.what();
  }
  return "";
}

// Baseline: without this, every assertion below could be green for the wrong
// reason (a fixture that no longer loads at all).
TEST(WbcArmDofSourceTest, UnmutatedFixtureLoads) {
  EXPECT_EQ(WbcLoadConfigError([](YAML::Node&) {}), "");
}

TEST(WbcArmDofSourceTest, SafePositionLengthDisagreeingWithArmDofThrows) {
  const std::string msg = WbcLoadConfigError([](YAML::Node& cfg) { cfg["arm_dof"] = kArmDof - 1; });
  EXPECT_NE(msg.find("demo_wbc_controller"), std::string::npos) << msg;
  EXPECT_NE(msg.find("arm_safe_position"), std::string::npos) << msg;
}

TEST(WbcArmDofSourceTest, MissingArmDofThrowsWithMigrationInstructions) {
  const std::string msg = WbcLoadConfigError([](YAML::Node& cfg) { cfg.remove("arm_dof"); });
  EXPECT_NE(msg.find("demo_wbc_controller"), std::string::npos) << msg;
  EXPECT_NE(msg.find("arm_dof: " + std::to_string(kArmDof)), std::string::npos)
      << "the message must name the value implied by the safe-position length: " << msg;
}

TEST(WbcArmDofSourceTest, MissingEstopSectionThrowsFromTheSharedParser) {
  const std::string msg = WbcLoadConfigError([](YAML::Node& cfg) { cfg.remove("estop"); });
  EXPECT_NE(msg.find("demo_wbc_controller"), std::string::npos) << msg;
  EXPECT_NE(msg.find("estop"), std::string::npos) << msg;
}

TEST(WbcArmDofSourceTest, ArmDofAboveTheControllerCapThrows) {
  const std::string msg = WbcLoadConfigError(
      [](YAML::Node& cfg) { cfg["arm_dof"] = DemoWbcController::kMaxArmDof + 1; });
  EXPECT_NE(msg.find("arm_dof"), std::string::npos) << msg;
  EXPECT_NE(msg.find("out of range"), std::string::npos) << msg;
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
