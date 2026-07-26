// ── Unit tests for TaskImpedanceController (spec §6.2, A=NONE) ───────────────
// Control-law + selection-matrix + configure + activation + E-STOP + zero-alloc
// tests that need a real robot model (the algebraic nullspace/DLS identities
// T4.5/T5.1 are covered model-free in test_compliance_core.cpp).
//
// Zero-allocation (T7.1) is checked by GLOBAL operator-new interposition, not
// Eigen's set_is_malloc_allowed: Compute() lives in the rtc_controllers library
// (a separate TU compiled without the Eigen malloc macro), so a same-TU Eigen
// guard would observe nothing. operator-new interposition catches allocations in
// the linked library too, and is NDEBUG-independent — the "malloc 인터포지션"
// path §11.7 T7.1 explicitly permits.
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

// That interposition is rtc::testing::ScopedAllocGate
// (rtc_controllers/testing/alloc_gate.hpp) — shared with every other suite that
// gates an RT path, rather than a per-file copy of the operator-new replacement.

namespace {

using rtc::TaskImpedanceController;
using Sel = rtc::TaskImpedanceController::TaskSelection;

std::string Urdf6() {
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

std::string Urdf7() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

rtc::ControllerState MakeState(int nj, const std::vector<double>& q, const std::vector<double>& qd,
                               double dt = 0.002) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  for (int i = 0; i < nj; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] =
        (i < static_cast<int>(q.size())) ? q[static_cast<std::size_t>(i)] : 0.0;
    state.devices[0].velocities[static_cast<std::size_t>(i)] =
        (i < static_cast<int>(qd.size())) ? qd[static_cast<std::size_t>(i)] : 0.0;
  }
  return state;
}

// Independent ĝ(q) for the same URDF, for τ = g(q) comparison.
std::vector<double> GravityAt(const std::string& urdf, const std::vector<double>& q) {
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = urdf;
  cfg.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
  rtc_urdf_bridge::RtModelHandle handle(builder.GetFullModel());
  handle.ComputeGeneralizedGravity(std::span<const double>(q.data(), q.size()));
  auto g = handle.GetGeneralizedGravity();
  return std::vector<double>(g.data(), g.data() + g.size());
}

// ── T4.2: f_ext=0, e=0, q̇=0 → τ = g(q) ─────────────────────────────────────
TEST(TaskImpedance, GravityOnlyAtRest) {
  const std::vector<double> q(6, 0.3);  // bent pose ⇒ nonzero gravity (non-vacuous)
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  // First Compute seeds X_d = X_meas ⇒ e = 0; q̇ = 0 ⇒ ė = 0; nv == m ⇒ no
  // nullspace ⇒ τ = ĝ(q).
  const auto out = ctrl.Compute(state);
  const auto grav = GravityAt(Urdf6(), q);
  ASSERT_EQ(out.command_type, rtc::CommandType::kTorque);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6)
        << "joint " << i;
}

// ── T4.4: TRANSLATION_ONLY filters rotation, keeps translation ──────────────
TEST(TaskImpedance, TranslationOnlySelection) {
  const std::vector<double> q(7, 0.2);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 50.0;         // required for TRANSLATION_ONLY (§6.1)
  gains.activation_ramp_time = 0.0;  // full gain immediately
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kTranslationOnly);
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  const auto seed = ctrl.Compute(state);  // seeds X_d = X_meas
  const double px = seed.actual_task_positions[0];
  const double py = seed.actual_task_positions[1];
  const double pz = seed.actual_task_positions[2];

  auto target = [&](double x, double y, double z, double roll, double pitch, double yaw) {
    const std::array<double, 6> t{x, y, z, roll, pitch, yaw};
    ctrl.SetDeviceTarget(0, std::span<const double>(t.data(), t.size()));
    return ctrl.Compute(state);
  };

  // Same position, two different orientations → identical torque (rotation is
  // not in the task under TRANSLATION_ONLY).
  const auto rotA = target(px, py, pz, 0.4, -0.2, 0.1);
  const auto rotB = target(px, py, pz, -0.3, 0.5, -0.6);
  for (int i = 0; i < 7; ++i)
    EXPECT_NEAR(rotA.devices[0].commands[static_cast<std::size_t>(i)],
                rotB.devices[0].commands[static_cast<std::size_t>(i)], 1e-9)
        << "orientation leaked into task torque, joint " << i;

  // A position offset DOES change the torque (translation is regulated).
  const auto shifted = target(px + 0.05, py, pz, 0.4, -0.2, 0.1);
  double diff = 0.0;
  for (int i = 0; i < 7; ++i)
    diff += std::abs(shifted.devices[0].commands[static_cast<std::size_t>(i)] -
                     rotA.devices[0].commands[static_cast<std::size_t>(i)]);
  EXPECT_GT(diff, 1e-3) << "position error produced no task torque";
}

// ── T6.2: TRANSLATION_ONLY + nullspace_stiffness == 0 → configure error ─────
TEST(TaskImpedance, ConfigureRejectsTranslationOnlyZeroNullspace) {
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kTranslationOnly);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Node()), std::runtime_error);

  // FULL_SE3 with zero nullspace is fine (orientation is a task DoF).
  TaskImpedanceController ctrl_full(Urdf6(), gains, Sel::kFullSe3);
  EXPECT_NO_THROW(ctrl_full.LoadConfig(YAML::Node()));

  // TRANSLATION_ONLY with nonzero nullspace stiffness is accepted.
  TaskImpedanceController::Gains ok = gains;
  ok.nullspace_kp = 30.0;
  TaskImpedanceController ctrl_ok(Urdf6(), ok, Sel::kTranslationOnly);
  EXPECT_NO_THROW(ctrl_ok.LoadConfig(YAML::Node()));
}

// ── Activation: re-seeds the desired pose to the measured state ─────────────
TEST(TaskImpedance, ReactivationHoldsMeasured) {
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto s1 = MakeState(6, std::vector<double>(6, 0.1), std::vector<double>(6, 0.0));
  (void)ctrl.Compute(s1);  // seed at q1

  // Move to a new measured pose and force re-activation: X_d must follow to the
  // new measured pose (e = 0 ⇒ τ = gravity at q2), not drive back to q1.
  auto s2 = MakeState(6, std::vector<double>(6, 0.4), std::vector<double>(6, 0.0));
  ctrl.ResetTargetInitializationForTesting();
  const auto out = ctrl.Compute(s2);
  const auto grav = GravityAt(Urdf6(), std::vector<double>(6, 0.4));
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6)
        << "joint " << i << " did not hold the re-measured pose";
}

// ── E-STOP: gravity-compensated damped hold (E-8) ───────────────────────────
TEST(TaskImpedance, EstopGravityCompHold) {
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));  // q̇ = 0
  (void)ctrl.Compute(state);
  ctrl.TriggerEstop();
  const auto out = ctrl.Compute(state);
  // τ = ĝ(q) − D·0 = ĝ(q), clamped (within limits here).
  const auto grav = GravityAt(Urdf6(), q);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6);
}

// ── E-STOP recovery: a target queued during the hold must NOT survive ClearEstop
// (#184). Compute() short-circuits into ComputeEstop() before the normal drain,
// and ClearEstop() does not bump the activation generation — so without the drain
// in the E-STOP branch a stale pre-hold command passes IsCurrentGeneration on the
// first recovery tick and overwrites the measured-pose re-seed. Mirror of the OSC
// regression (OSC.EstopDrainsPendingTargetsBeforeRecovery).
//
// Detection is on diag.pose_error (‖e‖ of the re-seeded setpoint vs the measured
// pose), NOT on the joint torque: the recovery re-seed sets X_d = X_meas ⇒ e = 0,
// so a leaked target makes e ≠ 0 directly. A torque assertion would be masked —
// a leaked target far enough to matter latches SAFE_STOP, whose gravity-comp hold
// at rest also reads as ĝ(q). diag.pose_error is recorded before the SAFE_STOP
// step and preserved through the ComputeEstop handoff, so it survives that mask.
TEST(TaskImpedance, EstopDrainsPendingTargetsBeforeRecovery) {
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));  // q̇ = 0
  const auto seed = ctrl.Compute(state);                      // seeds X_d = X_meas ⇒ e = 0

  // Queue a task goal 0.1 m off in +x *while E-STOPped*.
  ctrl.TriggerEstop();
  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.1,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);  // E-STOP hold tick: must drain the queued target
  ctrl.ClearEstop();

  // Recovery tick: X_d re-seeds to the measured pose ⇒ ‖e‖ ≈ 0. A leaked stale
  // target overwrites the re-seed, so ‖e‖ would jump to the queued offset.
  (void)ctrl.Compute(state);
  const auto diag = ctrl.GetDiagnosticsForTesting();
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(diag.pose_error[static_cast<std::size_t>(i)], 0.0, 1e-6)
        << "axis " << i << ": stale E-STOP target leaked into the recovery re-seed";
}

// ── T5.1 flavor: commands stay finite across random configurations ──────────
TEST(TaskImpedance, OutputAlwaysFinite) {
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 20.0;
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);
  for (double a : {-1.2, -0.4, 0.0, 0.7, 1.5}) {
    auto state = MakeState(7, std::vector<double>(7, a), std::vector<double>(7, 0.05));
    const auto out = ctrl.Compute(state);
    for (int i = 0; i < 7; ++i)
      EXPECT_TRUE(std::isfinite(out.devices[0].commands[static_cast<std::size_t>(i)]))
          << "a=" << a << " joint " << i;
  }
}

// ── T7.1: Compute() allocates nothing (from the very first call) ────────────
TEST(TaskImpedance, ComputeIsAllocationFree) {
  const std::vector<double> q(7, 0.15);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 40.0;  // exercise the nullspace / DLS path too
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);
  auto state = MakeState(7, q, std::vector<double>(7, 0.02));
  // Measure the FIRST Compute (spec T7.1: from the first call). RAII rather than
  // a bare arm/disarm pair: an ASSERT_* added inside the region returns from the
  // test, and a bare disarm line would then never run — leaving counting on for
  // every later test in the binary, where nothing reads it.
  {
    rtc::testing::ScopedAllocGate gate;
    const auto out = ctrl.Compute(state);
    EXPECT_EQ(gate.count(), 0u) << "Compute() touched the heap on the RT path";
    EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  }
}

// ── Sign of the stiffness law: +K_p·e restores TOWARD the goal (§6.2) ────────
// The rest of the suite pins only e=0 cases and equality/magnitude, so a sign
// flip to −K_p·e (an unstable, repelling law) would pass all of them. Command a
// pure +x translation offset (TRANSLATION_ONLY so no rotation task force can
// saturate the joints and corrupt the reconstruction) and recover the 3D task
// force from the gravity-free joint torque via J_transᵀ (rebuilt independently
// from the URDF): a restoring law puts f_trans along +x (toward the goal); a
// flipped one puts it along −x.
TEST(TaskImpedance, StiffnessSignRestoresTowardGoal) {
  const std::vector<double> q(7, 0.2);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 30.0;  // required for TRANSLATION_ONLY (§6.1); posture error=0 ⇒ inert
  gains.activation_ramp_time = 0.0;  // full gain on the offset tick
  gains.max_torque_rate = 1e12;      // disable slew limiting so τ_task = J_transᵀ f exactly
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kTranslationOnly);
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));  // q̇ = 0 ⇒ ė = 0
  const auto seed = ctrl.Compute(state);                      // seeds X_d = X_meas ⇒ e = 0
  const double px = seed.actual_task_positions[0];
  const double py = seed.actual_task_positions[1];
  const double pz = seed.actual_task_positions[2];

  // Goal +δ along world x; orientation is not in the task under TRANSLATION_ONLY.
  const double dx = 0.03;
  const std::array<double, 6> t{px + dx, py, pz, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(t.data(), t.size()));
  const auto out = ctrl.Compute(state);

  // Independent J_trans (top 3 rows, LWA) at q, same tip frame the controller uses.
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = Urdf7();
  cfg.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
  rtc_urdf_bridge::RtModelHandle handle(builder.GetFullModel());
  handle.ComputeJacobians(std::span<const double>(q.data(), q.size()));
  const auto tip = static_cast<pinocchio::FrameIndex>(builder.GetFullModel()->nframes - 1);
  Eigen::MatrixXd jac(6, 7);
  handle.GetFrameJacobian(tip, pinocchio::LOCAL_WORLD_ALIGNED, jac);
  const Eigen::MatrixXd j_trans = jac.topRows(3);  // 3×nv

  // τ_task = τ − ĝ(q) = J_transᵀ f_trans (ė = 0, posture error = 0 ⇒ no nullspace).
  // Recover the 3D task force: f_trans = (J_trans J_transᵀ)⁻¹ J_trans τ_task.
  const auto grav = GravityAt(Urdf7(), q);
  Eigen::VectorXd tau_task(7);
  for (int i = 0; i < 7; ++i)
    tau_task(i) =
        out.devices[0].commands[static_cast<std::size_t>(i)] - grav[static_cast<std::size_t>(i)];
  const Eigen::Vector3d f_trans = (j_trans * j_trans.transpose()).ldlt().solve(j_trans * tau_task);

  EXPECT_GT(f_trans(0), 0.0)
      << "stiffness law repels instead of restoring — sign of +K_p·e is flipped";
  EXPECT_GT(f_trans(0), std::abs(f_trans(1)));  // force is dominantly along the offset axis
  EXPECT_GT(f_trans(0), std::abs(f_trans(2)));
}

// ── §6.1/§10.6: under TRANSLATION_ONLY a large ORIENTATION error (left to the
// soft nullspace by design) must NOT latch SAFE_STOP — the pose-error fault is
// bounded to the regulated translation DoF only. Folding the full 6D norm (the
// bug) latches SAFE_STOP here while translation tracking is perfect. ──────────
TEST(TaskImpedance, TranslationOnlyRotationDriftDoesNotLatch) {
  const std::vector<double> q(7, 0.2);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 50.0;         // required for TRANSLATION_ONLY (§6.1)
  gains.activation_ramp_time = 0.0;  // reach RUNNING immediately
  gains.pose_error_limit = 0.3;      // low bound: rotation error alone would exceed it
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kTranslationOnly);
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  const auto seed = ctrl.Compute(state);  // seeds X_d = X_meas
  const double px = seed.actual_task_positions[0];
  const double py = seed.actual_task_positions[1];
  const double pz = seed.actual_task_positions[2];

  // Same translation (‖e_trans‖ ≈ 0), orientation rotated ~π about Z (‖e_rot‖ ≫ 0.3).
  const std::array<double, 6> t{px, py, pz, 0.0, 0.0, 3.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(t.data(), t.size()));
  for (int k = 0; k < 5; ++k)
    (void)ctrl.Compute(state);  // several ticks — a spurious latch would stick

  const auto diag = ctrl.GetDiagnosticsForTesting();
  EXPECT_NE(diag.state, static_cast<std::uint8_t>(rtc::compliance::ComplianceState::kSafeStop))
      << "orientation drift spuriously latched SAFE_STOP under TRANSLATION_ONLY";
  EXPECT_TRUE(diag.control_valid) << "controller fell back to the E-STOP hold on rotation drift";
}

// ── §10.6 fault → SAFE_STOP latch → ResetFault recovery ─────────────────────
// A critical fault (pose error past its bound) must latch SAFE_STOP: the latch
// persists even after the fault itself clears, and only ResetFault() returns the
// controller to control (re-seeding from the measured state). Covers deferred F4
// and exercises the F2-guarded non-redundant path (Urdf6, nv == m ⇒ no M).
TEST(TaskImpedance, SafeStopLatchesAndResetFaultRecovers) {
  using rtc::compliance::ComplianceState;
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;  // reach RUNNING on the first tick
  gains.pose_error_limit = 0.05;     // a 0.5 m commanded step blows past this
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));  // q̇ = 0
  const auto seed = ctrl.Compute(state);                      // seeds X_d = X_meas ⇒ e = 0
  ASSERT_NE(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop));

  // Command a large translation step ⇒ ‖e‖ ≫ pose_error_limit ⇒ critical fault.
  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.5,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);
  const auto d_latched = ctrl.GetDiagnosticsForTesting();
  EXPECT_EQ(d_latched.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop));
  EXPECT_FALSE(d_latched.control_valid) << "SAFE_STOP must hand off to the E-STOP hold";

  // Further ticks re-seed X_d to the measured pose (the fault clears), yet the
  // latch must hold — only ResetFault() may leave SAFE_STOP.
  for (int k = 0; k < 3; ++k)
    (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "SAFE_STOP unlatched without ResetFault()";

  // Recover: ResetFault() → HOLDING → re-seed measured (e = 0) → RUNNING, and the
  // command returns to the gravity hold.
  ctrl.ResetFault();
  const auto out = ctrl.Compute(state);
  const auto d_recovered = ctrl.GetDiagnosticsForTesting();
  EXPECT_NE(d_recovered.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop));
  EXPECT_TRUE(d_recovered.control_valid) << "controller did not resume after ResetFault()";
  const auto grav = GravityAt(Urdf6(), q);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6)
        << "joint " << i << " did not return to the gravity hold after recovery";
}

// ── F1: a global E-STOP over a latched SAFE_STOP must report the real state ──
// The global-E-STOP early return once published a zero-constructed Diagnostics
// (state = HOLDING), masking a latched controller-local SAFE_STOP/DEGRADED.
TEST(TaskImpedance, GlobalEstopSurfacesLatchedSafeStop) {
  using rtc::compliance::ComplianceState;
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.pose_error_limit = 0.05;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  const auto seed = ctrl.Compute(state);
  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.5,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);  // latch SAFE_STOP
  ASSERT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop));

  ctrl.TriggerEstop();
  (void)ctrl.Compute(state);  // global-E-STOP early-return path
  const auto diag = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(diag.estopped);
  EXPECT_EQ(diag.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "global E-STOP masked the latched SAFE_STOP as HOLDING (F1)";
}

// ── F2: the non-redundant path (nv == m ⇒ no nullspace, no M) stays valid and
// heap-free — M(q)/its Cholesky are now skipped entirely there. ──────────────
TEST(TaskImpedance, NonRedundantPathValidAndAllocationFree) {
  const std::vector<double> q(6, 0.15);
  TaskImpedanceController::Gains gains;  // default nullspace gains ⇒ inactive at nv == m
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.02));
  {
    rtc::testing::ScopedAllocGate gate;
    const auto out = ctrl.Compute(state);  // first call, from the very first tick
    EXPECT_EQ(gate.count(), 0u) << "non-redundant Compute() touched the heap";
    EXPECT_TRUE(ctrl.GetDiagnosticsForTesting().control_valid)
        << "non-redundant path must stay valid without the M factorization";
    for (int i = 0; i < 6; ++i)
      EXPECT_TRUE(std::isfinite(out.devices[0].commands[static_cast<std::size_t>(i)]));
  }
}

// ── NUM-1 (#257): the λ_max floor has to sit where the DLS solve USES it ─────
TEST(TaskImpedance, MaxDampingFloorSurvivesSetGains) {
  // set_gains() writes the Gains POD straight into the SeqLock, so a
  // configure-time-only floor is bypassable by every caller holding the handle —
  // the same hole 0a61aaf closed for the operational-space controller and
  // 2ee9662 for the admittance one.
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 30.0;  // nv(7) > m(6) with a live nullspace ⇒ Λ_S and the DLS run
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);

  ctrl.LoadConfig(YAML::Load("max_damping: 0.0\n"));
  ASSERT_GT(ctrl.get_gains().max_damping, 0.0) << "LoadConfig must floor λ_max as well";

  auto g = ctrl.get_gains();
  g.singularity_threshold = 10.0;  // σ₀ above this arm's σ_min → the DLS ramp is always armed
  g.max_damping = 0.0;             // exactly what LoadConfig just floored away
  ctrl.set_gains(g);

  const std::vector<double> q(7, 0.2);
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);

  const auto d = ctrl.GetDiagnosticsForTesting();
  ASSERT_LT(d.sigma_min, g.singularity_threshold) << "σ₀ must be armed for λ² to be observable";
  ASSERT_GT(d.sigma_min, g.singularity_critical) << "posture must stay out of SAFE_STOP";
  EXPECT_GT(d.lambda_sq, 0.0) << "λ_max = 0 reached the DLS solve — Λ_S⁻¹ is left unregularised";
}

// ── F5 (#236 E-8): the primary device's joint state must be checked ──────────
//
// This controller shipped without the gate its two siblings carry
// (TaskAdmittanceController, CascadedComplianceController). Unread channels
// default to 0, so an unchecked tick evaluates FK, the Jacobian and the whole
// §6.2 law at the ZERO configuration and emits a full-arm pull toward the
// origin — every number finite, no fault raised, nothing for the CM's
// actuator-boundary validator to reject.

// Gains that retire the ramp so the FSM reaches DEGRADED on the first degrading
// tick rather than sitting in HOLDING (the lattice honours a degrade cause at
// the HOLDING→RUNNING edge, which needs ramp_done).
TaskImpedanceController::Gains NoRampGains() {
  TaskImpedanceController::Gains g;
  g.activation_ramp_time = 0.0;
  return g;
}

TEST(TaskImpedance, InvalidDeviceStateEmitsNoCommandAndDegrades) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController ctrl(Urdf6(), NoRampGains(), Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));

  // Positive control: the healthy tick DOES command, so the zero-length below is
  // the gate firing and not an inert fixture.
  const auto healthy = ctrl.Compute(state);
  ASSERT_EQ(healthy.devices[0].num_channels, 6);
  ASSERT_TRUE(ctrl.GetDiagnosticsForTesting().control_valid);

  state.devices[0].valid = false;
  {
    rtc::testing::ScopedAllocGate gate;
    const auto out = ctrl.Compute(state);  // RT tick like any other — no heap
    EXPECT_EQ(gate.count(), 0u) << "the device-invalid path touched the heap";
    EXPECT_EQ(out.devices[0].num_channels, 0) << "commanded an arm whose state it could not read";
  }
  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_FALSE(d.control_valid);
  EXPECT_EQ(d.state, static_cast<std::uint8_t>(rtc::compliance::ComplianceState::kDegraded));
}

// The half of the gate that is actually reachable in production: a device
// narrower than the model. `valid` latches true on a backend's first state
// message and is never cleared, and CM's startup gate refuses to run Compute()
// until every group has reported — so `!valid` needs a group with no backend at
// all, while a channel-count mismatch is a plain configuration fault that never
// heals.
TEST(TaskImpedance, TooFewDeviceChannelsIsTreatedAsNoJointState) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController ctrl(Urdf6(), NoRampGains(), Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  state.devices[0].num_channels = 4;  // model has 6
  const auto out = ctrl.Compute(state);
  EXPECT_EQ(out.devices[0].num_channels, 0);
  EXPECT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(rtc::compliance::ComplianceState::kDegraded));

  // Non-vacuity: reconstruct what the UNGATED path would have emitted, so this
  // test cannot pass against a gate that merely truncated the command. The two
  // unreported channels read as 0, so the law would run at [0.3,0.3,0.3,0.3,0,0]
  // while X_d was seeded at [0.3]*6. Fed as a VALID state to an identically
  // seeded controller, that is a materially different torque — and every number
  // in it is finite, which is why nothing downstream rejects it.
  //
  // The observable here is ORIENTATION, not gravity: serial_6dof.urdf is a
  // coaxial z-axis stack, so ĝ(q) ≡ 0 and the TCP translation is the same point
  // on the z axis at every configuration. Only the TCP yaw moves with q (by the
  // 0.6 rad the two zeroed joints carry, well inside pose_error_limit).
  TaskImpedanceController ungated(Urdf6(), NoRampGains(), Sel::kFullSe3);
  auto seed_state = MakeState(6, q, std::vector<double>(6, 0.0));
  const auto seeded = ungated.Compute(seed_state);  // X_d := the true pose
  std::vector<double> q_partial_zero(q);
  q_partial_zero[4] = 0.0;
  q_partial_zero[5] = 0.0;
  auto zero_tail = MakeState(6, q_partial_zero, std::vector<double>(6, 0.0));
  const auto pulled = ungated.Compute(zero_tail);
  double delta = 0.0;
  for (std::size_t i = 0; i < 6; ++i)
    delta += std::abs(pulled.devices[0].commands[i] - seeded.devices[0].commands[i]);
  EXPECT_GT(delta, 1e-3) << "fixture cannot distinguish the ZERO-configuration evaluation";
}

// The E-STOP hold reads the same device the control law does, so it needs the
// same gate: ĝ(q) − D·q̇ is a hold only when q and q̇ are real. The rejected
// alternative (#236 E-8) was emitting nc0 zeros — that is a REAL 0 N·m write,
// which on a torque-mode arm is a drop, not a stop. Silence is the one answer
// this controller gives to "the device is unreadable", on both paths.
TEST(TaskImpedance, EstopHoldWithUnreadableDeviceEmitsNoCommand) {
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController ctrl(Urdf6(), NoRampGains(), Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  // Positive control: the E-STOP hold on a READABLE device still emits ĝ(q), so
  // the zero-length below is the new gate and not a dead E-STOP path.
  ctrl.TriggerEstop();
  const auto held = ctrl.Compute(state);
  ASSERT_EQ(held.devices[0].num_channels, 6);
  const auto grav = GravityAt(Urdf6(), q);
  ASSERT_NEAR(held.devices[0].commands[0], grav[0], 1e-6);

  state.devices[0].valid = false;
  const auto out = ctrl.Compute(state);
  EXPECT_EQ(out.devices[0].num_channels, 0)
      << "the E-STOP hold emitted a torque derived from a joint state it could not read";
  EXPECT_FALSE(ctrl.GetDiagnosticsForTesting().control_valid);
}

// A target pushed while the joint state was unreadable must not outlive the
// outage. The device-invalid path inherits the E-STOP path's re-seed contract,
// and neither bumps the activation generation — so without the drain a queued
// command still passes IsCurrentGeneration and overwrites the measured-pose
// re-seed on the first recovered tick. Detection is on diag.pose_error for the
// same reason the E-STOP drain test uses it: the recovery re-seed sets
// X_d = X_meas ⇒ e = 0, so a leak shows up directly, while a torque assertion
// would be masked by the gravity-comp hold a large enough leak latches into.
TEST(TaskImpedance, TargetsQueuedWhileTheDeviceIsInvalidDoNotSurviveRecovery) {
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController ctrl(Urdf6(), NoRampGains(), Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  const auto seed = ctrl.Compute(state);  // X_d := the measured pose ⇒ e = 0

  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.1,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  state.devices[0].valid = false;
  for (int k = 0; k < 8; ++k)  // more than kPendingTargetDepth
    ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);  // device-invalid tick: must drain

  state.devices[0].valid = true;
  const auto recovered = ctrl.Compute(state);
  ASSERT_GT(recovered.devices[0].num_channels, 0) << "the recovery tick emitted nothing";
  const auto d = ctrl.GetDiagnosticsForTesting();
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(d.pose_error[static_cast<std::size_t>(i)], 0.0, 1e-6)
        << "axis " << i << ": a target queued during the outage overwrote the re-seed";

  // Positive control: the very same target, pushed while the device is healthy,
  // DOES move the setpoint — so the equality above is a drain and not an inert
  // SetDeviceTarget or an unobservable offset.
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);
  EXPECT_GT(std::abs(ctrl.GetDiagnosticsForTesting().pose_error[0]), 1e-3);
}

}  // namespace
