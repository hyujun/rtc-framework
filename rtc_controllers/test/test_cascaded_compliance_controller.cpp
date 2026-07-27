// ── CascadedComplianceController (spec §7.6) ─────────────────────────────────
// Slice-4 surface: the outer admittance → inner impedance cascade itself. What
// needs pinning here is everything the two loops do TOGETHER — the wrench is
// spent exactly once (MUST-4), the bandwidth report (MUST-1), the return /
// hand-guiding behaviours of the compliant frame (MUST-3) — plus the safety
// contracts the controller inherits (torque E-STOP hold, staleness → DEGRADED,
// re-seed boundaries) and the three config/validity classes PR #256 found in the
// copied skeleton (F5 device validity, F7 config shape, F8 thresholds).
//
// The pieces each loop is built from are pinned model-free elsewhere:
// admittance_integrator + differential algebra in test_admittance_core.cpp, the
// §6.2 law in test_compliance_core.cpp (ImpedanceLaw), the wrench pipeline and
// state machine in test_compliance_core.cpp. What cannot be tested there is the
// COMPOSITION, which is what this file is about.
//
// Zero-allocation is checked by GLOBAL operator-new interposition (same reason
// as the other controller suites: Compute() lives in another TU, so a same-TU
// Eigen guard would observe nothing).
#include "rtc_controllers/direct/cascaded_compliance_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <span>
#include <string>
#include <vector>

// That interposition is rtc::testing::ScopedAllocGate
// (rtc_controllers/testing/alloc_gate.hpp) — shared with every other suite that
// gates an RT path, rather than a per-file copy of the operator-new replacement.

namespace {

using rtc::CascadedComplianceController;
using rtc::compliance::ComplianceState;
using rtc::compliance::kWrenchDim;
using rtc::compliance::Wrench6;

constexpr double kDt = 0.002;
constexpr double kRateHz = 1.0 / kDt;

std::string Urdf6() {
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

std::string Urdf7() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

rtc::ControllerState MakeState(int nj, const std::vector<double>& q,
                               const std::vector<double>& qd) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = kDt;
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

void Send(CascadedComplianceController& c, const Wrench6& w) {
  c.SetExternalWrench(std::span<const double, kWrenchDim>(w.data(), kWrenchDim));
}

// Transparent wrench chain (no deadband / filter / bias average) so what the
// caller publishes is what the outer loop integrates, up to the Ad^{-T} frame
// map. Each conditioning stage has its own test in test_compliance_core.cpp;
// mixing them in here would only make failures ambiguous.
std::string TransparentWrenchYaml(const std::string& extra = {}) {
  return R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 0
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)" + extra;
}

// Gains that keep the §10.5 safety layer INERT, so the emitted torque is exactly
// the control law and the reference reconstructions below are exact: no ramp, no
// slew bound, no joint-limit repulsion (the ±1e9 default limits already put the
// arm outside the margin band, but the gains are zeroed to say so on purpose).
CascadedComplianceController::Gains InertSafetyGains() {
  CascadedComplianceController::Gains g;
  g.activation_ramp_time = 0.0;
  g.max_torque_rate = 1e12;
  g.joint_limit_kp = 0.0;
  g.joint_limit_kd = 0.0;
  g.joint_limit_margin = 0.0;
  return g;
}

// An independent model handle at the same q — the reference side of every
// reconstruction below (never the controller's own state).
struct Reference {
  explicit Reference(const std::string& urdf) {
    rtc_urdf_bridge::ModelConfig cfg;
    cfg.urdf_path = urdf;
    cfg.root_joint_type = "fixed";
    builder = std::make_unique<rtc_urdf_bridge::PinocchioModelBuilder>(cfg);
    handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(builder->GetFullModel());
    tip = static_cast<pinocchio::FrameIndex>(builder->GetFullModel()->nframes - 1);
  }

  Eigen::VectorXd Gravity(const std::vector<double>& q) {
    handle->ComputeGeneralizedGravity(std::span<const double>(q.data(), q.size()));
    return handle->GetGeneralizedGravity();
  }

  Eigen::MatrixXd Jacobian(const std::vector<double>& q) {
    handle->ComputeJacobians(std::span<const double>(q.data(), q.size()));
    Eigen::MatrixXd J(6, handle->nv());
    handle->GetFrameJacobian(tip, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
  }

  // Recover the task force from an emitted torque: τ = Jᵀf + ĝ ⇒ f = (JJᵀ)⁻¹J(τ−ĝ).
  // Exact whenever J has full row rank and the safety layer was inert.
  Eigen::Matrix<double, 6, 1> TaskForce(const std::vector<double>& q,
                                        const rtc::ControllerOutput& out) {
    const Eigen::MatrixXd J = Jacobian(q);
    const Eigen::VectorXd g = Gravity(q);
    Eigen::VectorXd dtau(handle->nv());
    for (int i = 0; i < handle->nv(); ++i)
      dtau(i) = out.devices[0].commands[static_cast<std::size_t>(i)] - g(i);
    const Eigen::MatrixXd JJt = J * J.transpose();
    return JJt.ldlt().solve(J * dtau);
  }

  std::unique_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  pinocchio::FrameIndex tip{0};
};

Eigen::Matrix<double, 6, 1> ToVec6(const std::array<double, 6>& a) {
  Eigen::Matrix<double, 6, 1> v;
  for (int i = 0; i < 6; ++i)
    v(i) = a[static_cast<std::size_t>(i)];
  return v;
}

// ── Baseline ────────────────────────────────────────────────────────────────

// With no force published, X_c ≡ X_d ≡ the measured pose and q̇ = 0, so both
// loops are inert and the whole controller reduces to gravity compensation. A
// cascade that emits anything else at rest is pushing on nothing.
TEST(CascadedCompliance, GravityOnlyAtRest) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));

  const auto out = ctrl.Compute(state);
  ASSERT_EQ(out.command_type, rtc::CommandType::kTorque);
  Reference ref(Urdf6());
  const Eigen::VectorXd g = ref.Gravity(q);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)], g(i), 1e-9) << "joint " << i;

  const auto d = ctrl.GetDiagnosticsForTesting();
  for (double v : d.compliant_deviation)
    EXPECT_EQ(v, 0.0) << "the compliant frame moved with no force on it";
  EXPECT_TRUE(d.control_valid);
}

// §11.4.1 P3 — the cascade FOLLOWS a push (that is what makes it an admittance
// on the outside), and it does so through the compliant frame: the frame yields
// along the measured force, and the emitted torque then drives the arm after the
// frame. Both halves are asserted against the LWA force direction the controller
// itself reports, so the test does not depend on the fixture's tip orientation.
TEST(CascadedCompliance, PushYieldsTheFrameAndTheTorqueChasesIt) {
  const std::vector<double> q(7, 0.25);
  CascadedComplianceController ctrl(Urdf7(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);  // seed X_d = X_meas, X_c = X_d

  const Wrench6 push{12.0, -8.0, 5.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 60; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  const auto d = ctrl.GetDiagnosticsForTesting();
  const Eigen::Vector3d f_lwa = ToVec6(d.wrench_lwa).head<3>();
  ASSERT_GT(f_lwa.norm(), 1.0) << "no force reached the outer loop — the test would be vacuous";

  const Eigen::Vector3d dev = ToVec6(d.compliant_deviation).head<3>();
  ASSERT_GT(dev.norm(), 1e-6) << "the compliant frame never moved";
  EXPECT_GT(dev.dot(f_lwa), 0.0) << "the compliant frame moved AGAINST the push (§7.2 sign)";

  Send(ctrl, push);
  const auto out = ctrl.Compute(state);
  Reference ref(Urdf7());
  const Eigen::Matrix<double, 6, 1> f_task = ref.TaskForce(q, out);
  EXPECT_GT(f_task.head<3>().dot(f_lwa), 0.0)
      << "the inner loop pulled away from the compliant frame it is supposed to track";
}

// ── §7.6 MUST-4: the wrench is consumed exactly once ────────────────────────

// The structural claim (D19: no inertia-shaping term exists, so f_ext cannot
// re-enter the inner law) is only worth asserting if the alternative is visibly
// different — otherwise "it does not double-count" pins nothing. So the emitted
// task force is compared against BOTH candidate laws computed in the test: the
// single-count one it must equal, and the double-count one it must not. The gap
// between them IS f_ext, and the test refuses to run if that gap is small.
TEST(CascadedCompliance, InnerLoopDoesNotSpendTheWrenchASecondTime) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);

  const Wrench6 push{15.0, -10.0, 6.0, 0.5, -0.3, 0.2};
  for (int k = 0; k < 80; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  Send(ctrl, push);
  const auto out = ctrl.Compute(state);
  const auto d = ctrl.GetDiagnosticsForTesting();

  // The two candidate laws. q̇ = 0 ⇒ ν = 0, so ė = ν_c − 0 = ν_c.
  const Eigen::Matrix<double, 6, 1> e = ToVec6(d.pose_error);
  const Eigen::Matrix<double, 6, 1> nu_c = ToVec6(d.compliant_velocity);
  const Eigen::Matrix<double, 6, 1> f_ext = ToVec6(d.wrench_lwa);
  Eigen::Matrix<double, 6, 1> f_single;
  for (int i = 0; i < 6; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double kp = (i < 3) ? gains.impedance.kp_pos[ui] : gains.impedance.kp_rot[ui - 3];
    const double kd = (i < 3) ? gains.impedance.kd_pos[ui] : gains.impedance.kd_rot[ui - 3];
    f_single(i) = kp * e(i) + kd * nu_c(i);
  }
  const Eigen::Matrix<double, 6, 1> f_double = f_single + f_ext;

  Reference ref(Urdf7());
  const Eigen::Matrix<double, 6, 1> f_actual = ref.TaskForce(q, out);

  ASSERT_GT((f_double - f_single).norm(), 1.0)
      << "the two candidate laws are indistinguishable here — the assertion would be vacuous";
  EXPECT_LT((f_actual - f_single).norm(), 1e-6)
      << "emitted task force is not the §6.2 law on (X_c, ν_c)";
  EXPECT_GT((f_actual - f_double).norm(), 1.0)
      << "the wrench was counted twice — §7.6 MUST-4 (outer already spent it)";
}

// ── §7.6 MUST-1: bandwidth separation is REPORTED, never enforced ───────────

TEST(CascadedCompliance, BandwidthFlagStaysClearWhenTheInnerLoopIsFaster) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.impedance.kp_pos = {800.0, 800.0, 800.0};
  gains.impedance.kp_rot = {80.0, 80.0, 80.0};
  gains.admittance.stiffness = {20.0, 20.0, 20.0, 1.0, 1.0, 1.0};
  gains.admittance.inertia = {4.0, 4.0, 4.0, 0.2, 0.2, 0.2};
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);

  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(std::isfinite(d.bandwidth_ratio)) << "no axis was evaluated — the flag says nothing";
  EXPECT_GE(d.bandwidth_ratio, gains.min_bandwidth_ratio);
  EXPECT_FALSE(d.bandwidth_ratio_low);
}

// The same fixture with the two bands swapped: the flag must rise, and — this is
// the half that matters — NOTHING else changes. A sluggish inner loop is a
// tuning statement, and D20 keeps it out of the fault lattice precisely so it
// cannot stop a robot that is merely soft.
TEST(CascadedCompliance, BandwidthFlagRisesForACoupledCascadeWithoutFaulting) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.impedance.kp_pos = {5.0, 5.0, 5.0};
  gains.impedance.kp_rot = {0.5, 0.5, 0.5};
  gains.admittance.stiffness = {4000.0, 4000.0, 4000.0, 200.0, 200.0, 200.0};
  gains.admittance.inertia = {2.0, 2.0, 2.0, 0.05, 0.05, 0.05};
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);

  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(d.bandwidth_ratio_low) << "ω_i/ω_a = " << d.bandwidth_ratio << " went unreported";
  EXPECT_LT(d.bandwidth_ratio, gains.min_bandwidth_ratio);
  EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "a low bandwidth ratio stopped the robot — it is a diagnostic, not a fault (D20)";
  EXPECT_TRUE(d.control_valid);
}

// Hand-guiding (K_p^a = 0 on every axis) has no outer restoring frequency, so
// there is no ratio to report. Reported as infinite rather than as a violation:
// flagging the one configuration §7.6 MUST-3 explicitly allows would train
// operators to ignore the flag.
TEST(CascadedCompliance, BandwidthFlagIsSilentForHandGuiding) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.admittance.stiffness = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);

  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_FALSE(d.bandwidth_ratio_low);
  EXPECT_FALSE(std::isfinite(d.bandwidth_ratio));
}

// A zero inner stiffness is the WORST separation there is — ω_i = 0 under an
// outer loop that has a restoring frequency — and `inner.kp_pos: [0,0,0]` passes
// LoadConfig (>= 0). Excluding those axes as "not evaluable" removed the most
// coupled axis from a min() whose entire job is to report the most coupled axis,
// so the cascade this diagnostic exists to catch reported ∞ / not-low.
TEST(CascadedCompliance, BandwidthFlagRisesForAZeroInnerStiffnessAxis) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  // Five axes deliberately well separated; only Z has no inner stiffness.
  gains.impedance.kp_pos = {800.0, 800.0, 0.0};
  gains.impedance.kp_rot = {80.0, 80.0, 80.0};
  gains.admittance.stiffness = {20.0, 20.0, 20.0, 1.0, 1.0, 1.0};
  gains.admittance.inertia = {4.0, 4.0, 4.0, 0.2, 0.2, 0.2};
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);

  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_EQ(d.bandwidth_ratio, 0.0) << "the ω_i = 0 axis did not reach the worst-axis min()";
  EXPECT_TRUE(d.bandwidth_ratio_low);
  EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "still a diagnostic, not a fault (D20)";
}

// The ratio compares two GAIN SETS, so it has to follow a runtime retune. It was
// written on the seeding tick and never again, so `set_gains()` left a figure
// describing gains that no longer ran — and the operator reading it has no way
// to tell.
TEST(CascadedCompliance, BandwidthReportFollowsARuntimeRetune) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.impedance.kp_pos = {800.0, 800.0, 800.0};
  gains.impedance.kp_rot = {80.0, 80.0, 80.0};
  gains.admittance.stiffness = {20.0, 20.0, 20.0, 1.0, 1.0, 1.0};
  gains.admittance.inertia = {4.0, 4.0, 4.0, 0.2, 0.2, 0.2};
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)ctrl.Compute(state);
  ASSERT_FALSE(ctrl.GetDiagnosticsForTesting().bandwidth_ratio_low);

  // Retune into a coupled cascade WITHOUT re-activating — no re-seed happens.
  auto coupled = gains;
  coupled.impedance.kp_pos = {5.0, 5.0, 5.0};
  coupled.impedance.kp_rot = {0.5, 0.5, 0.5};
  coupled.admittance.stiffness = {4000.0, 4000.0, 4000.0, 200.0, 200.0, 200.0};
  ctrl.set_gains(coupled);
  (void)ctrl.Compute(state);

  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(d.bandwidth_ratio_low)
      << "the report still describes the retired gains (ratio = " << d.bandwidth_ratio << ")";
  EXPECT_LT(d.bandwidth_ratio, coupled.min_bandwidth_ratio);
}

// ── #277: the posture floor this controller already had, now at the point of
// use as well ──────────────────────────────────────────────────────────────
//
// LoadConfig has floored both posture gains since the controller shipped — it
// was the one of the five that got this right, and #277 converged the other
// four onto it. What no controller had was the other half of NUM-1: set_gains()
// writes the Gains POD straight into the SeqLock, so configure-time clamping
// alone is bypassable by every caller holding the handle. τ₀ with K_pⁿ < 0
// pushes away from the posture reference and K_dⁿ < 0 injects energy, and Nᵀ
// keeps both off the Cartesian task, so neither raises a fault.
TEST(CascadedCompliance, PostureFloorSurvivesSetGains) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.nullspace_kp = 5.0;  // nv(7) > 6 ⇒ a genuinely redundant null space
  gains.nullspace_kd = 1.0;
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));

  // Non-vacuity: the fixture opens the gate on positive gains, so a false below
  // means the floor closed it and not that it was never open.
  (void)ctrl.Compute(state);
  ASSERT_TRUE(ctrl.GetDiagnosticsForTesting().nullspace_active)
      << "the fixture never opens the posture gate — the negative case proves nothing";

  // The configure-time floor, pinned here because the other four controllers'
  // tests now assert against this one as the reference form.
  ctrl.LoadConfig(
      YAML::Load(TransparentWrenchYaml("nullspace_stiffness: -5.0\nnullspace_damping: -1.0\n")));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().nullspace_kp, 0.0);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().nullspace_kd, 0.0);

  // The bypass. The gate is `nv > 6 && (kp != 0 || kd != 0)`, so flooring only
  // the law's arguments would hold it open and pay for the Λ/Nᵀ block every
  // tick to project an all-zero τ₀.
  auto negative = gains;
  negative.nullspace_kp = -5.0;
  negative.nullspace_kd = -1.0;
  ctrl.set_gains(negative);
  (void)ctrl.Compute(state);
  EXPECT_FALSE(ctrl.GetDiagnosticsForTesting().nullspace_active)
      << "a negative posture gain held the gate open — the floor is configure-only";
}

// ── §7.6 MUST-3: what the compliant frame does when the force stops ─────────

TEST(CascadedCompliance, StiffOuterLoopReturnsTheFrameToTheDesiredPose) {
  const std::vector<double> q(6, 0.3);
  auto gains = InertSafetyGains();
  gains.admittance.stiffness = {200.0, 200.0, 200.0, 5.0, 5.0, 5.0};
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  const Wrench6 push{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 200; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  const double pushed =
      ToVec6(ctrl.GetDiagnosticsForTesting().compliant_deviation).head<3>().norm();
  ASSERT_GT(pushed, 1e-3) << "the frame never yielded — the return assertion would be vacuous";

  // Force removed, producer still alive (zeros, not silence) so nothing degrades.
  const Wrench6 zero{};
  for (int k = 0; k < 4000; ++k) {
    Send(ctrl, zero);
    (void)ctrl.Compute(state);
  }
  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_LT(ToVec6(d.compliant_deviation).norm(), 0.05 * pushed)
      << "K_p^a > 0 must pull X_c back to X_d once the load is gone";
  EXPECT_TRUE(d.control_valid);
}

TEST(CascadedCompliance, ZeroOuterStiffnessKeepsTheFrameWhereTheForceLeftIt) {
  const std::vector<double> q(6, 0.3);
  auto gains = InertSafetyGains();
  gains.admittance.stiffness = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // hand-guiding
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  const Wrench6 push{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 200; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  const Eigen::Vector3d pushed =
      ToVec6(ctrl.GetDiagnosticsForTesting().compliant_deviation).head<3>();
  ASSERT_GT(pushed.norm(), 1e-3);

  const Wrench6 zero{};
  for (int k = 0; k < 4000; ++k) {
    Send(ctrl, zero);
    (void)ctrl.Compute(state);
  }
  const Eigen::Vector3d held =
      ToVec6(ctrl.GetDiagnosticsForTesting().compliant_deviation).head<3>();
  // The damper still bleeds off the velocity it had when the force stopped, so
  // the frame coasts a little further; what it must NOT do is spring back.
  EXPECT_GT(held.dot(pushed.normalized()), 0.9 * pushed.norm())
      << "K_p^a = 0 returned the frame anyway — hand-guiding is broken";
}

// ── Staleness / safety ──────────────────────────────────────────────────────

TEST(CascadedCompliance, StaleWrenchFadesAndDegradesWithoutSafeStop) {
  const std::vector<double> q(6, 0.3);
  auto gains = InertSafetyGains();
  gains.wrench.timeout = 0.01;       // 5 ticks
  gains.wrench.fadeout_time = 0.02;  // 10 further ticks
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  const Wrench6 push{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Send(ctrl, push);
  (void)ctrl.Compute(state);
  ASSERT_GT(std::abs(ctrl.GetDiagnosticsForTesting().wrench_lwa[0]), 1.0);

  for (int k = 0; k < 60; ++k) {
    (void)ctrl.Compute(state);
    EXPECT_NE(ctrl.GetDiagnosticsForTesting().state,
              static_cast<std::uint8_t>(ComplianceState::kSafeStop))
        << "a dead wrench source stopped the robot (tick " << k << ")";
  }
  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(d.wrench_stale);
  for (double v : d.wrench_lwa)
    EXPECT_EQ(v, 0.0) << "expired wrench held instead of faded to zero (§10.6 MUST)";
  EXPECT_EQ(d.state, static_cast<std::uint8_t>(ComplianceState::kDegraded));
  EXPECT_TRUE(d.control_valid) << "DEGRADED must keep controlling, not hand off to the hold";
}

// E-8: the hold is the TORQUE-domain ĝ(q) − D·q̇, not a position latch — this
// controller emits torques, and re-emitting a measured position would be a free
// joint dressed as a hold.
TEST(CascadedCompliance, EstopHoldsWithGravityCompensationAndDamping) {
  const std::vector<double> q(6, 0.3);
  const std::vector<double> qd(6, 0.15);
  auto gains = InertSafetyGains();
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, qd);
  (void)ctrl.Compute(state);

  ctrl.TriggerEstop();
  const auto out = ctrl.Compute(state);
  EXPECT_TRUE(ctrl.IsEstopped());
  Reference ref(Urdf6());
  const Eigen::VectorXd g = ref.Gravity(q);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                g(i) - gains.estop_damping * qd[static_cast<std::size_t>(i)], 1e-9)
        << "joint " << i;
  EXPECT_FALSE(ctrl.GetDiagnosticsForTesting().control_valid);
}

// A held tick is a discontinuity: the compliant frame accrued before it describes
// a load the controller is no longer answering, so the next controllable tick
// must start from X_c = X_d again. Without this, clearing an E-STOP hands the
// arm a torque toward wherever the frame had drifted.
TEST(CascadedCompliance, EstopAndReactivationCollapseTheCompliantFrame) {
  const std::vector<double> q(6, 0.3);
  auto gains = InertSafetyGains();
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  const Wrench6 push{25.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 100; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  ASSERT_GT(ToVec6(ctrl.GetDiagnosticsForTesting().compliant_deviation).norm(), 1e-3);

  ctrl.TriggerEstop();
  (void)ctrl.Compute(state);
  ctrl.ClearEstop();
  const auto out = ctrl.Compute(state);  // first controllable tick: re-seeds
  const auto d = ctrl.GetDiagnosticsForTesting();
  for (double v : d.compliant_deviation)
    EXPECT_EQ(v, 0.0) << "the compliant frame survived an E-STOP";
  for (double v : d.compliant_velocity)
    EXPECT_EQ(v, 0.0) << "the compliant frame kept its velocity across an E-STOP";
  // ...and with X_c back on the measured pose the torque is gravity again.
  Reference ref(Urdf6());
  const Eigen::VectorXd g = ref.Gravity(q);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)], g(i), 1e-9) << "joint " << i;
}

// The same contract on the deactivate/reactivate boundary the CM drives.
TEST(CascadedCompliance, ReactivationReseedsTheDesiredPoseAndTheFrame) {
  const std::vector<double> q(6, 0.3);
  auto gains = InertSafetyGains();
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);
  const Wrench6 push{25.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 100; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  ASSERT_GT(ToVec6(ctrl.GetDiagnosticsForTesting().compliant_deviation).norm(), 1e-3);

  ctrl.ResetTargetInitializationForTesting();
  (void)ctrl.Compute(state);  // NOTHING published after the reset
  const auto d = ctrl.GetDiagnosticsForTesting();
  for (double v : d.compliant_deviation)
    EXPECT_EQ(v, 0.0) << "reactivation inherited a deviation accrued before it";
  // The sample sitting in the slot at reset is disowned, not re-dated (D22), so
  // the outer loop starts from no force rather than from a possibly dead one.
  EXPECT_FALSE(d.wrench_valid) << "a pre-activation sample was consumed as fresh";
}

// ── §10.7 activation ramp: the outer loop keeps its own clock ───────────────

// BIAS_CALIBRATING emits a ZERO wrench for `bias_calibration_samples` producer
// samples — 100 by default, ≈1 s at 100 Hz, and it re-runs on every re-seed
// including after each E-STOP. On one shared clock that interval spends the
// whole activation ramp on a force that does not exist yet, so the first real
// sample enters the outer loop at α = 1: exactly the step the ramp exists to
// prevent, on the path where an operator's hand is on the arm.
TEST(CascadedCompliance, TheWrenchRampSurvivesALongBiasCalibration) {
  constexpr int kBiasSamples = 40;
  constexpr int kRampTicks = 10;  // deliberately shorter than the bias average
  constexpr int kForceTicks = 3;  // ticks of real force to compare over
  const std::vector<double> q(6, 0.3);
  const std::string yaml = R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 40
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)";

  // Deviation of the compliant frame after the bias average has committed and
  // `kForceTicks` ticks of real force have gone through the outer loop.
  auto deviation_after_calibration = [&](double ramp_time) {
    auto gains = InertSafetyGains();
    gains.activation_ramp_time = ramp_time;
    CascadedComplianceController ctrl(Urdf6(), gains);
    ctrl.SetControlRate(kRateHz);
    ctrl.LoadConfig(YAML::Load(yaml));
    auto state = MakeState(6, q, std::vector<double>(6, 0.0));
    // Calibrate at rest — a force present THROUGHOUT the average is what the
    // average subtracts, so it would be indistinguishable from a zero wrench.
    for (int k = 0; k < kBiasSamples; ++k) {
      Send(ctrl, Wrench6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      (void)ctrl.Compute(state);
    }
    for (int k = 0; k < kForceTicks; ++k) {
      Send(ctrl, Wrench6{30.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      (void)ctrl.Compute(state);
    }
    const auto d = ctrl.GetDiagnosticsForTesting();
    EXPECT_TRUE(d.bias_calibrated) << "the bias average did not commit — fixture is wrong";
    return ToVec6(d.compliant_deviation).norm();
  };

  const double unramped = deviation_after_calibration(0.0);
  const double ramped = deviation_after_calibration(kRampTicks * kDt);

  ASSERT_GT(unramped, 1e-9) << "the outer loop never responded — nothing to ramp";
  // With one clock these are EQUAL: activation_elapsed_ is already past the ramp
  // by the time the first force arrives.
  EXPECT_LT(ramped, 0.5 * unramped)
      << "the activation ramp was spent during BIAS_CALIBRATING (ramped=" << ramped
      << " unramped=" << unramped << ")";
  EXPECT_GT(ramped, 0.0) << "the ramp suppressed the outer loop entirely";
}

// The inner loop must NOT wait for the bias average: freezing one shared clock
// would leave the arm on gravity compensation alone — free-floating — for the
// whole averaging window, which is worse than the step it would prevent.
TEST(CascadedCompliance, TheInnerLoopEngagesDuringBiasCalibration) {
  const std::vector<double> q(6, 0.3);
  const std::string yaml = R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 40
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)";
  auto gains = InertSafetyGains();
  gains.activation_ramp_time = 10.0 * kDt;
  CascadedComplianceController ctrl(Urdf6(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(yaml));

  // Moving joints while the bias is still averaging: the §6.2 damping term is
  // the only thing resisting, and it is inside α_track.
  auto state = MakeState(6, q, std::vector<double>(6, 0.4));
  for (int k = 0; k < 20; ++k) {  // still well inside the 40-sample average
    Send(ctrl, Wrench6{30.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    (void)ctrl.Compute(state);
  }
  const auto d = ctrl.GetDiagnosticsForTesting();
  ASSERT_FALSE(d.bias_calibrated) << "the average finished early — fixture is wrong";
  const auto out = ctrl.Compute(state);

  Reference ref(Urdf6());
  const Eigen::VectorXd g = ref.Gravity(q);
  double delta = 0.0;
  for (int i = 0; i < 6; ++i)
    delta += std::abs(out.devices[0].commands[static_cast<std::size_t>(i)] - g(i));
  EXPECT_GT(delta, 1e-3) << "only gravity compensation was emitted while the bias averaged — the "
                            "arm is free-floating for the whole window";
}

// ── F5 (PR #256): the primary device's joint state must be checked ──────────

// Unread channels default to 0, so an unchecked tick evaluates FK, the Jacobian
// and both loops at the ZERO configuration and emits a full-arm pull toward the
// origin — with every number finite and no fault raised.
TEST(CascadedCompliance, InvalidDeviceStateEmitsNoCommandAndDegrades) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  state.devices[0].valid = false;
  const auto out = ctrl.Compute(state);
  EXPECT_EQ(out.devices[0].num_channels, 0) << "commanded an arm whose state it could not read";
  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_FALSE(d.control_valid);
  EXPECT_EQ(d.state, static_cast<std::uint8_t>(ComplianceState::kDegraded));
}

// The E-STOP hold answers "the device is unreadable" the same way
// ComputeNoJointState does — zero-length silence, not a value (#236 E-8). It
// used to emit nc0 channels of value-initialised zeros, which is a REAL 0 N·m
// write: on a torque-mode arm that is a drop, not a stop. The comment that
// justified it ("a torque backend with no drive signal is a free joint")
// described zero-LENGTH, which every backend early-returns on.
TEST(CascadedCompliance, EstopHoldWithUnreadableDeviceEmitsNoCommand) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  // Positive control: the hold on a READABLE device still commands, so the
  // zero-length below is the gate and not a dead E-STOP path.
  ctrl.TriggerEstop();
  const auto held = ctrl.Compute(state);
  ASSERT_EQ(held.devices[0].num_channels, 6);

  state.devices[0].valid = false;
  const auto out = ctrl.Compute(state);
  EXPECT_EQ(out.devices[0].num_channels, 0)
      << "a real 0 N·m command was emitted for an arm whose state could not be read";
  EXPECT_FALSE(ctrl.GetDiagnosticsForTesting().control_valid);
}

TEST(CascadedCompliance, TooFewDeviceChannelsIsTreatedAsNoJointState) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  state.devices[0].num_channels = 4;  // model has 6
  const auto out = ctrl.Compute(state);
  EXPECT_EQ(out.devices[0].num_channels, 0);
  EXPECT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kDegraded));
}

// A target pushed while the joint state was unreadable must not outlive the
// outage. The E-STOP path drains for exactly this reason — ClearEstop() does not
// bump the activation generation, so a queued command still passes
// IsCurrentGeneration — and the device-invalid path inherits the same re-seed
// contract without (until now) the same drain. The producer is off-RT and keeps
// publishing through a backend dropout, so the depth-4 queue ends up holding the
// OLDEST commands of the outage (SpscQueue drops the newest when full) and the
// first recovered tick would jump to one of them instead of the measured pose,
// braked only by max_torque_rate.
TEST(CascadedCompliance, TargetsQueuedWhileTheDeviceIsInvalidDoNotSurviveRecovery) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  const auto seeded = ctrl.Compute(state);  // X_d := the measured pose

  // Offset small enough to stay inside pose_error_limit, so the recovery tick is
  // a normal emitting tick and not a SAFE_STOP hold.
  std::array<double, 6> away{};
  for (std::size_t i = 0; i < 6; ++i)
    away[i] = seeded.actual_task_positions[i];
  away[0] += 0.05;

  state.devices[0].valid = false;
  for (int k = 0; k < 8; ++k)  // more than kPendingTargetDepth
    ctrl.SetDeviceTarget(0, std::span<const double>(away.data(), away.size()));
  (void)ctrl.Compute(state);

  state.devices[0].valid = true;
  const auto recovered = ctrl.Compute(state);
  ASSERT_GT(recovered.devices[0].num_channels, 0) << "the recovery tick emitted nothing";
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(recovered.task_goal_positions[i], seeded.task_goal_positions[i], 1e-9)
        << "axis " << i << ": a target queued during the outage overwrote the re-seed";
  }

  // Positive control: the very same target, pushed while the device is healthy,
  // DOES move the frame — so the equality above is a drain and not an inert
  // SetDeviceTarget or an unobservable offset.
  ctrl.SetDeviceTarget(0, std::span<const double>(away.data(), away.size()));
  const auto commanded = ctrl.Compute(state);
  EXPECT_NEAR(commanded.task_goal_positions[0], away[0], 1e-9);
  EXPECT_GT(std::abs(commanded.task_goal_positions[0] - seeded.task_goal_positions[0]), 1e-3);
}

// ── F7 (PR #256): a mis-shaped gain node must not be silently dropped ───────

namespace {
// The message of whatever LoadConfig throws, or "" if it accepts the file.
// Matching on the message matters here: yaml-cpp's own conversion failure also
// derives from std::runtime_error, so a bare EXPECT_THROW would pass even when
// the value never reached the check under test.
std::string ConfigureError(const std::string& yaml) {
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  try {
    ctrl.LoadConfig(YAML::Load(yaml));
  } catch (const std::exception& e) {
    return e.what();
  }
  return {};
}

bool Mentions(const std::string& msg, const std::string& needle) {
  return msg.find(needle) != std::string::npos;
}
}  // namespace

// One level up from the leaf shape check: a section that is PRESENT but not a
// map was skipped whole, so every key under it silently kept its default while
// the file said otherwise — the same class of failure F7 fixed for the leaves,
// and the one that turns `outer.stiffness: [0,...]` (hand-guiding) back into the
// 200 N/m default that pushes an operator's hand away.
TEST(CascadedCompliance, ConfigureRejectsASectionThatIsPresentButNotAMap) {
  EXPECT_TRUE(
      Mentions(ConfigureError(TransparentWrenchYaml("outer: 3.0\n")), "outer must be a map"))
      << "a scalar `outer:` was accepted";
  EXPECT_TRUE(
      Mentions(ConfigureError(TransparentWrenchYaml("inner: [1.0, 2.0]\n")), "inner must be a map"))
      << "a sequence `inner:` was accepted";
  EXPECT_TRUE(Mentions(ConfigureError("external_wrench: 3.0\n"), "external_wrench must be a map"))
      << "a scalar `external_wrench:` was accepted";

  // ...but an ABSENT section stays legal — the defaults ARE the documented
  // behaviour, and rejecting that would break every minimal config file.
  EXPECT_EQ(ConfigureError(TransparentWrenchYaml()), "");
}

// `v < 0.0` is FALSE for NaN, so the non-negative branch of load6 passed NaN
// straight through, and every clamped scalar hid it differently: std::max(0.0,
// NaN) returns 0.0 — silently a third value. A NaN stiffness poisons the
// admittance state on the first tick and latches SAFE_STOP with `nan_inf`, with
// nothing pointing at the config line.
TEST(CascadedCompliance, ConfigureRejectsNonFiniteGains) {
  const auto nan_stiffness =
      ConfigureError(TransparentWrenchYaml("outer:\n  stiffness: [.nan, 0, 0, 0, 0, 0]\n"));
  EXPECT_TRUE(Mentions(nan_stiffness, "outer.stiffness")) << nan_stiffness;
  EXPECT_TRUE(Mentions(nan_stiffness, "finite")) << nan_stiffness;

  // `.inf` rather than an overflowing literal such as 1e400: yaml-cpp rejects
  // the latter itself ("bad conversion"), which would make the assertion below
  // pass without our check ever running.
  const auto inf_damping =
      ConfigureError(TransparentWrenchYaml("outer:\n  damping: [.inf, 1, 1, 1, 1, 1]\n"));
  EXPECT_TRUE(Mentions(inf_damping, "outer.damping")) << inf_damping;
  EXPECT_TRUE(Mentions(inf_damping, "finite")) << inf_damping;

  const auto nan_ramp = ConfigureError(TransparentWrenchYaml("activation_ramp_time: .nan\n"));
  EXPECT_TRUE(Mentions(nan_ramp, "activation_ramp_time")) << nan_ramp;

  // A clamped scalar must not launder it into the clamp bound either.
  const auto nan_clamped = ConfigureError(TransparentWrenchYaml("estop_damping: .nan\n"));
  EXPECT_TRUE(Mentions(nan_clamped, "estop_damping")) << nan_clamped;
}

TEST(CascadedCompliance, ConfigureRejectsMisshapedGainSequences) {
  auto make = [] {
    auto* c = new CascadedComplianceController(Urdf6(), InertSafetyGains());
    c->SetControlRate(kRateHz);
    return std::unique_ptr<CascadedComplianceController>(c);
  };
  // Scalar where a 3-sequence is required — the obvious way to write "soft on
  // every axis", and the shape that used to be dropped in favour of the default.
  EXPECT_THROW(make()->LoadConfig(YAML::Load(TransparentWrenchYaml("inner:\n  kp_pos: 50.0\n"))),
               std::runtime_error);
  EXPECT_THROW(
      make()->LoadConfig(YAML::Load(TransparentWrenchYaml("inner:\n  kd_rot: [1.0, 2.0]\n"))),
      std::runtime_error);
  EXPECT_THROW(make()->LoadConfig(
                   YAML::Load(TransparentWrenchYaml("outer:\n  stiffness: [1.0, 2.0, 3.0]\n"))),
               std::runtime_error);
  EXPECT_THROW(make()->LoadConfig(YAML::Load(
                   TransparentWrenchYaml("outer:\n  min_desired_inertia: [1.0, 2.0, 3.0]\n"))),
               std::runtime_error);
  // A correctly shaped one commits, so the throws above are about the SHAPE and
  // not about the key being rejected outright.
  auto ok = make();
  ok->LoadConfig(
      YAML::Load(TransparentWrenchYaml("inner:\n  kp_pos: [11.0, 22.0, 33.0]\n"
                                       "outer:\n  stiffness: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]\n")));
  EXPECT_DOUBLE_EQ(ok->get_gains().impedance.kp_pos[1], 22.0);
  EXPECT_DOUBLE_EQ(ok->get_gains().admittance.stiffness[5], 6.0);
}

// ── F8 (PR #256): no threshold may latch SAFE_STOP by being 0 or negative ───

TEST(CascadedCompliance, ConfigureRejectsThresholdsThatWouldLatchOnTheFirstTick) {
  auto make = [] {
    auto* c = new CascadedComplianceController(Urdf6(), InertSafetyGains());
    c->SetControlRate(kRateHz);
    return std::unique_ptr<CascadedComplianceController>(c);
  };
  // ‖e‖ > 0 is true on essentially every tick ⇒ SAFE_STOP on the first one, with
  // no cause field pointing at the config.
  EXPECT_THROW(make()->LoadConfig(YAML::Load(TransparentWrenchYaml("pose_error_limit: 0.0\n"))),
               std::runtime_error);
  EXPECT_THROW(make()->LoadConfig(YAML::Load(TransparentWrenchYaml("pose_error_limit: -1.0\n"))),
               std::runtime_error);
  // A zero slew bound freezes the command at the rate history forever — the arm
  // stops responding while every fault stays clear.
  EXPECT_THROW(make()->LoadConfig(YAML::Load(TransparentWrenchYaml("max_torque_rate: 0.0\n"))),
               std::runtime_error);
  // Λ_d is inverted every tick (NUM-2).
  EXPECT_THROW(make()->LoadConfig(YAML::Load(TransparentWrenchYaml(
                   "outer:\n  desired_inertia: [1.0, 0.0, 1.0, 0.1, 0.1, 0.1]\n"))),
               std::runtime_error);
  EXPECT_THROW(make()->LoadConfig(
                   YAML::Load(TransparentWrenchYaml("inner:\n  kp_pos: [10.0, -1.0, 10.0]\n"))),
               std::runtime_error);
}

// The positive half of F8: the DEFAULT configuration must not fault on tick one.
// Every threshold above is rejected at configure precisely so this holds.
TEST(CascadedCompliance, DefaultConfigurationDoesNotFaultOnTheFirstTick) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController ctrl(Urdf6());  // spec defaults, ramp included
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  for (int k = 0; k < 10; ++k) {
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop)) << "tick " << k;
    EXPECT_TRUE(d.control_valid) << "tick " << k;
  }
}

// ── Configure contracts ─────────────────────────────────────────────────────

TEST(CascadedCompliance, RequiresAnExternalWrenchSource) {
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load("external_wrench:\n  enabled: false\n")),
               std::runtime_error);
  // A rejected configure leaves the previous wiring intact, not half-applied.
  EXPECT_TRUE(ctrl.external_wrench_enabled());
}

TEST(CascadedCompliance, RejectsANonTorqueCommandType) {
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml("command_type: position\n"))),
               std::runtime_error);
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kTorque);
}

TEST(CascadedCompliance, ConfigureRejectsUnknownSensorFrame) {
  CascadedComplianceController ctrl(Urdf6(), InertSafetyGains());
  ctrl.SetControlRate(kRateHz);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml("  sensor_frame: no_such_link\n"))),
               std::runtime_error);
}

// ── RT gates ────────────────────────────────────────────────────────────────

// T7.1 — the FIRST Compute() must already be allocation-free (a warm-up tick
// outside the guard would hide exactly the sizing bug this catches). Both loops
// must be LIVE inside the guard, which takes some care: the seeding tick calls
// WrenchPipeline::ResetForActivation(), and that DISOWNS whatever sample is in
// the slot (§10.6 — an expired wrench goes to zero, never held). A wrench queued
// before the guard is therefore gone by the time Update() runs, every tick takes
// the `!sample.valid` early return, and WrenchConditioner::Apply /
// transformWrench / ContactHysteresis::Update never execute under the counter —
// the gate passes while covering nothing. So the sample is published AFTER the
// seeding tick, inside the guard, and the diagnostics below assert that the
// pipeline really did run rather than trusting the layout.
TEST(CascadedCompliance, ComputeIsAllocationFree) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.nullspace_kp = 20.0;  // exercise Λ_S / Nᵀ inside the guard as well
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.05));
  // Force above the default 5 N contact threshold so the hysteresis latch — the
  // last stage of the pipeline — takes its state-changing branch too.
  const Wrench6 push{40.0, -5.0, 3.0, 0.2, -0.1, 0.05};

  // RAII rather than a bare arm/disarm pair: an ASSERT_* added inside the
  // measured region returns from the test, and a bare disarm line would then
  // never run — leaving counting on for every later test in the binary, where
  // nothing reads it.
  {
    rtc::testing::ScopedAllocGate gate;
    Send(ctrl, push);                         // SetExternalWrench is a SeqLock store
    const auto out = ctrl.Compute(state);     // seeding tick: disowns the sample
    Send(ctrl, push);                         // ...so republish for the seeded tick
    const auto second = ctrl.Compute(state);  // full pipeline: condition → Ad^{-T} → contact
    EXPECT_EQ(gate.count(), 0u) << "Compute() allocated on the RT path";
    EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
    EXPECT_EQ(second.command_type, rtc::CommandType::kTorque);
  }

  // The gate is only worth its assertion if the branch it guards actually ran.
  const auto d = ctrl.GetDiagnosticsForTesting();
  ASSERT_TRUE(d.wrench_valid) << "no sample reached the pipeline inside the guard";
  ASSERT_TRUE(d.bias_calibrated) << "the pipeline stopped at BIAS_CALIBRATING and emitted zero";
  EXPECT_GT(ToVec6(d.wrench_lwa).norm(), 1e-9)
      << "the conditioning / frame-transform stages never executed under the counter";
  EXPECT_TRUE(d.in_contact) << "ContactHysteresis::Update never took its latching branch";
}

TEST(CascadedCompliance, OutputAlwaysFinite) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.nullspace_kp = 20.0;
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.1));
  for (int k = 0; k < 200; ++k) {
    Send(ctrl, Wrench6{40.0, -30.0, 25.0, 2.0, -1.5, 1.0});
    const auto out = ctrl.Compute(state);
    for (int i = 0; i < out.devices[0].num_channels; ++i)
      ASSERT_TRUE(std::isfinite(out.devices[0].commands[static_cast<std::size_t>(i)]))
          << "tick " << k << " joint " << i;
  }
}

}  // namespace
