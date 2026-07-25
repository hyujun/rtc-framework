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
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <new>
#include <span>
#include <string>
#include <vector>

// ── Allocation counter (global new/delete interposition) ────────────────────
namespace {
thread_local bool g_alloc_active = false;
thread_local std::size_t g_alloc_count = 0;
}  // namespace

void* operator new(std::size_t n) {
  if (g_alloc_active)
    ++g_alloc_count;
  void* p = std::malloc(n != 0 ? n : 1);
  if (p == nullptr)
    throw std::bad_alloc();
  return p;
}

void* operator new[](std::size_t n) {
  return ::operator new(n);
}

void operator delete(void* p) noexcept {
  std::free(p);
}

void operator delete[](void* p) noexcept {
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

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

// ── F7 (PR #256): a mis-shaped gain node must not be silently dropped ───────

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
// are live: a wrench is queued, so the pipeline, the integrator and the task
// dynamics all run inside the guard.
TEST(CascadedCompliance, ComputeIsAllocationFree) {
  const std::vector<double> q(7, 0.25);
  auto gains = InertSafetyGains();
  gains.nullspace_kp = 20.0;  // exercise Λ_S / Nᵀ inside the guard as well
  CascadedComplianceController ctrl(Urdf7(), gains);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(7, q, std::vector<double>(7, 0.05));
  Send(ctrl, Wrench6{10.0, -5.0, 3.0, 0.2, -0.1, 0.05});

  g_alloc_count = 0;
  g_alloc_active = true;
  const auto out = ctrl.Compute(state);
  const auto second = ctrl.Compute(state);  // the seeded path, with the frame live
  g_alloc_active = false;
  EXPECT_EQ(g_alloc_count, 0u) << "Compute() allocated on the RT path";
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  EXPECT_EQ(second.command_type, rtc::CommandType::kTorque);
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
