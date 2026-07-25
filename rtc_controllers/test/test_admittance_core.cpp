// ── Slice-3 numerical core (#236): admittance integrator + differential IK ───
// Everything here is model-free — pure Eigen — so the §7.2 integration scheme,
// the §7.5 workspace guards and the §7.3 DLS/nullspace algebra are pinned
// without a URDF. What needs a robot (frames, Jacobians, the sign convention
// against the impedance law, RT allocation) lives in
// test_task_admittance_controller.cpp.
//
// Two of these tests are MUTATION CHECKS: they carry an independent
// implementation of the wrong thing (explicit Euler; tangent-space rotation
// accumulation) and assert it produces a materially different answer. Without
// them "energy did not grow" and "the rotation stayed orthogonal" are both
// satisfied by implementations that are subtly wrong, and the spec asks for
// exactly this discrimination (§11.5 T5.2: "explicit Euler로 구현하면 이 테스트가
// 실패해야 한다").
#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

using rtc::compliance::AdmittanceIntegrator;
using rtc::compliance::AdmittanceParams;
using rtc::compliance::DifferentialIk;
using Vec6 = Eigen::Matrix<double, 6, 1>;

constexpr double kDt = 0.002;

// Λ_d = 2 kg / 0.05 kg·m², K_p = 200 N/m ⇒ ω = 10 rad/s at ω·dt = 0.02, well
// inside the semi-implicit stability bound and slow enough that 10 s of
// simulation covers ~16 oscillation periods.
AdmittanceParams BaseParams() {
  AdmittanceParams p;
  p.inertia = {{2.0, 2.0, 2.0, 0.05, 0.05, 0.05}};
  p.damping = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  p.stiffness = {{200.0, 200.0, 200.0, 5.0, 5.0, 5.0}};
  // Guards OFF (0 disables): they are dissipative, and an energy test must not
  // be able to pass because a limiter happened to bleed the energy away.
  p.max_displacement_lin = 0.0;
  p.max_displacement_ang = 0.0;
  p.max_velocity_lin = 0.0;
  p.max_velocity_ang = 0.0;
  return p;
}

Vec6 Force(double fx, double fy = 0.0, double fz = 0.0) {
  Vec6 f = Vec6::Zero();
  f(0) = fx;
  f(1) = fy;
  f(2) = fz;
  return f;
}

Vec6 Torque(double tx, double ty = 0.0, double tz = 0.0) {
  Vec6 f = Vec6::Zero();
  f(3) = tx;
  f(4) = ty;
  f(5) = tz;
  return f;
}

// ── §7.2 / §11.5 T5.2 / §11.8 T8.1 — semi-implicit Euler is not a pump ───────

TEST(AdmittanceIntegrator, UndampedEnergyStaysBoundedAfterTheForceIsRemoved) {
  const AdmittanceParams p = BaseParams();
  AdmittanceIntegrator integ;
  integ.Reset();

  // Excite: 0.2 s of constant push leaves both a displacement and a velocity —
  // there is no state setter, and driving it is what a real activation does.
  for (int k = 0; k < 100; ++k)
    ASSERT_TRUE(integ.Step(p, Force(20.0), kDt).finite);

  const double e0 = integ.Energy(p);
  ASSERT_GT(e0, 1e-6) << "the excitation phase must leave real energy behind";

  // 10 s of free oscillation with ZERO damping: the only thing keeping E from
  // growing is the integration scheme itself.
  double e_max = e0;
  for (int k = 0; k < 5000; ++k) {
    ASSERT_TRUE(integ.Step(p, Vec6::Zero(), kDt).finite);
    e_max = std::max(e_max, integ.Energy(p));
  }
  // Symplectic Euler conserves a SHADOW energy; the true E oscillates by O(ω·dt)
  // = 2% around it. It must not drift upward beyond that.
  EXPECT_LE(e_max, e0 * 1.05) << "energy grew — the integrator is pumping";
}

TEST(AdmittanceIntegrator, ExplicitEulerReferenceGainsEnergy) {
  // MUTATION CHECK for the test above: the same ODE, same gains, same dt, with
  // the ONE change §7.2 forbids (position updated from the OLD velocity). If
  // this reference did not blow up, the bound above would be vacuous.
  const AdmittanceParams p = BaseParams();
  const double lambda = p.inertia[0];
  const double kp = p.stiffness[0];

  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  auto energy = [&] { return 0.5 * lambda * v.squaredNorm() + 0.5 * kp * x.squaredNorm(); };

  for (int k = 0; k < 100; ++k) {
    const Eigen::Vector3d a = (Eigen::Vector3d(20.0, 0.0, 0.0) - kp * x) / lambda;
    x += v * kDt;  // ← position FIRST, from the stale velocity
    v += a * kDt;
  }
  const double e0 = energy();
  for (int k = 0; k < 5000; ++k) {
    const Eigen::Vector3d a = (-kp * x) / lambda;
    x += v * kDt;
    v += a * kDt;
  }
  EXPECT_GT(energy(), e0 * 2.0)
      << "explicit Euler failed to diverge — the energy bound above proves nothing";
}

TEST(AdmittanceIntegrator, DampedEnergyDecaysToRest) {
  AdmittanceParams p = BaseParams();
  p.damping = {{40.0, 40.0, 40.0, 1.0, 1.0, 1.0}};  // ζ = 1 at Λ=2, K_p=200
  AdmittanceIntegrator integ;
  integ.Reset();
  for (int k = 0; k < 100; ++k)
    integ.Step(p, Force(20.0), kDt);
  const double e0 = integ.Energy(p);
  for (int k = 0; k < 5000; ++k)
    integ.Step(p, Vec6::Zero(), kDt);
  EXPECT_LT(integ.Energy(p), e0 * 0.01);
  EXPECT_LT(integ.velocity().norm(), 1e-6);
  EXPECT_LT(integ.deviation().norm(), 1e-6) << "K_p > 0 must pull the frame back to X_d";
}

// ── §7.2 rotation retract / §11.5 T5.3 ──────────────────────────────────────

TEST(AdmittanceIntegrator, RotationRetractsOnSO3AndDivergesFromTangentAccumulation) {
  AdmittanceParams p = BaseParams();
  p.stiffness = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};  // free rotation: let it wind up
  AdmittanceIntegrator integ;
  integ.Reset();

  // Two NON-COAXIAL rotation phases. Coaxial rotations commute and their
  // rotation vectors add, so a coaxial test cannot tell a retract from a
  // tangent-space sum — the whole point of §7.2's MUST.
  Eigen::Vector3d naive = Eigen::Vector3d::Zero();  // Σ ω·dt — the forbidden form
  Eigen::Matrix3d reference = Eigen::Matrix3d::Identity();
  std::vector<Vec6> schedule;
  schedule.reserve(1000);
  for (int k = 0; k < 500; ++k)
    schedule.push_back(Torque(0.05, 0.0, 0.0));
  for (int k = 0; k < 500; ++k)
    schedule.push_back(Torque(0.0, 0.05, 0.0));

  for (const Vec6& tau : schedule) {
    ASSERT_TRUE(integ.Step(p, tau, kDt).finite);
    const Eigen::Vector3d omega = integ.velocity().tail<3>();
    naive += omega * kDt;
    // Independent recomposition from the SAME ω sequence, left-multiplied
    // because ω is a WORLD angular velocity (LWA) — this pins the composition
    // ORDER and the side of the multiplication, not just orthogonality.
    reference = rtc::math::se3::exp3(Eigen::Vector3d(omega * kDt)) * reference;
  }

  const Eigen::Matrix3d& R = integ.rotation();
  EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 1e-10)
      << "R̃ left SO(3) after 1000 steps";
  EXPECT_LT((R - reference).norm(), 1e-12) << "R̃ is not the left-composed exp3 product";

  const Eigen::Vector3d phi = integ.deviation().tail<3>();
  EXPECT_GT(phi.norm(), 0.3) << "the schedule must wind up far enough to discriminate";
  EXPECT_GT((phi - naive).norm(), 0.02)
      << "log3(R̃) coincides with the tangent-space sum — the retract is not doing anything";
}

// ── §7.5 workspace / velocity guards ────────────────────────────────────────

TEST(AdmittanceIntegrator, DisplacementBoundHoldsUnderASustainedPush) {
  AdmittanceParams p = BaseParams();
  p.stiffness = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};  // K_p = 0: pure hand-guiding drift
  p.damping = {{40.0, 40.0, 40.0, 1.0, 1.0, 1.0}};
  p.max_displacement_lin = 0.15;
  p.max_velocity_lin = 0.25;
  AdmittanceIntegrator integ;
  integ.Reset();

  bool ever_limited = false;
  for (int k = 0; k < 5000; ++k) {
    const auto st = integ.Step(p, Force(60.0), kDt);
    ever_limited = ever_limited || st.displacement_limited;
    ASSERT_LE(integ.translation().norm(), p.max_displacement_lin + 1e-9)
        << "‖x̃‖ escaped the §7.5 bound at step " << k;
  }
  EXPECT_TRUE(ever_limited) << "the guard never engaged — the test proves nothing";
  // ...and it holds the frame AT the wall rather than well short of it: a
  // guard that stopped early would satisfy the bound while silently shrinking
  // the reachable compliant workspace.
  EXPECT_NEAR(integ.translation().norm(), p.max_displacement_lin, 1e-3);
}

TEST(AdmittanceIntegrator, AtTheBoundTheFrameStillSlidesTangentially) {
  // Discriminates the §7.5 saturating spring + outward-velocity projection from
  // a plain ‖x̃‖ clamp: a clamp freezes the frame at the wall, while only the
  // OUTWARD component may be removed. Push +x to the wall, then push +y.
  AdmittanceParams p = BaseParams();
  p.stiffness = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  p.damping = {{40.0, 40.0, 40.0, 1.0, 1.0, 1.0}};
  p.max_displacement_lin = 0.15;
  p.max_velocity_lin = 0.25;
  AdmittanceIntegrator integ;
  integ.Reset();

  for (int k = 0; k < 4000; ++k)
    integ.Step(p, Force(60.0), kDt);
  ASSERT_NEAR(integ.translation().norm(), p.max_displacement_lin, 2e-3);
  const double y_before = integ.translation().y();

  for (int k = 0; k < 4000; ++k) {
    integ.Step(p, Force(0.0, 60.0), kDt);
    // Sliding is where a naive "zero the outward radial velocity" leaks: the
    // tangential step off the sphere adds (v·dt)²/2d every tick and walks the
    // frame past the bound (1e-5 m over this loop). The exact-landing solve is
    // what holds it here.
    ASSERT_LE(integ.translation().norm(), p.max_displacement_lin + 1e-9);
  }
  EXPECT_GT(integ.translation().y() - y_before, 0.05)
      << "the frame is stuck at the wall — a clamp, not a saturating spring";
}

TEST(AdmittanceIntegrator, VelocityBoundSurvivesALargeImpulse) {
  AdmittanceParams p = BaseParams();
  p.max_velocity_lin = 0.25;
  p.max_velocity_ang = 0.8;
  AdmittanceIntegrator integ;
  integ.Reset();
  for (int k = 0; k < 200; ++k) {
    const auto st = integ.Step(p, Force(5000.0) + Torque(0.0, 0.0, 500.0), kDt);
    ASSERT_TRUE(st.finite);
    ASSERT_LE(integ.velocity().head<3>().norm(), p.max_velocity_lin + 1e-9);
    ASSERT_LE(integ.velocity().tail<3>().norm(), p.max_velocity_ang + 1e-9);
  }
}

TEST(AdmittanceIntegrator, VelocityLimitPreservesDirection) {
  // A per-component clamp would turn a diagonal push into an axis-aligned one.
  AdmittanceParams p = BaseParams();
  p.max_velocity_lin = 0.1;
  AdmittanceIntegrator integ;
  integ.Reset();
  for (int k = 0; k < 200; ++k)
    integ.Step(p, Force(1000.0, 500.0), kDt);
  const Eigen::Vector3d v = integ.velocity().head<3>();
  ASSERT_NEAR(v.norm(), 0.1, 1e-9);
  EXPECT_NEAR(v.x() / v.y(), 2.0, 1e-6) << "the limiter rotated the motion";
}

// ── §7.4 contact-stability floor ────────────────────────────────────────────

TEST(AdmittanceIntegrator, DesiredInertiaBelowTheFloorBehavesAsTheFloor) {
  AdmittanceParams light = BaseParams();
  light.inertia = {{0.001, 0.001, 0.001, 1e-6, 1e-6, 1e-6}};  // far below the floor
  AdmittanceParams floored = BaseParams();
  floored.inertia = {{light.min_inertia_lin, light.min_inertia_lin, light.min_inertia_lin,
                      light.min_inertia_ang, light.min_inertia_ang, light.min_inertia_ang}};

  AdmittanceIntegrator a;
  AdmittanceIntegrator b;
  a.Reset();
  b.Reset();
  for (int k = 0; k < 500; ++k) {
    a.Step(light, Force(10.0) + Torque(0.2), kDt);
    b.Step(floored, Force(10.0) + Torque(0.2), kDt);
  }
  EXPECT_LT((a.deviation() - b.deviation()).norm(), 1e-12);
  EXPECT_LT((a.velocity() - b.velocity()).norm(), 1e-12);
  // And the floor is what makes it sane: an unfloored 1 g inertia would have
  // travelled orders of magnitude further under the same push.
  EXPECT_LT(a.translation().norm(), 1.0);
}

// ── Degenerate ticks must not poison the state ──────────────────────────────

TEST(AdmittanceIntegrator, NonFiniteWrenchAndNonPositiveDtLeaveTheStateUntouched) {
  const AdmittanceParams p = BaseParams();
  AdmittanceIntegrator integ;
  integ.Reset();
  for (int k = 0; k < 50; ++k)
    integ.Step(p, Force(10.0), kDt);
  const Vec6 before_x = integ.deviation();
  const Vec6 before_v = integ.velocity();

  Vec6 nan_force = Vec6::Zero();
  nan_force(1) = std::numeric_limits<double>::quiet_NaN();
  const auto st = integ.Step(p, nan_force, kDt);
  EXPECT_FALSE(st.finite);
  EXPECT_TRUE(integ.deviation().isApprox(before_x));
  EXPECT_TRUE(integ.velocity().isApprox(before_v));

  const auto st0 = integ.Step(p, Force(10.0), 0.0);
  EXPECT_TRUE(st0.finite);
  EXPECT_TRUE(integ.deviation().isApprox(before_x)) << "dt <= 0 must be a no-op, not a step";

  integ.Reset();
  EXPECT_LT(integ.deviation().norm(), 1e-15);
  EXPECT_LT(integ.velocity().norm(), 1e-15);
}

// ── §7.3 differential IK ────────────────────────────────────────────────────

// A deterministic, well-conditioned 6×7 Jacobian. Explicit entries rather than a
// PRNG so a failure is reproducible from the source alone.
Eigen::MatrixXd WellConditionedJ() {
  Eigen::MatrixXd J(6, 7);
  J << 1.0, 0.2, -0.3, 0.1, 0.0, 0.4, -0.1,  //
      0.0, 1.1, 0.2, -0.2, 0.3, 0.0, 0.2,    //
      0.3, 0.0, 0.9, 0.4, -0.1, 0.2, 0.0,    //
      -0.2, 0.4, 0.0, 1.2, 0.2, -0.3, 0.1,   //
      0.1, -0.1, 0.3, 0.0, 1.0, 0.2, 0.3,    //
      0.0, 0.2, -0.2, 0.3, 0.1, 1.1, -0.2;
  return J;
}

TEST(DifferentialIk, UndampedPseudoInverseIsARightInverseAndTheProjectorIsAProjector) {
  const Eigen::MatrixXd J = WellConditionedJ();
  DifferentialIk ik;
  ik.Resize(7, 6);
  // sigma0 below σ_min ⇒ λ² = 0 exactly (§6.5), so J⁺ is the true pseudoinverse.
  const auto r = ik.Compute(J, 1e-9, 0.05);
  ASSERT_TRUE(r.ok);
  EXPECT_DOUBLE_EQ(r.lambda_sq, 0.0);

  const Eigen::MatrixXd& Jp = ik.PseudoInverse();
  EXPECT_LT((J * Jp - Eigen::MatrixXd::Identity(6, 6)).norm(), 1e-10);

  const Eigen::MatrixXd& N = ik.NullspaceProjector();
  EXPECT_LT((N * N - N).norm(), 1e-10) << "N is not idempotent";
  EXPECT_LT((J * N).norm(), 1e-10) << "N leaks into the task space";
}

TEST(DifferentialIk, NullspaceMotionDoesNotDisturbTheTask) {
  const Eigen::MatrixXd J = WellConditionedJ();
  DifferentialIk ik;
  ik.Resize(7, 6);
  ASSERT_TRUE(ik.Compute(J, 1e-9, 0.05).ok);

  Vec6 twist;
  twist << 0.1, -0.2, 0.05, 0.3, 0.0, -0.1;
  Eigen::VectorXd qnull(7);
  qnull << 1.0, -2.0, 0.5, 0.25, -1.5, 3.0, 0.75;
  Eigen::VectorXd qdot(7);
  ik.Solve(twist, qnull, qdot);

  EXPECT_LT((J * qdot - twist).norm(), 1e-10) << "the posture task bled into the primary task";

  // ...and it is not simply being ignored.
  Eigen::VectorXd qdot_no_null(7);
  ik.Solve(twist, Eigen::VectorXd::Zero(7), qdot_no_null);
  EXPECT_GT((qdot - qdot_no_null).norm(), 0.1);
}

TEST(DifferentialIk, NonRedundantArmHasNoNullspace) {
  Eigen::MatrixXd J = WellConditionedJ().leftCols(6);
  DifferentialIk ik;
  ik.Resize(6, 6);
  ASSERT_TRUE(ik.Compute(J, 1e-9, 0.05).ok);
  EXPECT_LT(ik.NullspaceProjector().norm(), 1e-9);
}

TEST(DifferentialIk, SigmaMinMatchesTheSvdAndDrivesTheAdaptiveDamping) {
  const Eigen::MatrixXd J = WellConditionedJ();
  const double svd_sigma = Eigen::JacobiSVD<Eigen::MatrixXd>(J).singularValues().tail<1>()(0);

  DifferentialIk ik;
  ik.Resize(7, 6);
  const auto far = ik.Compute(J, 1e-9, 0.05);
  EXPECT_NEAR(far.sigma_min, svd_sigma, 1e-9);
  EXPECT_DOUBLE_EQ(far.lambda_sq, 0.0) << "damping engaged away from the singular shell";

  // Inside the shell (σ₀ well above σ_min) the §6.5 ramp must be active.
  const auto near = ik.Compute(J, svd_sigma * 4.0, 0.05);
  EXPECT_GT(near.lambda_sq, 0.0);
  EXPECT_LE(near.lambda_sq, 0.05 * 0.05);
}

TEST(DifferentialIk, RankDeficientJacobianStaysFiniteUnderDamping) {
  Eigen::MatrixXd J = WellConditionedJ();
  J.row(3) = J.row(0);  // exactly rank-deficient ⇒ σ_min = 0
  DifferentialIk ik;
  ik.Resize(7, 6);
  const auto r = ik.Compute(J, 0.02, 0.05);
  ASSERT_TRUE(r.ok) << "the DLS must keep the normal equations factorable";
  EXPECT_NEAR(r.sigma_min, 0.0, 1e-9);
  EXPECT_NEAR(r.lambda_sq, 0.05 * 0.05, 1e-12);  // full damping at σ_min = 0

  Vec6 twist;
  twist << 0.1, -0.2, 0.05, 0.3, 0.0, -0.1;
  Eigen::VectorXd qdot(7);
  ik.Solve(twist, Eigen::VectorXd::Zero(7), qdot);
  EXPECT_TRUE(qdot.allFinite());
  EXPECT_LT(qdot.norm(), 1e3);
}

TEST(DifferentialIk, NonFiniteJacobianReportsFailureAndLeavesTheStaleInverseAlone) {
  // The reachable route to an unfactorable A = J Jᵀ + λ²I is a NON-FINITE J (a
  // NaN joint state propagating through FK), not rank deficiency: J Jᵀ is always
  // positive SEMI-definite, and Eigen's LLT accepts a singular one. The contract
  // on failure is that `ok` goes false and J⁺/N keep their previous contents,
  // so a caller that ignores `ok` gets the last good inverse rather than NaN.
  const Eigen::MatrixXd good = WellConditionedJ();
  DifferentialIk ik;
  ik.Resize(7, 6);
  ASSERT_TRUE(ik.Compute(good, 1e-9, 0.05).ok);
  const Eigen::MatrixXd saved = ik.PseudoInverse();

  Eigen::MatrixXd bad = good;
  bad(2, 3) = std::numeric_limits<double>::quiet_NaN();
  const auto r = ik.Compute(bad, 0.02, 0.05);
  EXPECT_FALSE(r.ok);
  EXPECT_LT((ik.PseudoInverse() - saved).norm(), 1e-15);
  EXPECT_TRUE(ik.PseudoInverse().allFinite());
}

// ── §7.3 command-divergence fault is CRITICAL ───────────────────────────────

TEST(ComplianceFaultsCommandDivergence, IsCriticalAndLatchesSafeStop) {
  rtc::compliance::ComplianceFaults f;
  EXPECT_FALSE(f.AnyCritical());
  f.command_divergence = true;
  EXPECT_TRUE(f.AnyCritical());
  EXPECT_FALSE(f.AnyDegrade()) << "a wound-away command is not a reduced-authority mode";

  rtc::compliance::ComplianceStateMachine sm;
  sm.Step(f, /*ramp_done=*/true, kDt, 0.5);
  EXPECT_TRUE(sm.in_safe_stop());
  // Latched: clearing the cause is not enough (§10.6 자동 복구 금지).
  sm.Step(rtc::compliance::ComplianceFaults{}, true, kDt, 0.5);
  EXPECT_TRUE(sm.in_safe_stop());
  sm.ResetFault();
  EXPECT_FALSE(sm.in_safe_stop());
}

}  // namespace
