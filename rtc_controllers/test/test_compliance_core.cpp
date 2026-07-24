// ── Unit tests for the compliance core helpers (slice 1, step 3) ────────────
// task_dynamics (Λ_S / Nᵀ / adaptive DLS), safety_limiter (§10.5 four stages),
// compliance_state_machine (reduced §10.6), torque_estop (E-8 hold). The task
// dynamics identities (T4.5, T5.1) are purely algebraic — they hold for any SPD
// M and selected Jacobian J_S — so no robot model is needed here; the control
// law tests that need a URDF live in the controller test (step 4).
//
// no_malloc_scope.hpp MUST precede every Eigen header (it installs the
// zero-allocation tripwire); the compliance headers pull Eigen, so it comes
// first.
#include "rtc_base/testing/no_malloc_scope.hpp"  // before any Eigen include
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/safety_limiter.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace {

using namespace rtc::compliance;

std::mt19937 MakeRng() {
  return std::mt19937(0xC0FFEEu);
}

// Random symmetric positive-definite nv×nv matrix (M = A Aᵀ + n·I).
Eigen::MatrixXd RandomSpd(int n, std::mt19937& rng) {
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd A(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      A(i, j) = g(rng);
  return A * A.transpose() + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);
}

Eigen::MatrixXd RandomMat(int rows, int cols, std::mt19937& rng) {
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd M(rows, cols);
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      M(i, j) = g(rng);
  return M;
}

// ── task_dynamics ──────────────────────────────────────────────────────────

// AdaptiveDampingSquared boundaries (§6.5): 0 above σ₀, λ_max² at σ=0, and the
// quadratic in between; σ₀≤0 is a hard 0 (NUM guard, no divide-by-zero).
TEST(TaskDynamics, AdaptiveDampingBoundaries) {
  constexpr double s0 = 0.02;
  constexpr double lmax = 0.05;
  EXPECT_EQ(AdaptiveDampingSquared(0.1, s0, lmax), 0.0);                 // σ ≥ σ₀ → 0
  EXPECT_EQ(AdaptiveDampingSquared(s0, s0, lmax), 0.0);                  // σ = σ₀ → 0
  EXPECT_DOUBLE_EQ(AdaptiveDampingSquared(0.0, s0, lmax), lmax * lmax);  // σ = 0
  EXPECT_DOUBLE_EQ(AdaptiveDampingSquared(0.5 * s0, s0, lmax), lmax * lmax * 0.75);
  EXPECT_EQ(AdaptiveDampingSquared(0.01, 0.0, lmax), 0.0);  // σ₀ ≤ 0 guard
}

// T4.5 — nullspace orthogonality: for a well-conditioned J_S (λ²=0) the
// dynamically-consistent projector satisfies J_S M⁻¹ Nᵀ τ = 0 exactly. This is
// the transpose-error detector of §6.4 (Nᵀ vs N).
TEST(TaskDynamics, NullspaceOrthogonality) {
  auto rng = MakeRng();
  for (int m : {3, 6}) {
    const int nv = 7;
    TaskDynamics dyn;
    dyn.Resize(nv, m);
    for (int trial = 0; trial < 50; ++trial) {
      const Eigen::MatrixXd M = RandomSpd(nv, rng);
      Eigen::LLT<Eigen::MatrixXd> llt_M(M);
      const Eigen::MatrixXd J_S = RandomMat(m, nv, rng);  // full row rank a.s.
      const auto r = dyn.Compute(J_S, llt_M, /*sigma0=*/0.02, /*lambda_max=*/0.05);
      ASSERT_TRUE(r.ok);
      ASSERT_GE(r.sigma_min, 0.02) << "test needs a well-conditioned J_S (λ²=0)";
      EXPECT_EQ(r.lambda_sq, 0.0);
      const Eigen::MatrixXd Minv = M.inverse();
      const Eigen::MatrixXd coupling = J_S * Minv * dyn.NullspaceProjectorTranspose();
      EXPECT_LT(coupling.norm(), 1e-9) << "m=" << m << " trial " << trial;
      // And through an explicit posture torque.
      const Eigen::VectorXd tau_posture = RandomMat(nv, 1, rng);
      Eigen::VectorXd tau_null(nv);
      dyn.ProjectNullspace(tau_posture, tau_null);
      EXPECT_LT((J_S * Minv * tau_null).norm(), 1e-9);
    }
  }
}

// T5.1 — rank-deficient J_S (singular pose): the adaptive DLS keeps every output
// finite (no NaN/Inf), and a positive λ² is applied.
TEST(TaskDynamics, RankDeficientStaysFinite) {
  auto rng = MakeRng();
  const int nv = 7;
  const int m = 6;
  TaskDynamics dyn;
  dyn.Resize(nv, m);
  const Eigen::MatrixXd M = RandomSpd(nv, rng);
  Eigen::LLT<Eigen::MatrixXd> llt_M(M);
  Eigen::MatrixXd J_S = RandomMat(m, nv, rng);
  J_S.row(2) = J_S.row(1);          // linearly dependent rows → rank deficient
  J_S.row(4) = 1e-14 * J_S.row(0);  // a near-zero direction
  const auto r = dyn.Compute(J_S, llt_M, 0.02, 0.05);
  ASSERT_TRUE(r.ok);
  EXPECT_LT(r.sigma_min, 0.02);
  EXPECT_GT(r.lambda_sq, 0.0);
  EXPECT_TRUE(dyn.NullspaceProjectorTranspose().allFinite());
  EXPECT_TRUE(dyn.LambdaS().allFinite());
  Eigen::VectorXd tau_null(nv);
  dyn.ProjectNullspace(RandomMat(nv, 1, rng), tau_null);
  EXPECT_TRUE(tau_null.allFinite());
}

// Zero-allocation gate: TaskDynamics::Compute must not touch the heap after
// Resize() (spec §11.7 T7.1). Runs in the same TU as the Eigen calls so the
// guard observes them.
TEST(TaskDynamics, ComputeIsAllocationFree) {
  auto rng = MakeRng();
  const int nv = 7;
  const int m = 6;
  TaskDynamics dyn;
  dyn.Resize(nv, m);
  const Eigen::MatrixXd M = RandomSpd(nv, rng);
  Eigen::LLT<Eigen::MatrixXd> llt_M(M);
  const Eigen::MatrixXd J_S = RandomMat(m, nv, rng);
  Eigen::VectorXd tau_posture = RandomMat(nv, 1, rng);
  Eigen::VectorXd tau_null(nv);
  // Warm up once outside the guard is NOT allowed — spec requires the FIRST
  // compute to be clean — so measure the very first call.
  {
    rtc::testing::ScopedNoMalloc guard;
    const auto r = dyn.Compute(J_S, llt_M, 0.02, 0.05);
    dyn.ProjectNullspace(tau_posture, tau_null);
    EXPECT_EQ(guard.violations(), 0u) << "TaskDynamics::Compute allocated on the RT path";
    (void)r;
  }
}

// ── safety_limiter (§10.5 four stages, independent) ─────────────────────────

TEST(SafetyLimiter, JointLimitRepulsiveLowerBound) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd q(1), qdot(1), qmin(1), qmax(1);
  q << -0.95;
  qdot << 0.0;
  qmin << -1.0;
  qmax << 1.0;
  // margin 0.1 → soft lower bound = -0.9; q = -0.95 < -0.9 → push +.
  AddJointLimitRepulsive(tau, q, qdot, qmin, qmax, 0.1, 10.0, 1.0);
  EXPECT_DOUBLE_EQ(tau(0), -10.0 * (-0.95 - (-0.9)));  // = +0.5, pushes away from lower limit
  EXPECT_GT(tau(0), 0.0);
}

TEST(SafetyLimiter, JointLimitRepulsiveInsideIsNoOp) {
  Eigen::VectorXd tau(1);
  tau << 3.0;
  Eigen::VectorXd q(1), qdot(1), qmin(1), qmax(1);
  q << 0.0;
  qdot << 5.0;
  qmin << -1.0;
  qmax << 1.0;
  AddJointLimitRepulsive(tau, q, qdot, qmin, qmax, 0.1, 10.0, 1.0);
  EXPECT_DOUBLE_EQ(tau(0), 3.0);  // untouched away from the margin band
}

TEST(SafetyLimiter, JointLimitRepulsiveDampingIncluded) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd q(1), qdot(1), qmin(1), qmax(1);
  q << 0.95;
  qdot << 2.0;  // moving further into the upper margin
  qmin << -1.0;
  qmax << 1.0;
  AddJointLimitRepulsive(tau, q, qdot, qmin, qmax, 0.1, 10.0, 3.0);
  // upper soft bound 0.9: spring = -10*(0.95-0.9) = -0.5; damping = -3*2 = -6.
  EXPECT_DOUBLE_EQ(tau(0), -0.5 - 6.0);
}

TEST(SafetyLimiter, SaturateAbsolute) {
  Eigen::VectorXd tau(3), tau_max(3);
  tau << 5.0, -8.0, 1.0;
  tau_max << 4.0, 4.0, 4.0;
  EXPECT_TRUE(SaturateAbsolute(tau, tau_max));
  EXPECT_DOUBLE_EQ(tau(0), 4.0);
  EXPECT_DOUBLE_EQ(tau(1), -4.0);
  EXPECT_DOUBLE_EQ(tau(2), 1.0);
  Eigen::VectorXd small(1), lim(1);
  small << 1.0;
  lim << 4.0;
  EXPECT_FALSE(SaturateAbsolute(small, lim));
}

// Rate limit uses the ACTUAL dt: halving dt halves the allowed step.
TEST(SafetyLimiter, RateLimitUsesDt) {
  Eigen::VectorXd tau(1), tau_prev(1);
  tau << 10.0;
  tau_prev << 0.0;
  EXPECT_TRUE(RateLimit(tau, tau_prev, /*max_rate=*/1000.0, /*dt=*/0.002));
  EXPECT_DOUBLE_EQ(tau(0), 2.0);       // 1000 * 0.002 = 2.0
  EXPECT_DOUBLE_EQ(tau_prev(0), 2.0);  // advanced

  Eigen::VectorXd tau2(1), tau_prev2(1);
  tau2 << 10.0;
  tau_prev2 << 0.0;
  EXPECT_TRUE(RateLimit(tau2, tau_prev2, 1000.0, 0.001));
  EXPECT_DOUBLE_EQ(tau2(0), 1.0);  // 1000 * 0.001 = 1.0 → dt-dependent, not 500 Hz
}

TEST(SafetyLimiter, RateLimitDegenerateDtDisabled) {
  Eigen::VectorXd tau(1), tau_prev(1);
  tau << 10.0;
  tau_prev << 0.0;
  EXPECT_FALSE(RateLimit(tau, tau_prev, 1000.0, 0.0));  // dt≤0 must not freeze
  EXPECT_DOUBLE_EQ(tau(0), 10.0);
  EXPECT_DOUBLE_EQ(tau_prev(0), 10.0);
}

TEST(SafetyLimiter, AllFiniteDetectsNaN) {
  Eigen::VectorXd ok(2);
  ok << 1.0, 2.0;
  EXPECT_TRUE(AllFinite(ok));
  Eigen::VectorXd bad(2);
  bad << 1.0, std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(AllFinite(bad));
}

// The orchestrator reports non-finite for BOTH ∞ and NaN. ∞ is the tricky case:
// absolute saturation would clamp it to the torque limit and mask it, so
// finiteness is read before saturation (regression guard for that masking).
TEST(SafetyLimiter, ApplyLayerReportsNonFinite) {
  const int n = 2;
  for (double bad :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    Eigen::VectorXd tau(n), tau_prev(n), q(n), qdot(n), qmin(n), qmax(n), tau_max(n);
    tau << 1.0, bad;
    tau_prev = Eigen::VectorXd::Zero(n);
    q = Eigen::VectorXd::Zero(n);
    qdot = Eigen::VectorXd::Zero(n);
    qmin = Eigen::VectorXd::Constant(n, -1.0);
    qmax = Eigen::VectorXd::Constant(n, 1.0);
    tau_max = Eigen::VectorXd::Constant(n, 100.0);
    const auto s =
        ApplySafetyLayer(tau, tau_prev, q, qdot, qmin, qmax, tau_max, 0.1, 10.0, 1.0, 1e6, 0.002);
    EXPECT_FALSE(s.finite) << (std::isnan(bad) ? "NaN" : "inf") << " not reported";
  }
}

// ── compliance_state_machine (reduced §10.6) ────────────────────────────────

TEST(StateMachine, HoldingToRunningOnRamp) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  EXPECT_EQ(sm.Step(f, /*ramp_done=*/false, 0.002, 0.5), ComplianceState::kHolding);
  EXPECT_EQ(sm.Step(f, /*ramp_done=*/true, 0.002, 0.5), ComplianceState::kRunning);
}

TEST(StateMachine, RunningToDegradedAndRecovery) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  sm.Step(f, true, 0.002, 0.5);  // → RUNNING
  f.saturation_persist = true;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kDegraded);
  // Cause persists → stays DEGRADED, timer keeps resetting.
  EXPECT_EQ(sm.Step(f, true, 0.4, 0.5), ComplianceState::kDegraded);
  // Cause cleared, accrue recovery time.
  f.saturation_persist = false;
  EXPECT_EQ(sm.Step(f, true, 0.3, 0.5), ComplianceState::kDegraded);  // 0.3 < 0.5
  EXPECT_EQ(sm.Step(f, true, 0.3, 0.5), ComplianceState::kRunning);   // 0.6 ≥ 0.5
}

// SAFE_STOP is latched: clearing the fault does NOT recover it — only
// ResetFault() does. And it is not the same signal as any global E-STOP bool.
TEST(StateMachine, SafeStopLatchedUntilResetFault) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  sm.Step(f, true, 0.002, 0.5);  // RUNNING
  f.nan_inf = true;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kSafeStop);
  // Fault cleared — must remain latched.
  f.nan_inf = false;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kSafeStop);
  EXPECT_TRUE(sm.in_safe_stop());
  // Only ResetFault leaves SAFE_STOP, returning to HOLDING (re-ramp).
  sm.ResetFault();
  EXPECT_EQ(sm.state(), ComplianceState::kHolding);
}

TEST(StateMachine, CriticalFromDegradedGoesSafeStop) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  sm.Step(f, true, 0.002, 0.5);
  f.sigma_below_threshold = true;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kDegraded);
  f.sigma_below_critical = true;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kSafeStop);
}

// ── torque_estop (E-8 gravity-comp damped hold) ─────────────────────────────

TEST(TorqueEstop, GravityMinusDampingClamped) {
  Eigen::VectorXd tau(3), g(3), qdot(3), tau_max(3);
  g << 5.0, -3.0, 20.0;
  qdot << 1.0, 2.0, 0.0;
  tau_max << 100.0, 100.0, 10.0;
  GravityCompDampedHold(tau, g, qdot, /*damping=*/2.0, tau_max);
  EXPECT_DOUBLE_EQ(tau(0), 5.0 - 2.0 * 1.0);   // 3.0
  EXPECT_DOUBLE_EQ(tau(1), -3.0 - 2.0 * 2.0);  // -7.0
  EXPECT_DOUBLE_EQ(tau(2), 10.0);              // 20 clamped to +10
}

}  // namespace
