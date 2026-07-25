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
#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/safety_limiter.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"
#include "rtc_controllers/compliance/wrench_conditioning.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <span>

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

// Finite-guard: the E-STOP hold is the last-resort fallback, so a non-finite
// input (NaN/∞ from a frozen encoder feeding ĝ(q) or q̇) must NOT reach the
// actuator — the per-joint clamp cannot catch it (NaN fails every comparison,
// ∞ would clamp to ±lim and mask the fault), so it is forced to 0 N·m. Finite
// joints in the same vector are unaffected.
// ── external_wrench: staleness by receipt, not by clock (D9, §10.6) ─────────

std::span<const double, kWrenchDim> Span6(const Wrench6& w) {
  return std::span<const double, kWrenchDim>(w.data(), kWrenchDim);
}

// Before anything arrives there is no sample to age: valid=false, fade=0. This
// is the activation transient, and reporting it as "stale" would raise a
// wrench_timeout fault on every single activation.
TEST(WrenchInput, NeverWrittenIsInvalidNotStale) {
  WrenchInput in;
  for (int k = 0; k < 100; ++k) {
    const auto r = in.Read(0.002, 0.05, 0.1);
    EXPECT_FALSE(r.valid);
    EXPECT_FALSE(r.stale);
    EXPECT_EQ(r.fade, 0.0);
    EXPECT_EQ(r.age, 0.0);
  }
}

// age = ticks_since_change × dt, and the fade is a LINEAR ramp to exactly zero
// over wrench_fadeout_time once wrench_timeout is passed (§10.6 forbids holding
// the last value; a plateau at any nonzero fraction would be exactly that bug).
TEST(WrenchInput, LinearFadeToZeroAfterTimeout) {
  constexpr double kDt = 0.002;
  constexpr double kTimeout = 0.01;  // 5 ticks
  constexpr double kFadeout = 0.02;  // 10 further ticks
  WrenchInput in;
  const Wrench6 w{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  in.Set(Span6(w));

  const auto fresh = in.Read(kDt, kTimeout, kFadeout);
  ASSERT_TRUE(fresh.valid);
  EXPECT_EQ(fresh.age, 0.0);
  EXPECT_FALSE(fresh.stale);
  EXPECT_DOUBLE_EQ(fresh.fade, 1.0);
  EXPECT_DOUBLE_EQ(fresh.value[2], 3.0);

  // Ticks 1..5: age ≤ timeout ⇒ still fully trusted.
  for (int k = 1; k <= 5; ++k) {
    const auto r = in.Read(kDt, kTimeout, kFadeout);
    EXPECT_NEAR(r.age, k * kDt, 1e-12) << "tick " << k;
    EXPECT_FALSE(r.stale) << "tick " << k;
    EXPECT_DOUBLE_EQ(r.fade, 1.0) << "tick " << k;
  }
  // Ticks 6..15: stale, fading linearly.
  for (int k = 6; k <= 15; ++k) {
    const auto r = in.Read(kDt, kTimeout, kFadeout);
    EXPECT_TRUE(r.stale) << "tick " << k;
    EXPECT_NEAR(r.fade, 1.0 - (k * kDt - kTimeout) / kFadeout, 1e-12) << "tick " << k;
  }
  // Past the ramp: exactly zero, and it STAYS zero (no wrap, no revival).
  for (int k = 16; k <= 200; ++k) {
    const auto r = in.Read(kDt, kTimeout, kFadeout);
    EXPECT_EQ(r.fade, 0.0) << "tick " << k;
    EXPECT_TRUE(r.stale) << "tick " << k;
  }
}

// The freshness criterion is RECEIPT, not value change. A live sensor holding a
// constant contact force re-sends the identical vector every cycle; a
// value-change detector would declare that sensor dead and fade a real contact
// force to zero. This is the test that distinguishes the two designs.
TEST(WrenchInput, IdenticalResendCountsAsFresh) {
  constexpr double kDt = 0.002;
  WrenchInput in;
  const Wrench6 w{7.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 50; ++k) {
    in.Set(Span6(w));  // byte-identical every time
    const auto r = in.Read(kDt, /*timeout=*/0.004, /*fadeout=*/0.01);
    EXPECT_EQ(r.age, 0.0) << "tick " << k;
    EXPECT_FALSE(r.stale) << "tick " << k;
    EXPECT_DOUBLE_EQ(r.fade, 1.0) << "tick " << k;
  }
}

// ResetTiming (the activation re-seed) clears the accrued age: a controller that
// was inactive for a minute must not open with an instant timeout fault.
TEST(WrenchInput, ResetTimingClearsAccruedAge) {
  constexpr double kDt = 0.002;
  WrenchInput in;
  const Wrench6 w{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  in.Set(Span6(w));
  for (int k = 0; k < 100; ++k)
    (void)in.Read(kDt, 0.01, 0.02);
  ASSERT_TRUE(in.Read(kDt, 0.01, 0.02).stale);

  in.ResetTiming();
  const auto r = in.Read(kDt, 0.01, 0.02);
  EXPECT_EQ(r.age, 0.0);
  EXPECT_FALSE(r.stale);
  EXPECT_DOUBLE_EQ(r.fade, 1.0);
}

// fadeout ≤ 0 is legal (an immediate drop) but must still reach ZERO — the one
// thing §10.6 forbids is the last value surviving the timeout.
TEST(WrenchInput, ZeroFadeoutDropsImmediately) {
  WrenchInput in;
  const Wrench6 w{5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  in.Set(Span6(w));
  (void)in.Read(0.002, 0.001, 0.0);  // age 0
  const auto r = in.Read(0.002, 0.001, 0.0);
  EXPECT_TRUE(r.stale);
  EXPECT_EQ(r.fade, 0.0);
}

TEST(ContactHysteresis, EntersHighExitsLow) {
  ContactHysteresis c;
  EXPECT_FALSE(c.Update(4.0, /*enter=*/5.0, /*exit=*/3.0));
  EXPECT_FALSE(c.Update(5.0, 5.0, 3.0));  // strictly greater to enter
  EXPECT_TRUE(c.Update(5.1, 5.0, 3.0));
  // Between the thresholds the latch HOLDS — that is the whole point.
  EXPECT_TRUE(c.Update(4.0, 5.0, 3.0));
  EXPECT_TRUE(c.Update(3.0, 5.0, 3.0));
  EXPECT_FALSE(c.Update(2.9, 5.0, 3.0));
  c.Reset();
  EXPECT_FALSE(c.in_contact());
}

// ── wrench_conditioning: the four stages, independently (§3.2.1) ────────────

TEST(WrenchConditioning, BiasRemoval) {
  Wrench6 w{10.0, -2.0, 3.0, 0.5, -0.5, 1.0};
  const Wrench6 bias{1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
  RemoveBias(w, bias);
  EXPECT_DOUBLE_EQ(w[0], 9.0);
  EXPECT_DOUBLE_EQ(w[1], -3.0);
  EXPECT_DOUBLE_EQ(w[5], 0.9);
}

// The payload term is the weight rotated into the SENSOR frame plus the lever
// arm moment r_c × f. Both halves are checked: a rotated sensor must see the
// weight on a different axis, and an offset CoM must produce a moment.
TEST(WrenchConditioning, StaticGravityRotatesAndLevers) {
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  const double m = 2.0;

  // Sensor aligned with world, CoM on +x ⇒ f = (0,0,−mg), τ = r×f = (0, +mg·rx, 0).
  const Wrench6 a =
      StaticGravityWrench(Eigen::Matrix3d::Identity(), g, m, Eigen::Vector3d(0.1, 0, 0));
  EXPECT_NEAR(a[2], -m * 9.81, 1e-12);
  EXPECT_NEAR(a[0], 0.0, 1e-12);
  EXPECT_NEAR(a[4], 0.1 * m * 9.81, 1e-12);  // (r × f)_y = rx·(−fz)·(−1) = +rx·m·g
  EXPECT_NEAR(a[3], 0.0, 1e-12);
  EXPECT_NEAR(a[5], 0.0, 1e-12);

  // Sensor rotated 90° about +y: world −Z maps onto the sensor's +x axis.
  const Eigen::Matrix3d R(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  const Wrench6 b = StaticGravityWrench(R, g, m, Eigen::Vector3d::Zero());
  EXPECT_NEAR(b[0], m * 9.81, 1e-9);
  EXPECT_NEAR(b[2], 0.0, 1e-9);

  // Zero mass ⇒ identically zero, no rounding dust.
  const Wrench6 z = StaticGravityWrench(R, g, 0.0, Eigen::Vector3d(1, 2, 3));
  for (double v : z)
    EXPECT_EQ(v, 0.0);
}

// SOFT deadband: the output SHRINKS by d rather than snapping to zero at the
// edge. A hard band would step by d here, which is the torque discontinuity
// §10.6 spends a MUST on avoiding.
TEST(WrenchConditioning, DeadbandIsContinuousShrink) {
  Wrench6 w{0.5, -0.5, 2.0, 0.0, 0.05, -3.0};
  const Wrench6 d{1.0, 1.0, 1.0, 0.1, 0.1, 0.0};
  ApplyDeadband(w, d);
  EXPECT_EQ(w[0], 0.0);         // inside the band
  EXPECT_EQ(w[1], 0.0);         // inside, negative side
  EXPECT_DOUBLE_EQ(w[2], 1.0);  // 2.0 − 1.0, sign kept
  EXPECT_EQ(w[4], 0.0);
  EXPECT_DOUBLE_EQ(w[5], -3.0);  // d == 0 ⇒ stage disabled for that axis

  // Continuity across the edge: 1+ε ↦ ε, so the jump is O(ε), not O(d).
  Wrench6 edge{1.0 + 1e-9, 0, 0, 0, 0, 0};
  ApplyDeadband(edge, d);
  EXPECT_NEAR(edge[0], 1e-9, 1e-15);
}

TEST(WrenchConditioning, SaturationClipsAndReports) {
  Wrench6 w{600.0, -600.0, 10.0, 0.0, 0.0, 0.0};
  const Wrench6 lim{500.0, 500.0, 500.0, 50.0, 50.0, 0.0};
  EXPECT_TRUE(SaturateWrench(w, lim));
  EXPECT_DOUBLE_EQ(w[0], 500.0);
  EXPECT_DOUBLE_EQ(w[1], -500.0);
  EXPECT_DOUBLE_EQ(w[2], 10.0);

  Wrench6 small{1.0, 0, 0, 0, 0, 0};
  EXPECT_FALSE(SaturateWrench(small, lim));
}

// The composed chain must apply the stages in the NORMATIVE order. Checked by
// construction rather than by inspection: raw = bias + gravity + signal, so a
// correct chain leaves exactly `signal` shrunk by the deadband. Swapping bias
// and gravity, or deadband and saturation, changes this number.
TEST(WrenchConditioning, ChainAppliesStagesInOrder) {
  WrenchConditioningConfig cfg;
  cfg.deadband = Wrench6{0.5, 0.5, 0.5, 0.05, 0.05, 0.05};
  cfg.max_abs = Wrench6{100.0, 100.0, 100.0, 10.0, 10.0, 10.0};
  cfg.filter_enabled = false;  // isolate the algebra from the filter dynamics
  cfg.bias_samples = 4;
  WrenchConditioner c;
  c.Configure(cfg, 500.0);

  const Wrench6 bias{2.0, -1.0, 0.5, 0.1, 0.0, -0.2};
  const Wrench6 grav{0.0, 0.0, -19.62, 0.0, 1.0, 0.0};
  auto raw_with = [&](const Wrench6& signal) {
    Wrench6 r{};
    for (std::size_t i = 0; i < kWrenchDim; ++i)
      r[i] = bias[i] + grav[i] + signal[i];
    return r;
  };

  // Calibrate at rest (signal = 0) — the bias must come out exactly.
  const Wrench6 rest = raw_with(Wrench6{});
  for (int k = 0; k < 3; ++k)
    EXPECT_FALSE(c.AccumulateBias(rest, grav)) << "committed early at sample " << k;
  EXPECT_TRUE(c.AccumulateBias(rest, grav));
  for (std::size_t i = 0; i < kWrenchDim; ++i)
    EXPECT_NEAR(c.bias()[i], bias[i], 1e-12) << "component " << i;

  // At the calibration pose with no external load the chain output is 0 — the
  // property that makes "bias then gravity" self-consistent.
  const Wrench6 idle = c.Apply(rest, grav);
  for (std::size_t i = 0; i < kWrenchDim; ++i)
    EXPECT_NEAR(idle[i], 0.0, 1e-12) << "component " << i;

  // A real load survives, shrunk by the deadband only.
  const Wrench6 signal{3.0, 0.2, 0.0, 0.0, 0.0, 0.0};
  const Wrench6 out = c.Apply(raw_with(signal), grav);
  EXPECT_NEAR(out[0], 3.0 - 0.5, 1e-12);  // deadband shrink, not zeroed
  EXPECT_NEAR(out[1], 0.0, 1e-12);        // 0.2 < 0.5 ⇒ inside the band
  EXPECT_NEAR(out[2], 0.0, 1e-12);        // gravity fully removed

  // Saturation is the LAST algebraic stage, so |out| ≤ max_abs holds on the
  // deadband-shrunk value, not on the raw one.
  const Wrench6 huge = c.Apply(raw_with(Wrench6{1e6, 0, 0, 0, 0, 0}), grav);
  EXPECT_DOUBLE_EQ(huge[0], 100.0);
  EXPECT_TRUE(c.saturated());
}

// bias_samples == 0 means "trust the sensor's own zeroing": the bias commits
// immediately as zero rather than stranding the FSM in BIAS_CALIBRATING forever.
TEST(WrenchConditioning, ZeroBiasSamplesCommitsImmediately) {
  WrenchConditioningConfig cfg;
  cfg.bias_samples = 0;
  cfg.filter_enabled = false;
  WrenchConditioner c;
  c.Configure(cfg, 500.0);
  EXPECT_TRUE(c.AccumulateBias(Wrench6{9, 9, 9, 9, 9, 9}, Wrench6{}));
  for (double v : c.bias())
    EXPECT_EQ(v, 0.0);
}

// D11 trap: BesselFilterN returns 0 for every sample before Init() and says
// nothing about it. Configure() is therefore mandatory — this pins that a
// configured conditioner passes a DC signal through at unity gain once seeded,
// so a regression that skips Configure() shows up as a silent zero here.
TEST(WrenchConditioning, SeededFilterPassesDcUnchanged) {
  WrenchConditioningConfig cfg;
  cfg.filter_enabled = true;
  cfg.filter_cutoff_force_hz = 20.0;
  cfg.filter_cutoff_torque_hz = 15.0;
  cfg.bias_samples = 0;
  WrenchConditioner c;
  c.Configure(cfg, 500.0);

  const Wrench6 w{4.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  c.SeedFilter(w);
  const Wrench6 out = c.Apply(w, Wrench6{});
  EXPECT_NEAR(out[0], 4.0, 1e-9) << "filter not seeded — activation would step";
  EXPECT_NEAR(out[5], 1.0, 1e-9);

  // Unseeded, the first sample is heavily attenuated (a step response) — the
  // contrast is what makes the assertion above non-vacuous.
  WrenchConditioner c2;
  c2.Configure(cfg, 500.0);
  const Wrench6 cold = c2.Apply(w, Wrench6{});
  EXPECT_LT(std::abs(cold[0]), 1.0);
}

// SeedFromSample is what activation actually uses: it seeds at the CONDITIONED
// steady state of a raw sample, not at zero. Seeding at zero would be right only
// for an unloaded arm — with a standing load (holding an object) it reintroduces
// exactly the step response §3.3 requires seeding to prevent.
TEST(WrenchConditioning, SeedFromSampleHandlesAStandingLoad) {
  WrenchConditioningConfig cfg;
  cfg.filter_enabled = true;
  cfg.bias_samples = 0;  // sensor pre-zeroed ⇒ bias 0, the load is real
  WrenchConditioner c;
  c.Configure(cfg, 500.0);

  const Wrench6 standing{12.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  c.SeedFromSample(standing, Wrench6{});
  const Wrench6 out = c.Apply(standing, Wrench6{});
  EXPECT_NEAR(out[0], 12.0, 1e-9) << "activation with a standing load stepped through the filter";

  // Seeding at zero instead would attenuate the very first sample — the contrast
  // that makes the assertion above about SeedFromSample and not about the filter.
  WrenchConditioner z;
  z.Configure(cfg, 500.0);
  z.SeedFilter(Wrench6{});
  EXPECT_LT(std::abs(z.Apply(standing, Wrench6{})[0]), 11.0);
}

TEST(WrenchConditioning, ConfigureRejectsCutoffAtOrAboveNyquist) {
  WrenchConditioningConfig cfg;
  cfg.filter_cutoff_force_hz = 250.0;  // == Nyquist at 500 Hz
  WrenchConditioner c;
  EXPECT_THROW(c.Configure(cfg, 500.0), std::runtime_error);
  cfg.filter_cutoff_force_hz = 20.0;
  EXPECT_NO_THROW(c.Configure(cfg, 500.0));
  cfg.deadband[0] = -1.0;
  EXPECT_THROW(c.Configure(cfg, 500.0), std::runtime_error);
}

// Zero-allocation gate for the whole RT-side wrench path (§11.7 T7.1): the
// conditioning chain, the bias accumulator and the staleness read must not
// touch the heap, from the very first call.
TEST(WrenchConditioning, RtPathIsAllocationFree) {
  WrenchConditioningConfig cfg;
  cfg.filter_enabled = true;
  cfg.bias_samples = 2;
  WrenchConditioner c;
  c.Configure(cfg, 500.0);
  WrenchInput in;
  const Wrench6 raw{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
  const Wrench6 grav{0.0, 0.0, -5.0, 0.0, 0.0, 0.0};
  in.Set(Span6(raw));
  {
    rtc::testing::ScopedNoMalloc guard;
    const auto r = in.Read(0.002, 0.05, 0.1);
    (void)c.AccumulateBias(r.value, grav);
    const Wrench6 out = c.Apply(r.value, grav);
    c.SeedFilter(out);
    EXPECT_EQ(guard.violations(), 0u) << "wrench RT path allocated";
  }
}

// ── state machine: the wrench-driven §10.6 states ───────────────────────────

TEST(StateMachine, BiasCalibratingPrecedesHoldingAndRunning) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  sm.BeginBiasCalibration();
  EXPECT_EQ(sm.state(), ComplianceState::kBiasCalibrating);
  // Ramp completion cannot skip the calibration.
  EXPECT_EQ(sm.Step(f, /*ramp_done=*/true, 0.002, 0.5, /*bias_done=*/false),
            ComplianceState::kBiasCalibrating);
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5, /*bias_done=*/true), ComplianceState::kHolding);
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5, true), ComplianceState::kRunning);
}

TEST(StateMachine, ContactSplitFollowsHysteresisInput) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  ASSERT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kRunning);
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5, true, /*in_contact=*/true),
            ComplianceState::kRunningContact);
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5, true, /*in_contact=*/false), ComplianceState::kRunning);
  EXPECT_TRUE(IsRunning(ComplianceState::kRunningContact));
  EXPECT_TRUE(IsRunning(ComplianceState::kRunning));
  EXPECT_FALSE(IsRunning(ComplianceState::kDegraded));
}

// §10.6's central claim: a dead wrench source is the MIDDLE state. It degrades
// and recovers; it never latches SAFE_STOP. A regression that promoted it to
// critical would stop the arm every time a 100 Hz estimator hiccups.
TEST(StateMachine, WrenchTimeoutDegradesButNeverLatches) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  ASSERT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kRunning);
  f.wrench_timeout = true;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kDegraded);
  for (int k = 0; k < 100; ++k)
    EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kDegraded) << "tick " << k;
  EXPECT_FALSE(sm.in_safe_stop());
  // Source returns → recovers after the dwell, no ResetFault() needed.
  f.wrench_timeout = false;
  EXPECT_EQ(sm.Step(f, true, 0.3, 0.5), ComplianceState::kDegraded);
  EXPECT_EQ(sm.Step(f, true, 0.3, 0.5), ComplianceState::kRunning);

  // quality_low behaves identically (both are §10.6 degrade causes).
  f.quality_low = true;
  EXPECT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kDegraded);
  EXPECT_FALSE(sm.in_safe_stop());
}

// A DEGRADED source that recovers while in contact returns to RUNNING_CONTACT,
// not to free space — otherwise the diagnostics would claim the contact ended.
TEST(StateMachine, DegradedRecoversIntoTheContactStateItLeft) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  sm.Step(f, true, 0.002, 0.5);
  f.wrench_timeout = true;
  ASSERT_EQ(sm.Step(f, true, 0.002, 0.5, true, true), ComplianceState::kDegraded);
  f.wrench_timeout = false;
  EXPECT_EQ(sm.Step(f, true, 0.6, 0.5, true, /*in_contact=*/true),
            ComplianceState::kRunningContact);
}

// E-8: re-activation runs BeginBiasCalibration, and re-activation happens on the
// tick after every hold tick — so if the latch did not win here, a latched
// SAFE_STOP would silently wash out one tick later.
TEST(StateMachine, BeginBiasCalibrationCannotLaunderSafeStop) {
  ComplianceStateMachine sm;
  ComplianceFaults f;
  f.nan_inf = true;
  ASSERT_EQ(sm.Step(f, true, 0.002, 0.5), ComplianceState::kSafeStop);
  sm.BeginBiasCalibration();
  EXPECT_EQ(sm.state(), ComplianceState::kSafeStop) << "SAFE_STOP laundered by re-activation";
  // ResetFault remains the only exit, and it lands in HOLDING (unchanged).
  sm.ResetFault();
  EXPECT_EQ(sm.state(), ComplianceState::kHolding);
  sm.BeginBiasCalibration();
  EXPECT_EQ(sm.state(), ComplianceState::kBiasCalibrating);
}

TEST(TorqueEstop, NonFiniteInputForcedToZero) {
  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  const double inf_v = std::numeric_limits<double>::infinity();
  Eigen::VectorXd tau(4), g(4), qdot(4), tau_max(4);
  g << 5.0, nan_v, 2.0, inf_v;   // joint 1: NaN gravity, joint 3: ∞ gravity
  qdot << 0.0, 0.0, nan_v, 0.0;  // joint 2: NaN velocity
  tau_max << 100.0, 100.0, 100.0, 100.0;
  GravityCompDampedHold(tau, g, qdot, /*damping=*/2.0, tau_max);
  EXPECT_TRUE(tau.allFinite()) << "non-finite input leaked to the actuator";
  EXPECT_DOUBLE_EQ(tau(0), 5.0);  // finite joint unaffected
  EXPECT_DOUBLE_EQ(tau(1), 0.0);  // NaN gravity → 0
  EXPECT_DOUBLE_EQ(tau(2), 0.0);  // NaN velocity → 0
  EXPECT_DOUBLE_EQ(tau(3), 0.0);  // ∞ gravity → 0 (not clamped to +100)
}

}  // namespace
