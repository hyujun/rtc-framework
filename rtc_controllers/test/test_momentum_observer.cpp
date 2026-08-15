// Layer 1 of #135 — generalized-momentum observer residual.
//
// Every case below drives the observer from a QUASI-STATIC robot: q̇ == 0, so
// the momentum p is identically zero and the Coriolis product vanishes. That is
// not a convenience — it is [A3], the assumption Layer 2A is built on, and it
// makes the oracle exact rather than approximate. From [MO-1] at rest,
//
//   0 + 0 + g(q) = tau_m + tau_ext   ⇒   tau_m = g - tau_ext
//
// so a test states the external torque it wants and derives the measured torque
// that a real arm would report under it. beta = tau_m + C^T v - g then equals
// -tau_ext exactly, which is what lets the convergence assertions below be tight
// instead of eyeballed.
#include "rtc_controllers/estimation/momentum_observer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {

using rtc::estimation::MomentumInvalidReason;
using rtc::estimation::MomentumObserver;

constexpr int kDof = 6;
constexpr double kDt = 0.002;  // 500 Hz, the default control rate

std::vector<double> Gains(double k, int n = kDof) { return std::vector<double>(static_cast<std::size_t>(n), k); }

/// Gravity torque of some non-degenerate posture. Arbitrary but fixed: the
/// observer must cancel it exactly, so a nonzero g is a stronger input than a
/// zero one (a bug that drops the -g term is invisible when g == 0).
const std::vector<double> kGravity{12.0, -30.5, 7.25, -1.5, 0.75, -0.125};

const std::vector<double> kZero(kDof, 0.0);

/// tau_m a resting arm reports while `tau_ext` is applied to it.
std::vector<double> MeasuredTorque(const std::vector<double>& tau_ext) {
  std::vector<double> tau_m(kDof);
  for (int i = 0; i < kDof; ++i)
    tau_m[static_cast<std::size_t>(i)] =
        kGravity[static_cast<std::size_t>(i)] - tau_ext[static_cast<std::size_t>(i)];
  return tau_m;
}

/// One quasi-static tick under a constant external torque.
void TickAtRest(MomentumObserver& obs, const std::vector<double>& tau_m, double dt = kDt) {
  obs.Update(kZero, kZero, kGravity, tau_m, dt);
}

double MaxAbsError(std::span<const double> got, const std::vector<double>& want) {
  double m = 0.0;
  for (int i = 0; i < kDof; ++i)
    m = std::max(m, std::fabs(got[static_cast<std::size_t>(i)] - want[static_cast<std::size_t>(i)]));
  return m;
}

MomentumObserver MakeObserver(double gain) {
  MomentumObserver obs;
  obs.Init(kDof, Gains(gain));
  return obs;
}

}  // namespace

// ── AC4-a — the null case ────────────────────────────────────────────────────

// A motionless, unloaded arm must produce NOTHING. This is the weak half of
// AC4 on its own (a do-nothing implementation also passes it), which is why
// ConstantExternalTorque* below is its mandatory positive control.
TEST(MomentumObserver, StationaryAndUnloadedKeepsResidualAtZero) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_m = MeasuredTorque(kZero);  // == gravity

  for (int k = 0; k < 2000; ++k) {
    TickAtRest(obs, tau_m);
    ASSERT_TRUE(obs.valid()) << "tick " << k;
  }
  EXPECT_LT(MaxAbsError(obs.residual(), kZero), 1e-12);
}

// ── AC4-b — the positive control ─────────────────────────────────────────────

// The residual must reproduce a KNOWN constant external torque in steady state.
// 5 s at K_I = 20 is 100 time constants, so the remaining error is numerical
// rather than dynamic.
TEST(MomentumObserver, ConstantExternalTorqueConvergesToThatTorque) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_ext{3.0, -5.0, 1.5, 0.25, -0.75, 2.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);

  for (int k = 0; k < 2500; ++k)
    TickAtRest(obs, tau_m);

  ASSERT_TRUE(obs.valid());
  EXPECT_LT(MaxAbsError(obs.residual(), tau_ext), 1e-9);
}

// The approach is monotone from zero and never overshoots — the signature of a
// first-order lag. An observer that ramped (the [MO-3c] mutation) would sail
// past tau_ext here rather than settling on it.
TEST(MomentumObserver, ResidualApproachesFromZeroWithoutOvershoot) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_ext{4.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);

  double prev = 0.0;
  for (int k = 0; k < 2500; ++k) {
    TickAtRest(obs, tau_m);
    const double r = obs.residual()[0];
    EXPECT_GE(r, prev - 1e-15) << "not monotone at tick " << k;
    EXPECT_LE(r, 4.0 + 1e-12) << "overshoot at tick " << k;
    prev = r;
  }
}

// ── [MO-2] — the first-order dynamics the +r_k feedback creates ──────────────

// dot r = K_I (tau_ext - r) means the time constant is 1/K_I, so doubling the
// gain must halve the time to 63.2 %. This is the case that distinguishes "some
// filter" from "the filter [MO-2] specifies"; a pure integrator has no time
// constant to halve.
TEST(MomentumObserver, DoublingTheGainHalvesTheConvergenceTime) {
  const std::vector<double> tau_ext{6.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);
  constexpr double kTarget = 0.632 * 6.0;

  auto ticks_to_target = [&](double gain) {
    MomentumObserver obs = MakeObserver(gain);
    for (int k = 1; k <= 100000; ++k) {
      TickAtRest(obs, tau_m);
      if (obs.residual()[0] >= kTarget)
        return k;
    }
    return -1;
  };

  const int n_slow = ticks_to_target(5.0);
  const int n_fast = ticks_to_target(10.0);
  ASSERT_GT(n_slow, 0);
  ASSERT_GT(n_fast, 0);

  // 1/K_I is the continuous-time constant; the discrete pole differs by
  // O(dt·K_I) (~1 % here), so the ratio is checked with a band rather than an
  // equality.
  const double ratio = static_cast<double>(n_slow) / static_cast<double>(n_fast);
  EXPECT_NEAR(ratio, 2.0, 0.1) << "n_slow=" << n_slow << " n_fast=" << n_fast;

  // And the absolute scale is right, not just the ratio: 1/5 s at 500 Hz = 100
  // ticks. Without this a mutation that broke both runs equally would slip
  // through on the ratio alone.
  EXPECT_NEAR(n_slow, 100, 5);
}

// ── AC5 (core half) — Hold() ─────────────────────────────────────────────────

// The caller's lane gate reaching the observer must FREEZE it, not zero it and
// not advance it. Zeroing would assert "no external torque", which is the
// fabricated measurement this whole path exists to avoid.
TEST(MomentumObserver, HoldFreezesTheResidualAndReportsInvalid) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_ext{3.0, -5.0, 1.5, 0.25, -0.75, 2.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);

  for (int k = 0; k < 2500; ++k)
    TickAtRest(obs, tau_m);
  ASSERT_TRUE(obs.valid());

  std::array<double, kDof> before{};
  for (int i = 0; i < kDof; ++i)
    before[static_cast<std::size_t>(i)] = obs.residual()[static_cast<std::size_t>(i)];

  for (int k = 0; k < 50; ++k) {
    obs.Hold();
    EXPECT_FALSE(obs.valid());
    EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kHeld);
    for (int i = 0; i < kDof; ++i) {
      EXPECT_DOUBLE_EQ(obs.residual()[static_cast<std::size_t>(i)],
                       before[static_cast<std::size_t>(i)])
          << "hold " << k << " joint " << i;
    }
  }
  EXPECT_EQ(obs.ticks_since_seed(), 0u);
}

// A gap is not a pause. While Hold() is in force the momentum keeps evolving
// unobserved, so resuming against the OLD reference would charge the entire
// unseen momentum change to tau_ext. Here the arm is unloaded throughout and
// merely moved during the gap; the residual after resuming must stay near zero
// rather than reporting the jump as a load.
TEST(MomentumObserver, ResumingAfterAGapDoesNotChargeUnobservedMotionToTheResidual) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> unloaded = MeasuredTorque(kZero);

  for (int k = 0; k < 1000; ++k)
    TickAtRest(obs, unloaded);
  ASSERT_LT(MaxAbsError(obs.residual(), kZero), 1e-12);

  for (int k = 0; k < 100; ++k)
    obs.Hold();

  // Resume with a momentum far from the pre-gap reference (the arm swung while
  // the lane was closed) but still no external load.
  const std::vector<double> p_after{40.0, -25.0, 10.0, 5.0, -2.0, 1.0};
  for (int k = 0; k < 500; ++k)
    obs.Update(p_after, kZero, kGravity, unloaded, kDt);

  ASSERT_TRUE(obs.valid());
  EXPECT_LT(MaxAbsError(obs.residual(), kZero), 1e-9)
      << "the gap's momentum change leaked into the residual";
  EXPECT_GT(obs.ticks_since_seed(), 0u);
}

// ── Rejected ticks ───────────────────────────────────────────────────────────

TEST(MomentumObserver, NonFiniteInputIsRejectedRatherThanLaundered) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_ext{3.0, -5.0, 1.5, 0.25, -0.75, 2.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);
  for (int k = 0; k < 2500; ++k)
    TickAtRest(obs, tau_m);

  const double before = obs.residual()[0];

  std::vector<double> bad = tau_m;
  bad[2] = std::numeric_limits<double>::quiet_NaN();
  obs.Update(kZero, kZero, kGravity, bad, kDt);

  EXPECT_FALSE(obs.valid());
  EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kNonFiniteInput);
  EXPECT_DOUBLE_EQ(obs.residual()[0], before);
  for (int i = 0; i < kDof; ++i)
    EXPECT_TRUE(std::isfinite(obs.residual()[static_cast<std::size_t>(i)]));
}

TEST(MomentumObserver, NonPositiveOrNonFiniteDtIsRejected) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_m = MeasuredTorque(kZero);
  TickAtRest(obs, tau_m);

  for (const double dt : {0.0, -kDt, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
    TickAtRest(obs, tau_m, dt);
    EXPECT_FALSE(obs.valid()) << "dt=" << dt;
    EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kNonPositiveDt) << "dt=" << dt;
  }
}

TEST(MomentumObserver, ShortInputSpanIsRejected) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_m = MeasuredTorque(kZero);
  const std::vector<double> too_short(kDof - 1, 0.0);

  obs.Update(too_short, kZero, kGravity, tau_m, kDt);
  EXPECT_FALSE(obs.valid());
  EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kShortInput);

  obs.Update(kZero, kZero, kGravity, too_short, kDt);
  EXPECT_FALSE(obs.valid());
  EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kShortInput);
}

TEST(MomentumObserver, UpdateBeforeInitIsRejected) {
  MomentumObserver obs;
  EXPECT_FALSE(obs.valid());
  EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kNotInitialized);

  obs.Update(kZero, kZero, kGravity, kGravity, kDt);
  EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kNotInitialized);

  obs.Hold();
  EXPECT_EQ(obs.invalid_reason(), MomentumInvalidReason::kNotInitialized);
}

// ── Init validation (non-RT, may throw) ──────────────────────────────────────

TEST(MomentumObserver, InitRejectsBadConfiguration) {
  MomentumObserver obs;
  EXPECT_THROW(obs.Init(0, Gains(1.0, 1)), std::invalid_argument);
  EXPECT_THROW(obs.Init(-1, Gains(1.0, 1)), std::invalid_argument);
  EXPECT_THROW(obs.Init(rtc::estimation::kMaxObserverDof + 1,
                        Gains(1.0, rtc::estimation::kMaxObserverDof + 1)),
               std::invalid_argument);
  // gains shorter than nv
  EXPECT_THROW(obs.Init(kDof, Gains(1.0, kDof - 1)), std::invalid_argument);
  // A zero gain would pin that joint's residual at 0 forever — rejected rather
  // than read as "disable this joint".
  {
    std::vector<double> g = Gains(1.0);
    g[3] = 0.0;
    EXPECT_THROW(obs.Init(kDof, g), std::invalid_argument);
  }
  {
    std::vector<double> g = Gains(1.0);
    g[1] = -2.0;
    EXPECT_THROW(obs.Init(kDof, g), std::invalid_argument);
  }
  {
    std::vector<double> g = Gains(1.0);
    g[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(obs.Init(kDof, g), std::invalid_argument);
  }
  EXPECT_FALSE(obs.initialized());

  EXPECT_NO_THROW(obs.Init(kDof, Gains(15.0)));
  EXPECT_TRUE(obs.initialized());
  EXPECT_EQ(obs.dof(), kDof);
  EXPECT_EQ(obs.residual().size(), static_cast<std::size_t>(kDof));
}

// Per-joint gains really are per joint: joint 0 converges faster than joint 1.
TEST(MomentumObserver, PerJointGainsActIndependently) {
  MomentumObserver obs;
  std::vector<double> gains = Gains(5.0);
  gains[0] = 40.0;
  obs.Init(kDof, gains);

  const std::vector<double> tau_ext{2.0, 2.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);
  for (int k = 0; k < 60; ++k)  // ~0.12 s: past 1/40 but well short of 1/5
    TickAtRest(obs, tau_m);

  EXPECT_GT(obs.residual()[0], 0.9 * 2.0);
  EXPECT_LT(obs.residual()[1], 0.6 * 2.0);
}

TEST(MomentumObserver, ResetRtStateClearsTheEstimateButKeepsInit) {
  MomentumObserver obs = MakeObserver(20.0);
  const std::vector<double> tau_ext{3.0, -5.0, 1.5, 0.25, -0.75, 2.0};
  const std::vector<double> tau_m = MeasuredTorque(tau_ext);
  for (int k = 0; k < 2500; ++k)
    TickAtRest(obs, tau_m);
  ASSERT_GT(std::fabs(obs.residual()[0]), 1.0);

  obs.ResetRtState();
  EXPECT_FALSE(obs.valid());
  EXPECT_TRUE(obs.initialized());
  EXPECT_EQ(obs.dof(), kDof);
  EXPECT_EQ(obs.ticks_since_seed(), 0u);
  EXPECT_LT(MaxAbsError(obs.residual(), kZero), 1e-15);

  TickAtRest(obs, tau_m);
  EXPECT_TRUE(obs.valid());
  EXPECT_EQ(obs.ticks_since_seed(), 1u);
}
