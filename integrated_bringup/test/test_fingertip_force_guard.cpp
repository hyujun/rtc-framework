// Unit tests for the fingertip force delta-spike guard (FingertipForceGuard).
//
// The guard decides what the contact_stop force LPF is fed; it never touches
// the raw force. Each law below is stated once here against the type itself —
// the controller tests pin the WIRING (that the LPF really is fed this output
// and that the CSV carries it), not the arithmetic.
//
// The allocation counter is TU-global on purpose: the guard is header-only, so
// it is instantiated in this translation unit and its every heap touch is
// visible here (the rtc_base ScopedNoMalloc gate observes Eigen only, and this
// type uses no Eigen).

#include "integrated_bringup/controllers/fingertip_force_guard.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <new>

namespace {

// ── TU-global allocation counter ────────────────────────────────────────────
std::atomic<std::size_t> g_alloc_count{0};

}  // namespace

void* operator new(std::size_t size) {
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(size))
    return p;
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(size))
    return p;
  throw std::bad_alloc();
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

using integrated_bringup::FingertipForceGuard;
using integrated_bringup::FingertipForceGuardConfig;

// Production p1b values (integrated_bringup/config/ur5e_p1b/controllers/*.yaml).
constexpr FingertipForceGuardConfig kCfg{/*delta_max_n=*/4.0, /*max_hold_ticks=*/2};

std::array<double, 3> Fz(double fz) {
  return {0.0, 0.0, fz};
}

// Law 1: the first finite triplet is accepted as-is — there is nothing to
// compare it against, and rejecting it would leave the filter seeded from the
// safe zero for as long as the sensor keeps reporting a nonzero load.
TEST(FingertipForceGuard, FirstFiniteTripletIsAccepted) {
  FingertipForceGuard guard;
  EXPECT_FALSE(guard.primed());
  const auto r = guard.Apply({1.0, -2.0, 50.0}, kCfg);
  EXPECT_FALSE(r.rejected);
  EXPECT_DOUBLE_EQ(r.value[0], 1.0);
  EXPECT_DOUBLE_EQ(r.value[1], -2.0);
  // 50 N on the first sample is accepted: with no base, "large" is meaningless.
  EXPECT_DOUBLE_EQ(r.value[2], 50.0);
  EXPECT_TRUE(guard.primed());
}

// Law 2: in-band motion passes through untouched and re-bases the guard, so a
// slow ramp of any total size is never rejected.
TEST(FingertipForceGuard, InBandChangeIsAcceptedAndRebasesTheGuard) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(0.0), kCfg);
  double fz = 0.0;
  for (int i = 0; i < 10; ++i) {
    fz += 3.9;  // just under the 4 N threshold, every tick
    const auto r = guard.Apply(Fz(fz), kCfg);
    EXPECT_FALSE(r.rejected) << "tick " << i;
    EXPECT_DOUBLE_EQ(r.value[2], fz);
    EXPECT_DOUBLE_EQ(guard.last_accepted()[2], fz);
  }
  EXPECT_DOUBLE_EQ(guard.last_accepted()[2], 39.0);
}

// Law 3: one out-of-band axis holds the WHOLE triplet. The filter must never be
// fed a mix of two epochs (the accepted fx/fy of this tick with the held fz).
TEST(FingertipForceGuard, OneAxisOverThresholdHoldsAllThreeAxes) {
  FingertipForceGuard guard;
  (void)guard.Apply({0.5, -0.5, 0.02}, kCfg);
  const auto r = guard.Apply({0.6, -0.4, 6.4}, kCfg);
  EXPECT_TRUE(r.rejected);
  EXPECT_DOUBLE_EQ(r.value[0], 0.5);
  EXPECT_DOUBLE_EQ(r.value[1], -0.5);
  EXPECT_DOUBLE_EQ(r.value[2], 0.02);
  // Law 5: the base is unchanged, so the spike cannot re-baseline the guard.
  EXPECT_DOUBLE_EQ(guard.last_accepted()[2], 0.02);
}

// Law 4 + 6: the second spike tick is compared against the SAME base, not
// against the previous raw sample — otherwise a 2-tick spike would have its
// second tick accepted (|6.06 - 6.41| < 4) and the tail would reach the filter
// anyway. This is the exact shape of the 260730_1017 index/ring fz spikes.
TEST(FingertipForceGuard, SecondSpikeTickIsComparedAgainstLastAcceptedNotPreviousRaw) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(0.02), kCfg);
  const auto t1 = guard.Apply(Fz(6.406), kCfg);
  const auto t2 = guard.Apply(Fz(6.058), kCfg);
  EXPECT_TRUE(t1.rejected);
  EXPECT_TRUE(t2.rejected);
  EXPECT_DOUBLE_EQ(t1.value[2], 0.02);
  EXPECT_DOUBLE_EQ(t2.value[2], 0.02);
  EXPECT_EQ(guard.consecutive_rejections(), 2);

  // Law 5: raw returns to the true value and is accepted immediately — the
  // guard has not drifted toward the spike.
  const auto t3 = guard.Apply(Fz(0.04), kCfg);
  EXPECT_FALSE(t3.rejected);
  EXPECT_DOUBLE_EQ(t3.value[2], 0.04);
  EXPECT_EQ(guard.consecutive_rejections(), 0);
}

// The escape hatch. Holding forever whenever |Δ| stays over the threshold makes
// a genuine fast contact step unobservable — raw 6 N sustained is out of band on
// EVERY tick, so the filter would stay pinned at the pre-contact value and
// contact_stop could never fire. After max_hold_ticks the sensor's persistent
// opinion wins.
TEST(FingertipForceGuard, SustainedStepEscapesAfterHoldCap) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(0.0), kCfg);
  EXPECT_TRUE(guard.Apply(Fz(6.0), kCfg).rejected);
  EXPECT_TRUE(guard.Apply(Fz(6.0), kCfg).rejected);
  const auto accepted = guard.Apply(Fz(6.0), kCfg);
  EXPECT_FALSE(accepted.rejected) << "a sustained step must not be rejected forever";
  EXPECT_DOUBLE_EQ(accepted.value[2], 6.0);
  EXPECT_DOUBLE_EQ(guard.last_accepted()[2], 6.0);
  // Base moved with it: the next in-band tick is not a fresh spike.
  EXPECT_FALSE(guard.Apply(Fz(6.1), kCfg).rejected);
}

// max_hold_ticks = 0 disables the delta hold outright (the escape fires on the
// first out-of-band tick), which is the documented way to run a session with the
// guard's delta arm off. Non-finite rejection survives it — see below.
TEST(FingertipForceGuard, ZeroHoldCapDisablesDeltaHold) {
  const FingertipForceGuardConfig cfg{4.0, 0};
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(0.0), cfg);
  const auto r = guard.Apply(Fz(6.4), cfg);
  EXPECT_FALSE(r.rejected);
  EXPECT_DOUBLE_EQ(r.value[2], 6.4);
}

// Law 6/7 (NaN): a non-finite sample admitted into an IIR poisons the delay line
// permanently, and downstream max/clamp arithmetic then launders the NaN into a
// plausible number. It is rejected on every tick with NO escape — unlike the
// delta arm, persistence is not evidence here.
TEST(FingertipForceGuard, NanIsRejectedAndNeverEscapesTheHoldCap) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(1.0), kCfg);
  for (int i = 0; i < 10; ++i) {
    const auto r = guard.Apply({std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0}, kCfg);
    EXPECT_TRUE(r.rejected) << "tick " << i;
    EXPECT_TRUE(std::isfinite(r.value[0])) << "tick " << i;
    EXPECT_DOUBLE_EQ(r.value[2], 1.0) << "tick " << i;
  }
  EXPECT_DOUBLE_EQ(guard.last_accepted()[2], 1.0);
  // Recovery is immediate once the sensor reports finite values again.
  EXPECT_FALSE(guard.Apply(Fz(1.5), kCfg).rejected);
}

TEST(FingertipForceGuard, InfIsRejectedAndNeverEscapesTheHoldCap) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(1.0), kCfg);
  for (int i = 0; i < 10; ++i) {
    const auto r = guard.Apply({0.0, 0.0, std::numeric_limits<double>::infinity()}, kCfg);
    EXPECT_TRUE(r.rejected) << "tick " << i;
    EXPECT_DOUBLE_EQ(r.value[2], 1.0) << "tick " << i;
  }
  const auto neg = guard.Apply({0.0, -std::numeric_limits<double>::infinity(), 0.0}, kCfg);
  EXPECT_TRUE(neg.rejected);
  EXPECT_DOUBLE_EQ(neg.value[2], 1.0);
  EXPECT_DOUBLE_EQ(guard.last_accepted()[2], 1.0);
}

// Law 9: before the first finite sample there is no value to hold, so the guard
// reports the documented safe zero rather than whatever bits arrived.
TEST(FingertipForceGuard, NonFiniteBeforeFirstAcceptGivesSafeZero) {
  FingertipForceGuard guard;
  const auto r = guard.Apply(
      {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), 3.0},
      kCfg);
  EXPECT_TRUE(r.rejected);
  EXPECT_FALSE(guard.primed());
  EXPECT_DOUBLE_EQ(r.value[0], 0.0);
  EXPECT_DOUBLE_EQ(r.value[1], 0.0);
  EXPECT_DOUBLE_EQ(r.value[2], 0.0);

  // Still unprimed, so the first finite triplet is a first sample: accepted
  // whole, no matter how far it is from the safe zero.
  const auto first = guard.Apply(Fz(9.0), kCfg);
  EXPECT_FALSE(first.rejected);
  EXPECT_DOUBLE_EQ(first.value[2], 9.0);
}

// A dropout must drop the base with it. Comparing the re-entry force against a
// base the world has left behind (5 N lane returning at 0.2 N = a 4.8 N step)
// would hold the stale value indefinitely — and the LPF would then seed from it.
TEST(FingertipForceGuard, InvalidateMakesTheNextSampleAFirstSample) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(5.0), kCfg);
  guard.Invalidate();
  EXPECT_FALSE(guard.primed());
  EXPECT_EQ(guard.consecutive_rejections(), 0);

  const auto r = guard.Apply(Fz(0.2), kCfg);
  EXPECT_FALSE(r.rejected) << "re-entry after a dropout is a first sample";
  EXPECT_DOUBLE_EQ(r.value[2], 0.2);
}

// Threshold is exclusive: exactly 4.0 N of change is accepted, 4.0 + eps is not.
// Pins which side of the boundary the YAML value means.
TEST(FingertipForceGuard, ThresholdIsExclusive) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(0.0), kCfg);
  EXPECT_FALSE(guard.Apply(Fz(4.0), kCfg).rejected);

  FingertipForceGuard other;
  (void)other.Apply(Fz(0.0), kCfg);
  EXPECT_TRUE(other.Apply(Fz(4.0 + 1e-9), kCfg).rejected);
}

// RT-1: the guard runs inside the RT tick, so no path through Apply() may touch
// the heap. Covers accept, delta-reject, escape and non-finite reject.
TEST(FingertipForceGuard, ApplyDoesNotAllocate) {
  FingertipForceGuard guard;
  (void)guard.Apply(Fz(0.0), kCfg);  // warm up any lazy state before counting

  const std::size_t before = g_alloc_count.load(std::memory_order_relaxed);
  for (int i = 0; i < 1000; ++i) {
    (void)guard.Apply(Fz(0.01 * static_cast<double>(i % 3)), kCfg);  // accept
    (void)guard.Apply(Fz(6.4), kCfg);                                // reject
    (void)guard.Apply(Fz(6.4), kCfg);                                // reject
    (void)guard.Apply(Fz(6.4), kCfg);                                // escape-accept
    (void)guard.Apply({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, kCfg);
    guard.Invalidate();
    (void)guard.Apply(Fz(0.0), kCfg);
  }
  EXPECT_EQ(g_alloc_count.load(std::memory_order_relaxed), before)
      << "FingertipForceGuard::Apply allocated on the RT path";
}

}  // namespace
