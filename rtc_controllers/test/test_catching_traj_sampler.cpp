// L2 predicted-trajectory sampler — gates G2-A..E, G2-G, G2-H
// (docs/dynamic_catching/L2_prediction.md §9) plus the count / NaN / spacing
// items of L1 G1-A that belong to the ROS-free core.
//
// Ported from the reference docs/dynamic_catching/test_l2.cpp with thresholds
// unchanged. Two fixture changes, both forced by the port and approved at S1
// kick-off (sub-plan F-3):
//  • the reference built 60 points at 1/60 s; the snapshot holds kCap = 40, so
//    the fixture is 40 points (horizon 0.65 s) and every in-horizon sweep stops
//    at the last sample instead of 0.9 s;
//  • sample instants are integer steady ns now, so 1/60 s is rounded to the ns.
//    The reference truth is therefore propagated to the ROUNDED instants — a
//    truth taken at the unrounded time would differ by ‖v‖·0.5 ns ≈ 2.5e-9 m and
//    swamp G2-C's 1e-10 m.
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_sampler.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/catching_ball_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <string>
#include <type_traits>

namespace {

using rtc::catching::BallTime;
using rtc::catching::Check;
using rtc::catching::Interpolate;
using rtc::catching::kCap;
using rtc::catching::MakeNowLead;
using rtc::catching::NowLead;
using rtc::catching::NowReal;
using rtc::catching::SampleAt;
using rtc::catching::SampleEval;
using rtc::catching::TrajCheck;
using rtc::catching::TrajectorySnapshot;
using rtc::catching::TrajLimits;
using rtc::catching::TrajReject;
using rtc::catching::TrajSample;
namespace fx = rtc::catching::fixture;

constexpr std::int64_t kT0 = 1'000'000'000'000;  // origin: 1000 s of steady time
constexpr std::int64_t kCamNs = 16'666'667;      // 1/60 s, rounded to the ns
constexpr std::int64_t kRtNs = 2'000'000;        // RT tick 2 ms
constexpr double kDrag = 0.0229;

fx::BallState InitialState(double drag_k) {
  fx::BallState x;
  x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, drag_k;
  return x;
}

/// Truth at the absolute instant t_ns (propagated from kT0 with fine steps).
fx::BallState TruthAt(std::int64_t t_ns, double drag_k) {
  fx::BallState x = InitialState(drag_k);
  const double t = static_cast<double>(t_ns - kT0) * 1e-9;
  if (t > 0.0)
    (void)fx::Propagate(fx::BallModel{}, x, t, 1e-4, 1 << 22);
  return x;
}

void Store(const fx::BallState& x, TrajSample& s) {
  const fx::BallState dx = fx::F(fx::BallModel{}, x);
  for (std::size_t k = 0; k < 3; ++k) {
    const Eigen::Index i = static_cast<Eigen::Index>(k);
    s.p[k] = x(i);
    s.v[k] = x(3 + i);
    s.a[k] = dx(3 + i);
  }
}

/// A vision-like prediction: n samples of the true flight, `spacing_ns` apart.
TrajectorySnapshot MakeTraj(int n, double drag_k, std::int64_t spacing_ns = kCamNs) {
  TrajectorySnapshot tr{};
  tr.n = n;
  tr.valid = true;
  tr.token.snapshot_sequence = 1;
  fx::BallState x = InitialState(drag_k);
  std::int64_t t_prev = kT0;
  for (int i = 0; i < n; ++i) {
    TrajSample& s = tr.s[static_cast<std::size_t>(i)];
    s.t_ns = kT0 + i * spacing_ns;
    if (s.t_ns > t_prev)
      (void)fx::Propagate(fx::BallModel{}, x, static_cast<double>(s.t_ns - t_prev) * 1e-9, 1e-4,
                          1 << 22);
    t_prev = s.t_ns;
    Store(x, s);
  }
  return tr;
}

std::int64_t Last(const TrajectorySnapshot& tr) {
  return tr.s[static_cast<std::size_t>(tr.n - 1)].t_ns;
}

std::string Sci(double x) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3e", x);
  return b;
}

const TrajSample& At(const TrajectorySnapshot& tr, int i) {
  return tr.s[static_cast<std::size_t>(i)];
}

Eigen::Vector3d V3(const std::array<double, 3>& a) {
  return Eigen::Vector3d(a[0], a[1], a[2]);
}

TrajLimits Limits(int n_min = 10) {
  TrajLimits lim;
  lim.n_min = n_min;
  lim.n_max = kCap;
  lim.dt_min_ns = 1'000'000;  // 1 ms
  return lim;
}

// ── ported reference cases ──────────────────────────────────────────────────

// G2-A: the interpolant reproduces p, v, a at BOTH ends of every interval.
// Checking s = 0 alone leaves every basis derivative except D1(0) unconstrained.
TEST(CatchingTrajSampler, G2AReproducesSamplesAtBothEnds) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  double ep = 0;
  double ev = 0;
  double ea = 0;
  for (int i = 0; i + 1 < tr.n; ++i) {
    const TrajSample& A = At(tr, i);
    const TrajSample& B = At(tr, i + 1);
    const SampleEval e0 = Interpolate(A, B, BallTime{A.t_ns});
    const SampleEval e1 = Interpolate(A, B, BallTime{B.t_ns});
    ASSERT_TRUE(e0.valid && e1.valid);
    ep = std::max({ep, (e0.p - V3(A.p)).norm(), (e1.p - V3(B.p)).norm()});
    ev = std::max({ev, (e0.v - V3(A.v)).norm(), (e1.v - V3(B.v)).norm()});
    ea = std::max({ea, (e0.a - V3(A.a)).norm(), (e1.a - V3(B.a)).norm()});
  }
  EXPECT_LT(ep, 1e-12);
  EXPECT_LT(ev, 1e-12);
  EXPECT_LT(ea, 1e-12);
}

// Supports G2-A: inside an interval v and a are the true derivatives of p,
// which is what pins the D·/S· coefficients and their powers of h.
TEST(CatchingTrajSampler, G2AVelocityAndAccelAreDerivativesInside) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  const std::int64_t d_ns = 1'000;  // 1 µs
  const double d = 1e-6;
  double ev = 0;
  double ea = 0;
  for (int i = 0; i + 1 < tr.n; i += 7) {
    const TrajSample& A = At(tr, i);
    const TrajSample& B = At(tr, i + 1);
    for (double s : {0.15, 0.37, 0.5, 0.63, 0.88}) {
      const std::int64_t t =
          A.t_ns + static_cast<std::int64_t>(s * static_cast<double>(B.t_ns - A.t_ns));
      const SampleEval c = Interpolate(A, B, BallTime{t});
      const SampleEval m = Interpolate(A, B, BallTime{t - d_ns});
      const SampleEval p = Interpolate(A, B, BallTime{t + d_ns});
      ev = std::max(ev, (c.v - (p.p - m.p) / (2 * d)).norm());
      ea = std::max(ea, (c.a - (p.p - 2 * c.p + m.p) / (d * d)).norm());
    }
  }
  EXPECT_LT(ev, 1e-6);
  EXPECT_LT(ea, 1e-2);
}

// G2-B: C² — acceleration is continuous across sample boundaries.
TEST(CatchingTrajSampler, G2BAccelContinuousAcrossSamples) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  const std::int64_t d = 100;  // 0.1 µs
  double jump = 0.0;
  for (int i = 1; i < tr.n - 1; ++i) {
    const std::int64_t t = At(tr, i).t_ns;
    jump = std::max(jump, (SampleAt(tr, NowLead{t + d}).a - SampleAt(tr, NowLead{t - d}).a).norm());
  }
  EXPECT_LT(jump, 1e-6);
}

// G2-C: a drag-free (quadratic) flight lies in the quintic basis — exact.
TEST(CatchingTrajSampler, G2CExactForPureGravity) {
  const TrajectorySnapshot tr = MakeTraj(kCap, 0.0);
  double worst = 0.0;
  for (std::int64_t t = kT0; t <= Last(tr); t += kRtNs)
    worst = std::max(worst, (SampleAt(tr, NowLead{t}).p - TruthAt(t, 0.0).head<3>()).norm());
  EXPECT_LT(worst, 1e-10);
}

// Reference case 4: with drag, the interpolation error at 60 Hz spacing.
TEST(CatchingTrajSampler, DragModelErrorAt60HzSpacing) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  double wp = 0.0;
  double wa = 0.0;
  for (std::int64_t t = kT0; t <= Last(tr); t += kRtNs) {
    const fx::BallState y = TruthAt(t, kDrag);
    const SampleEval e = SampleAt(tr, NowLead{t});
    wp = std::max(wp, (e.p - y.head<3>()).norm());
    wa = std::max(wa, (e.a - fx::F(fx::BallModel{}, y).segment<3>(3)).norm());
  }
  RecordProperty("pos_err_60hz_m", Sci(wp));
  RecordProperty("acc_err_60hz_mps2", Sci(wa));
  EXPECT_LT(wp, 1e-9);
}

// L2 §7 S1.2b: re-measure the accuracy at the 0.05 s spacing of the sim
// profile (D-15: 17 points, 0.8 s). Recorded, not gated — L2 has no threshold
// for it; the value feeds S3.6's spacing choice.
TEST(CatchingTrajSampler, RecordsErrorAtSimProfileSpacing) {
  const TrajectorySnapshot tr = MakeTraj(17, kDrag, 50'000'000);
  double wp = 0.0;
  double wa = 0.0;
  for (std::int64_t t = kT0; t <= Last(tr); t += kRtNs) {
    const fx::BallState y = TruthAt(t, kDrag);
    const SampleEval e = SampleAt(tr, NowLead{t});
    ASSERT_TRUE(e.valid);
    wp = std::max(wp, (e.p - y.head<3>()).norm());
    wa = std::max(wa, (e.a - fx::F(fx::BallModel{}, y).segment<3>(3)).norm());
  }
  RecordProperty("pos_err_50ms_m", Sci(wp));
  RecordProperty("acc_err_50ms_mps2", Sci(wa));
  std::printf("[ record ] 0.05 s spacing, 17 pts: max pos err %.3e m, max accel err %.3e m/s^2\n",
              wp, wa);
  EXPECT_TRUE(std::isfinite(wp) && std::isfinite(wa));
}

// G2-D: horizon flags tell before from after; the last sample is not extrapolated.
TEST(CatchingTrajSampler, G2DHorizonFlagsDistinguishSides) {
  const TrajectorySnapshot tr = MakeTraj(20, kDrag);
  const SampleEval in = SampleAt(tr, NowLead{kT0 + 150'000'000});
  const SampleEval out = SampleAt(tr, NowLead{kT0 + 500'000'000});
  const SampleEval pre = SampleAt(tr, NowLead{kT0 - 10'000'000});
  const SampleEval end = SampleAt(tr, NowLead{Last(tr)});
  EXPECT_FALSE(in.extrapolated);
  EXPECT_TRUE(out.after_horizon);
  EXPECT_FALSE(out.before_horizon);
  EXPECT_TRUE(pre.before_horizon);
  EXPECT_FALSE(pre.after_horizon);
  EXPECT_FALSE(end.extrapolated);
  EXPECT_TRUE(end.valid);
}

// G2-D: the hint cursor gives exactly the binary-search answer.
TEST(CatchingTrajSampler, G2DHintCursorMatchesFreshSearch) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  int hint = 0;
  double worst = 0.0;
  for (std::int64_t t = kT0; t < kT0 + 950'000'000; t += kRtNs)
    worst = std::max(worst, (SampleAt(tr, NowLead{t}, hint).p - SampleAt(tr, NowLead{t}).p).norm());
  EXPECT_EQ(worst, 0.0);
  // A stale hint far ahead recovers too.
  hint = kCap - 2;
  const std::int64_t t = kT0 + 3 * kCamNs + 1'000;
  EXPECT_EQ((SampleAt(tr, NowLead{t}, hint).p - SampleAt(tr, NowLead{t}).p).norm(), 0.0);
  EXPECT_EQ(hint, 3);
}

// G2-D: format checker accepts a good snapshot and names a monotonicity break.
TEST(CatchingTrajSampler, G2DFormatCheck) {
  TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  const TrajCheck ok = Check(tr, Limits());
  EXPECT_TRUE(ok.ok);
  EXPECT_EQ(ok.reason, TrajReject::kNone);
  EXPECT_EQ(ok.horizon_ns, (kCap - 1) * kCamNs);
  EXPECT_EQ(ok.dt_min_ns, kCamNs);
  EXPECT_EQ(ok.dt_max_ns, kCamNs);

  tr.s[30].t_ns = tr.s[29].t_ns;  // monotonicity violation
  const TrajCheck bad = Check(tr, Limits());
  EXPECT_FALSE(bad.ok);
  EXPECT_FALSE(bad.t_monotonic);
  EXPECT_EQ(bad.reason, TrajReject::kNonMonotonic);
}

// G2-G: a non-increasing pair is reported invalid, not silently returned.
TEST(CatchingTrajSampler, G2GNonMonotonicPairIsInvalid) {
  TrajSample A{};
  TrajSample B{};
  A.t_ns = 100;
  A.p = {1, 2, 3};
  B.t_ns = 100;  // h == 0
  EXPECT_FALSE(Interpolate(A, B, BallTime{100}).valid);
  B.t_ns = 50;  // h < 0
  EXPECT_FALSE(Interpolate(A, B, BallTime{80}).valid);
  // SampleAt never SELECTS such a pair: both the hint path and the binary search
  // only return an interval with t[i] ≤ t < t[i+1], so h > 0 by construction
  // and the guard above is reachable only by a direct call. What the RT read
  // owes on a snapshot that skipped Check() is a finite answer everywhere.
  TrajectorySnapshot tr = MakeTraj(10, kDrag);
  tr.s[5].t_ns = tr.s[4].t_ns;      // zero-length pair
  tr.s[7].t_ns = tr.s[6].t_ns - 1;  // reversed pair
  int hint = 0;
  for (std::int64_t t = kT0 - 5'000'000; t < Last(tr) + 5'000'000; t += 100'000) {
    const SampleEval e = SampleAt(tr, NowLead{t}, hint);
    ASSERT_TRUE(e.valid) << t;
    ASSERT_TRUE(e.p.allFinite() && e.v.allFinite() && e.a.allFinite()) << t;
  }
}

// ── new regression cases (G2-H, G1-A count / NaN / spacing items) ───────────

// n beyond the capacity (or ≤ 0) is rejected BEFORE any indexing. Under the
// sanitizer build (-D_GLIBCXX_ASSERTIONS + ASan) an out-of-range read here is a
// hard failure, which is the half of G2-H a plain build cannot see.
TEST(CatchingTrajSampler, G2HCountOutOfRangeRejectedBeforeIndexing) {
  TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  for (std::int32_t n : {kCap + 1, kCap + 1000, 0, -1, std::numeric_limits<std::int32_t>::max(),
                         std::numeric_limits<std::int32_t>::min()}) {
    tr.n = n;
    const TrajCheck c = Check(tr, Limits());
    EXPECT_FALSE(c.ok) << n;
    EXPECT_EQ(c.reason, TrajReject::kCount) << n;
    int hint = 0;
    EXPECT_FALSE(SampleAt(tr, NowLead{kT0 + 100'000'000}, hint).valid) << n;
  }
  // Below the run-time minimum as well.
  tr.n = 5;
  EXPECT_EQ(Check(tr, Limits(10)).reason, TrajReject::kCount);
}

// Limits that exceed the compile-time capacity, or a missing dt_min, are
// themselves rejected (fail-closed: the S3.6 n_max must fit kCap).
TEST(CatchingTrajSampler, G2HUnusableLimitsRejected) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  TrajLimits lim = Limits();
  lim.n_max = kCap + 1;
  EXPECT_EQ(Check(tr, lim).reason, TrajReject::kLimitsInvalid);
  lim = Limits();
  lim.dt_min_ns = 0;
  EXPECT_EQ(Check(tr, lim).reason, TrajReject::kLimitsInvalid);
  lim = Limits();
  lim.n_min = 0;
  EXPECT_EQ(Check(tr, lim).reason, TrajReject::kLimitsInvalid);
}

// NaN / Inf anywhere in a sample: Check rejects, and the RT read (which does
// not re-run Check) reports invalid instead of handing a NaN target on.
TEST(CatchingTrajSampler, G2HNonFiniteRejectedAndNeverSampledValid) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (int field = 0; field < 3; ++field) {
    for (double bad : {nan, inf}) {
      TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
      std::array<double, 3>& target =
          field == 0 ? tr.s[12].p : (field == 1 ? tr.s[12].v : tr.s[12].a);
      target[1] = bad;
      EXPECT_EQ(Check(tr, Limits()).reason, TrajReject::kNonFinite) << field;
      int hint = 0;
      const std::int64_t mid = (tr.s[11].t_ns + tr.s[12].t_ns) / 2;
      EXPECT_FALSE(SampleAt(tr, NowLead{mid}, hint).valid) << field;
      EXPECT_FALSE(SampleAt(tr, NowLead{tr.s[12].t_ns + 1}, hint).valid) << field;
    }
  }
  // Extrapolating from a non-finite end sample is invalid too.
  TrajectorySnapshot tr = MakeTraj(10, kDrag);
  tr.s[9].v[0] = nan;
  EXPECT_FALSE(SampleAt(tr, NowLead{Last(tr) + 10'000'000}).valid);
}

// An interval below dt_min is a rejection, not a warning (Interpolate divides
// by h²).
TEST(CatchingTrajSampler, G2HSpacingBelowDtMinRejected) {
  TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  tr.s[20].t_ns = tr.s[19].t_ns + 999'999;  // just under the 1 ms minimum
  const TrajCheck c = Check(tr, Limits());
  EXPECT_FALSE(c.ok);
  EXPECT_FALSE(c.spacing_ok);
  EXPECT_EQ(c.reason, TrajReject::kSpacing);
  tr.s[20].t_ns = tr.s[19].t_ns + 1'000'000;  // exactly the minimum is accepted
  EXPECT_TRUE(Check(tr, Limits()).ok);
}

TEST(CatchingTrajSampler, G2HSnapshotTypesAreTriviallyCopyable) {
  static_assert(std::is_trivially_copyable_v<TrajectorySnapshot>);
  static_assert(std::is_trivially_copyable_v<rtc::catching::PlanSnapshot>);
  SUCCEED();
}

// Sampling is on the lead axis (plan §3): with T_arm ≠ 0 the sample at
// MakeNowLead(now, T_arm) is the flight at now + T_arm, and the horizon flag
// trips T_arm before the real axis reaches the last sample.
TEST(CatchingTrajSampler, SamplesOnLeadAxisWithNonZeroTArm) {
  const TrajectorySnapshot tr = MakeTraj(kCap, kDrag);
  const std::int64_t t_arm = 50'000'000;
  const NowReal now{kT0 + 200'000'000};
  const SampleEval lead = SampleAt(tr, MakeNowLead(now, t_arm));
  const SampleEval direct = SampleAt(tr, NowLead{now.ns + t_arm});
  EXPECT_EQ((lead.p - direct.p).norm(), 0.0);
  EXPECT_GT((lead.p - SampleAt(tr, NowLead{now.ns}).p).norm(), 0.1);  // axes really differ

  const NowReal near_end{Last(tr) - t_arm / 2};
  EXPECT_TRUE(SampleAt(tr, MakeNowLead(near_end, t_arm)).after_horizon);
}

// G2-E: the RT read path allocates nothing (operator new AND Eigen's own
// allocator), and its cost is recorded: worst sample time and the bytes the
// per-tick snapshot copy moves (D-21 copies the whole kCap snapshot each tick).
TEST(CatchingTrajSampler, G2ERtReadPathAllocationFreeAndRecorded) {
  const TrajectorySnapshot src = MakeTraj(kCap, kDrag);
  TrajectorySnapshot copy{};
  double sink = 0.0;
  std::int64_t worst_ns = 0;
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    int hint = 0;
    for (std::int64_t t = kT0 - 20'000'000; t < Last(src) + 40'000'000; t += kRtNs) {
      const auto t0 = std::chrono::steady_clock::now();
      copy = src;  // the per-tick SeqLock Load copy
      const SampleEval e = SampleAt(copy, NowLead{t}, hint);
      const auto t1 = std::chrono::steady_clock::now();
      sink += e.p.x();
      worst_ns = std::max<std::int64_t>(
          worst_ns, std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }
    sink += Check(copy, Limits()).ok ? 1.0 : 0.0;
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_EQ(heap, 0u);
  EXPECT_EQ(eigen, 0u);
  EXPECT_TRUE(std::isfinite(sink));
  RecordProperty("snapshot_copy_bytes", std::to_string(sizeof(TrajectorySnapshot)));
  RecordProperty("worst_copy_plus_sample_ns", std::to_string(worst_ns));
  std::printf("[ record ] snapshot copy %zu B, worst copy+SampleAt %lld ns\n",
              sizeof(TrajectorySnapshot), static_cast<long long>(worst_ns));
}

}  // namespace
