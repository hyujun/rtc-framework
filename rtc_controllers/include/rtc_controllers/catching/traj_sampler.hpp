// ── Predicted-trajectory sampler (dynamic_catching S1.2, L2) ─────────────────
// Samples the vision prediction at the RT lead instant. The controller does NOT
// re-propagate the ball: it trusts the (p, v, a) samples vision sends and only
// interpolates between them (L2 §4.1).
//
// Interpolation is quintic Hermite: matching p, v, a at both ends makes the
// result C², so the feedforward term the reference generator builds from the
// target acceleration does not step at sample boundaries (a one-sample Taylor
// expansion p + v·dt + a·dt²/2 does — L2 §4.2).
//
// Time: sample instants and the query are absolute steady ns (plan §3). The
// query is the NowLead of the tick; the only relative seconds formed are the
// in-interval offset and length, both as differences of two instants of the
// same snapshot, so no origins are mixed.
//
// RT-safe: fixed size, no allocation, noexcept, no ROS. Check() is the non-RT
// acceptance gate run once on receipt; SampleAt() is the per-tick read and does
// not repeat it, but it still bounds `n` before indexing and reports a
// non-monotonic pair or non-finite result as invalid (a torn or corrupted
// snapshot must not come back as a valid sample).
#pragma once

#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rtc::catching {

/// Result of one sample. Plain Eigen members: this lives on the RT stack, it
/// is never a SeqLock payload.
struct SampleEval {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  // [m]
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};  // [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};  // [m/s²]
  // Horizon side is kept apart: when vision's first point is not at offset 0
  // (or the snapshot is a little old) before_horizon is normal operation, so
  // the supervisor watches after_horizon only (L2 §4.6).
  bool before_horizon{false};  // query < first sample
  bool after_horizon{false};   // query > last sample — catching forbidden here
  bool extrapolated{false};    // before_horizon || after_horizon
  bool valid{false};
};

/// Quintic Hermite basis on s ∈ [0, 1]: values H, first derivatives D and
/// second derivatives S (with respect to s).
struct HermiteBasis {
  double H0, H1, H2, H3, H4, H5;
  double D0, D1, D2, D3, D4, D5;
  double S0, S1, S2, S3, S4, S5;
};

[[nodiscard]] inline HermiteBasis Hermite5(double s) noexcept {
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  HermiteBasis b{};
  b.H0 = 1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5;
  b.H1 = s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5;
  b.H2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
  b.H3 = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  b.H4 = -4.0 * s3 + 7.0 * s4 - 3.0 * s5;
  b.H5 = 0.5 * s3 - s4 + 0.5 * s5;
  b.D0 = -30.0 * s2 + 60.0 * s3 - 30.0 * s4;
  b.D1 = 1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4;
  b.D2 = s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4;
  b.D3 = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
  b.D4 = -12.0 * s2 + 28.0 * s3 - 15.0 * s4;
  b.D5 = 1.5 * s2 - 4.0 * s3 + 2.5 * s4;
  b.S0 = -60.0 * s + 180.0 * s2 - 120.0 * s3;
  b.S1 = -36.0 * s + 96.0 * s2 - 60.0 * s3;
  b.S2 = 1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3;
  b.S3 = 60.0 * s - 180.0 * s2 + 120.0 * s3;
  b.S4 = -24.0 * s + 84.0 * s2 - 60.0 * s3;
  b.S5 = 3.0 * s - 12.0 * s2 + 10.0 * s3;
  return b;
}

namespace detail {

[[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> Vec(
    const std::array<double, 3>& x) noexcept {
  return Eigen::Map<const Eigen::Vector3d>(x.data());
}

/// A valid result must also be finite: a NaN that slipped past Check() (or a
/// corrupted read) is reported, never handed on as a target.
inline void MarkFinite(SampleEval& e) noexcept {
  e.valid = e.valid && e.p.allFinite() && e.v.allFinite() && e.a.allFinite();
}

}  // namespace detail

/// Structural floor on an interpolated interval [ns]. Check()'s dt_min is the
/// authoritative spacing gate at ingress; this only keeps the RT read from
/// turning a snapshot that bypassed Check() into an O(1/h²) finite-but-garbage
/// acceleration. 100 µs is far below any vision spacing (≥ 1/120 s).
inline constexpr std::int64_t kMinInterpIntervalNs = 100'000;

/// Interpolate inside [A, B] at ball instant t (clamped to the interval).
/// Returns invalid if the pair is shorter than kMinInterpIntervalNs (which
/// includes non-increasing) — Check() guarantees spacing on receipt, but the RT
/// read does not re-run Check().
[[nodiscard]] inline SampleEval Interpolate(const TrajSample& A, const TrajSample& B,
                                            BallTime t) noexcept {
  SampleEval e{};
  // Saturating: t_ns is wire data, so B − A can overflow int64 (UB).
  if (!(B.t_ns > A.t_ns) || detail::SatSub(B.t_ns, A.t_ns) < kMinInterpIntervalNs)
    return e;
  const double h = SecondsBetween(BallTime{A.t_ns}, BallTime{B.t_ns});
  const double s = std::clamp(SecondsBetween(BallTime{A.t_ns}, t) / h, 0.0, 1.0);
  const HermiteBasis b = Hermite5(s);
  const Eigen::Map<const Eigen::Vector3d> Ap = detail::Vec(A.p);
  const Eigen::Map<const Eigen::Vector3d> Av = detail::Vec(A.v);
  const Eigen::Map<const Eigen::Vector3d> Aa = detail::Vec(A.a);
  const Eigen::Map<const Eigen::Vector3d> Bp = detail::Vec(B.p);
  const Eigen::Map<const Eigen::Vector3d> Bv = detail::Vec(B.v);
  const Eigen::Map<const Eigen::Vector3d> Ba = detail::Vec(B.a);
  e.p = b.H0 * Ap + (b.H1 * h) * Av + (b.H2 * h * h) * Aa + b.H3 * Bp + (b.H4 * h) * Bv +
        (b.H5 * h * h) * Ba;
  e.v = (b.D0 * Ap + b.D3 * Bp) / h + b.D1 * Av + b.D4 * Bv + (b.D2 * h) * Aa + (b.D5 * h) * Ba;
  e.a = (b.S0 * Ap + b.S3 * Bp) / (h * h) + (b.S1 * Av + b.S4 * Bv) / h + b.S2 * Aa + b.S5 * Ba;
  e.valid = true;
  detail::MarkFinite(e);
  return e;
}

/// Taylor extrapolation from sample S to ball instant t. Meaningful only for a
/// short distance; the caller treats `after_horizon` as "do not catch here".
[[nodiscard]] inline SampleEval Extrapolate(const TrajSample& S, BallTime t, bool after) noexcept {
  const double d = SecondsBetween(BallTime{S.t_ns}, t);
  const Eigen::Map<const Eigen::Vector3d> p = detail::Vec(S.p);
  const Eigen::Map<const Eigen::Vector3d> v = detail::Vec(S.v);
  const Eigen::Map<const Eigen::Vector3d> a = detail::Vec(S.a);
  SampleEval e{};
  e.p = p + v * d + (0.5 * d * d) * a;
  e.v = v + a * d;
  e.a = a;
  e.before_horizon = !after;
  e.after_horizon = after;
  e.extrapolated = true;
  e.valid = true;
  detail::MarkFinite(e);
  return e;
}

/// Sample the trajectory at the tick's lead instant (plan §3: sampling is on the
/// lead axis). `hint` is the interval index of the previous call on this
/// snapshot — RT ticks move forward in time, so this is O(1) on average; a
/// stale hint recovers by binary search (O(log n)). Reset it to 0 when
/// `token.snapshot_sequence` changes (L2 §5.2) — correctness does not depend on
/// it, the tick cost does.
[[nodiscard]] inline SampleEval SampleAt(const TrajectorySnapshot& tr, NowLead now_lead,
                                         int& hint) noexcept {
  // Bound n BEFORE any indexing: n comes from the wire.
  if (!tr.valid || tr.n <= 0 || tr.n > kCap)
    return {};
  const BallTime t{now_lead.ns};  // the lead instant, read on the ball axis
  const auto at = [&tr](int i) noexcept -> const TrajSample& {
    return tr.s[static_cast<std::size_t>(i)];
  };
  const int n = tr.n;
  if (n == 1)
    return Extrapolate(at(0), t, t.ns > at(0).t_ns);
  if (t.ns < at(0).t_ns)
    return Extrapolate(at(0), t, false);
  if (t.ns > at(n - 1).t_ns)
    return Extrapolate(at(n - 1), t, true);
  if (t.ns == at(n - 1).t_ns) {  // the horizon end itself is not extrapolation
    SampleEval e{};
    e.p = detail::Vec(at(n - 1).p);
    e.v = detail::Vec(at(n - 1).v);
    e.a = detail::Vec(at(n - 1).a);
    e.valid = true;
    detail::MarkFinite(e);
    return e;
  }

  int i = std::clamp(hint, 0, n - 2);
  if (!(at(i).t_ns <= t.ns && t.ns < at(i + 1).t_ns)) {
    if (i + 1 <= n - 2 && at(i + 1).t_ns <= t.ns && t.ns < at(i + 2).t_ns) {
      ++i;  // the common case: the next interval
    } else {
      int lo = 0;
      int hi = n - 1;  // binary search (resync); invariant at(lo) <= t < at(hi)
      while (hi - lo > 1) {
        const int mid = lo + (hi - lo) / 2;
        if (at(mid).t_ns <= t.ns)
          lo = mid;
        else
          hi = mid;
      }
      i = lo;
    }
  }
  hint = i;
  return Interpolate(at(i), at(i + 1), t);
}

[[nodiscard]] inline SampleEval SampleAt(const TrajectorySnapshot& tr, NowLead now_lead) noexcept {
  int hint = 0;
  return SampleAt(tr, now_lead, hint);
}

/// Acceptance limits for Check(). n_max is the run-time bound (S3.6) and must
/// not exceed the compile-time capacity; dt_min is the smallest accepted sample
/// spacing (prediction.dt_min).
struct TrajLimits {
  int n_min{2};
  int n_max{kCap};
  std::int64_t dt_min_ns{0};
};

enum class TrajReject : std::uint8_t {
  kNone = 0,
  kLimitsInvalid,  // n_min/n_max/dt_min themselves are unusable
  kCount,          // n outside [n_min, n_max]
  kNonFinite,      // a NaN/Inf position, velocity or acceleration
  kNonMonotonic,   // sample times not strictly increasing
  kSpacing,        // an interval shorter than dt_min
};

struct TrajCheck {
  bool ok{false};
  TrajReject reason{TrajReject::kLimitsInvalid};  // first failure found
  bool count_ok{false};
  bool finite{false};
  bool t_monotonic{false};
  bool spacing_ok{false};
  std::int64_t dt_min_ns{0};   // smallest observed interval
  std::int64_t dt_max_ns{0};   // largest observed interval
  std::int64_t horizon_ns{0};  // last − first sample instant
};

/// Format check of a received snapshot (once on receipt, non-RT — but
/// allocation-free and noexcept so it can run anywhere). Rejects, in order:
/// unusable limits, n outside [n_min, n_max] (checked before any indexing),
/// non-finite values, non-increasing times, intervals below dt_min. A spacing
/// below dt_min is a rejection, not a warning: Interpolate divides by h².
[[nodiscard]] inline TrajCheck Check(const TrajectorySnapshot& tr, const TrajLimits& lim) noexcept {
  TrajCheck c{};
  if (lim.n_min < 1 || lim.n_max < lim.n_min || lim.n_max > kCap || lim.dt_min_ns <= 0)
    return c;  // reason = kLimitsInvalid

  c.count_ok = (tr.n >= lim.n_min && tr.n <= lim.n_max);
  if (!c.count_ok) {
    c.reason = TrajReject::kCount;
    return c;
  }

  c.finite = true;
  c.t_monotonic = true;
  c.spacing_ok = true;
  c.dt_min_ns = std::numeric_limits<std::int64_t>::max();
  c.dt_max_ns = 0;
  const std::size_t n = static_cast<std::size_t>(tr.n);
  for (std::size_t i = 0; i < n; ++i) {
    const TrajSample& s = tr.s[i];
    if (!detail::Vec(s.p).allFinite() || !detail::Vec(s.v).allFinite() ||
        !detail::Vec(s.a).allFinite())
      c.finite = false;
    if (i > 0) {
      // Saturating: wire timestamps are unchecked here, and an overflowed
      // difference would be UB inside the gate meant to reject them.
      const std::int64_t d = detail::SatSub(s.t_ns, tr.s[i - 1].t_ns);
      if (d <= 0)
        c.t_monotonic = false;
      else if (d < lim.dt_min_ns)
        c.spacing_ok = false;
      c.dt_min_ns = std::min(c.dt_min_ns, d);
      c.dt_max_ns = std::max(c.dt_max_ns, d);
    }
  }
  if (n == 1)
    c.dt_min_ns = 0;
  c.horizon_ns = detail::SatSub(tr.s[n - 1].t_ns, tr.s[0].t_ns);

  if (!c.finite)
    c.reason = TrajReject::kNonFinite;
  else if (!c.t_monotonic)
    c.reason = TrajReject::kNonMonotonic;
  else if (!c.spacing_ok)
    c.reason = TrajReject::kSpacing;
  else
    c.reason = TrajReject::kNone;
  c.ok = (c.reason == TrajReject::kNone);
  return c;
}

}  // namespace rtc::catching
