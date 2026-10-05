// ── Ball prediction at one planner node: mean and 6×6 covariance ───────────────
// (dynamic_catching E1-F13, #739)
//
// A segment planner's grid is anchored at the catch instant, so its nodes fall
// BETWEEN the samples of the vision prediction. This is the one place that
// turns "the prediction and its covariance" into "the ball at node instant t":
//
//   mean        SampleAt() — the quintic Hermite every other consumer of the
//               prediction uses (traj_sampler.hpp), so two planners looking at
//               the same instant see the same ball.
//   covariance  local propagation from the NEAREST sample i, Δ = t − t_i:
//
//                 Σ(t) ≈ F(Δ) Σ_i F(Δ)ᵀ,   F(Δ) = [ I  Δ·I ]
//                                                  [ 0   I  ]
//
//               the message's own interpolation rule (no process noise, no
//               re-propagation of the ball — the controller trusts the
//               prediction). Δ may be negative.
//
// The covariance is a 6×6 over [p; v] in the MODEL world, symmetric when
// cov_valid. It is reported invalid — never silently replaced by zero, which
// would read as certainty — when
//   • there is no covariance, it is not valid, or it belongs to another
//     prediction (cov_matched = false: the provenance tokens differ),
//   • the nearest sample has no covariance entry (cov.n too small),
//   • any of its 36 elements is non-finite (NaN is the wire's "not known"),
//   • a propagated variance is negative (F Σ Fᵀ preserves PSD, it does not
//     create it; a diagonal below zero is the cheap necessary check — a
//     consumer that takes the square root of some other quadratic form must
//     still check that form itself).
// The MEAN's validity is independent of the covariance's: a planner whose
// terms need no covariance at a node reads `valid` alone.
//
// RT-safe: fixed size, no heap, noexcept, no ROS. The nearest-sample search is
// a bounded scan (n ≤ kCap).
#pragma once

#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"  // CovarianceSnapshot
#include "rtc_controllers/catching/traj_sampler.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>

namespace rtc::catching {

using BallCovariance = Eigen::Matrix<double, 6, 6>;

/// The predicted ball at one node instant.
struct BallNodeSample {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  ///< [m] model world
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};  ///< [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};  ///< [m/s²]
  /// Covariance of [p; v] at the node instant ([m², m²/s; m²/s, m²/s²]),
  /// exactly symmetric. Zero unless cov_valid.
  BallCovariance cov{BallCovariance::Zero()};
  bool valid{false};          ///< p, v, a are usable
  bool after_horizon{false};  ///< the instant is past the prediction (extrapolated mean)
  bool cov_valid{false};      ///< cov is usable (header note)
};

/// @brief F(Δ) Σ F(Δ)ᵀ for the constant-velocity transition over Δ [s],
///        written out block by block (RT-safe).
/// @param sigma covariance of [p; v]; read as given (symmetrise first)
[[nodiscard]] inline BallCovariance PropagateBallCovariance(const BallCovariance& sigma,
                                                            double delta) noexcept {
  const Eigen::Matrix3d pp = sigma.topLeftCorner<3, 3>();
  const Eigen::Matrix3d pv = sigma.topRightCorner<3, 3>();
  const Eigen::Matrix3d vp = sigma.bottomLeftCorner<3, 3>();
  const Eigen::Matrix3d vv = sigma.bottomRightCorner<3, 3>();
  BallCovariance out;
  out.topLeftCorner<3, 3>() = pp + delta * (pv + vp) + (delta * delta) * vv;
  out.topRightCorner<3, 3>() = pv + delta * vv;
  out.bottomLeftCorner<3, 3>() = vp + delta * vv;
  out.bottomRightCorner<3, 3>() = vv;
  return out;
}

/// @brief The ball at instant `t`: mean by SampleAt, covariance by local
///        propagation from the nearest prediction sample (RT-safe).
/// @param traj        the prediction; `traj.n` is bounded before any indexing
/// @param cov         its covariance, or nullptr (then cov_valid = false)
/// @param cov_matched `cov` carries the same provenance token as `traj`
/// @param t           node instant on the ball axis
/// @param[in,out] hint SampleAt's interval hint (reset to 0 on a new snapshot)
/// @return the sample; `valid` false when the prediction cannot be sampled.
[[nodiscard]] inline BallNodeSample SampleBallNode(const TrajectorySnapshot& traj,
                                                   const CovarianceSnapshot* cov, bool cov_matched,
                                                   BallTime t, int& hint) noexcept {
  BallNodeSample out;
  const SampleEval mean = SampleAt(traj, NowLead{t.ns}, hint);
  if (!mean.valid) {
    return out;
  }
  out.p = mean.p;
  out.v = mean.v;
  out.a = mean.a;
  out.valid = true;
  out.after_horizon = mean.after_horizon;

  if (cov == nullptr || !cov->valid || !cov_matched) {
    return out;
  }
  // Nearest sample by |t − t_i|. SampleAt accepted the snapshot, so 1 ≤ n ≤ kCap.
  std::size_t nearest = 0;
  std::int64_t best = detail::SatSub(t.ns, traj.s[0].t_ns);
  best = best < 0 ? detail::SatSub(0, best) : best;
  for (std::size_t i = 1; i < static_cast<std::size_t>(traj.n); ++i) {
    std::int64_t d = detail::SatSub(t.ns, traj.s[i].t_ns);
    d = d < 0 ? detail::SatSub(0, d) : d;
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  if (cov->n <= 0 || nearest >= static_cast<std::size_t>(cov->n) ||
      nearest >= static_cast<std::size_t>(kCap)) {
    return out;
  }
  // Row-major 6×6, p then v. Finite BEFORE anything is formed from it.
  const auto& c = cov->c[nearest];
  BallCovariance raw;
  for (Eigen::Index r = 0; r < 6; ++r) {
    for (Eigen::Index q = 0; q < 6; ++q) {
      raw(r, q) = c[static_cast<std::size_t>(r * 6 + q)];
    }
  }
  if (!raw.allFinite()) {
    return out;
  }
  const BallCovariance sym = 0.5 * (raw + raw.transpose());
  const double delta = SecondsBetween(BallTime{traj.s[nearest].t_ns}, t);
  const BallCovariance moved = PropagateBallCovariance(sym, delta);
  if (!moved.allFinite()) {
    return out;
  }
  for (Eigen::Index i = 0; i < 6; ++i) {
    if (moved(i, i) < 0.0) {
      return out;
    }
  }
  // The block form is symmetric up to rounding of (pv + vp); make it exact.
  out.cov = 0.5 * (moved + moved.transpose());
  out.cov_valid = true;
  return out;
}

}  // namespace rtc::catching
