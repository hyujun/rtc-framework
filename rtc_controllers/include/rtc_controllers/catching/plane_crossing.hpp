// ── When the ball crosses a plane of the hand's catch frame ──────────────────
//
// The hand is commanded to close a fixed lead before the ball reaches a plane
// that moves with the hand: the plane through a point s_plane along the catch
// frame's z axis, normal to that axis. With the hand on a planned trajectory
// and the ball on a predicted one, the crossing instant is the root of
//
//   g(t) = e_3ᵀ R(t)ᵀ ( p_b(t) − p_h(t) ) − s_plane
//
// where (R, p_h) is the catch frame's pose and p_b the ball's position, all in
// one world frame. The root is found by the secant method from a starting
// guess — the instant the plan was made for — because ġ needs the hand's
// Jacobian and the two trajectories are only given by evaluation.
//
// The caller supplies g as a callable, so this file knows neither a trajectory
// payload nor a model. RT-safe: no allocation, no lock, no log, no throw; the
// number of evaluations is bounded by PlaneCrossingParams::max_iterations + 1.
#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <limits>

namespace rtc::catching {

/// @brief g at one instant from the two poses (file header): the ball's
///        coordinate along the catch frame's z axis, less the plane's offset.
/// @param R catch-frame orientation in the world frame
/// @param p_h catch-frame origin in the world frame [m]
/// @param p_b ball position in the world frame [m]
/// @param s_plane the plane's offset along the catch frame's z axis [m]
[[nodiscard]] inline double PlaneGap(const Eigen::Matrix3d& R, const Eigen::Vector3d& p_h,
                                     const Eigen::Vector3d& p_b, double s_plane) noexcept {
  return R.col(2).dot(p_b - p_h) - s_plane;
}

struct PlaneCrossingParams {
  /// Half-width of the accepted interval around the starting guess [ns]. A
  /// root — or an iterate — outside it is refused: the guess is the instant
  /// the hand's trajectory was planned for, and a crossing far from it is not
  /// the one that trajectory was made to meet.
  std::int64_t window_ns{0};
  /// The iteration has converged when a step is no longer than this [ns]. Also
  /// the first step away from the starting guess.
  std::int64_t tol_ns{0};
  /// Secant steps at most; the solve evaluates g at most this many times + 1.
  int max_iterations{8};
};

struct PlaneCrossing {
  bool valid{false};
  /// The crossing instant [ns]; meaningful only when `valid`.
  std::int64_t t_ns{0};
  /// Secant steps taken.
  int iterations{0};
};

/// @brief The root of g nearest the starting guess, by the secant method.
/// @param gap callable `bool(std::int64_t t_ns, double& g)`: g at `t_ns`, false
///        when it cannot be evaluated there (outside either trajectory)
/// @param t_start_ns the starting guess [ns]
/// @return invalid when g cannot be evaluated at an iterate, is not finite, has
///         no slope between two iterates, an iterate leaves the window, or the
///         steps do not shrink to `tol_ns` within `max_iterations`.
template <typename Gap>
[[nodiscard]] PlaneCrossing SolvePlaneCrossing(Gap&& gap, std::int64_t t_start_ns,
                                               const PlaneCrossingParams& params) noexcept {
  PlaneCrossing out{};
  if (!(params.tol_ns > 0) || !(params.window_ns > 0) || params.max_iterations < 1) {
    return out;
  }
  // Offsets from the starting guess, in ns as double: differences of instants
  // of a few hundred ms are exact there, the instants themselves are not.
  const double window = static_cast<double>(params.window_ns);
  const double tol = static_cast<double>(params.tol_ns);
  const auto eval = [&gap, t_start_ns](double x, double& g) noexcept {
    return gap(t_start_ns + static_cast<std::int64_t>(std::llround(x)), g) && std::isfinite(g);
  };
  double x0 = 0.0;
  double g0 = 0.0;
  if (!eval(x0, g0)) {
    return out;
  }
  double x1 = tol;
  for (int k = 1; k <= params.max_iterations; ++k) {
    double g1 = 0.0;
    if (!eval(x1, g1)) {
      return out;
    }
    const double dg = g1 - g0;
    // No slope between the two iterates: the ball does not close on the plane
    // there (or both sit on it, in which case the earlier step already ended).
    if (!(std::fabs(dg) > std::numeric_limits<double>::min())) {
      return out;
    }
    const double x2 = x1 - g1 * (x1 - x0) / dg;
    if (!std::isfinite(x2) || std::fabs(x2) > window) {
      return out;
    }
    out.iterations = k;
    if (std::fabs(x2 - x1) <= tol) {
      out.valid = true;
      out.t_ns = t_start_ns + static_cast<std::int64_t>(std::llround(x2));
      return out;
    }
    x0 = x1;
    g0 = g1;
    x1 = x2;
  }
  return out;
}

}  // namespace rtc::catching
