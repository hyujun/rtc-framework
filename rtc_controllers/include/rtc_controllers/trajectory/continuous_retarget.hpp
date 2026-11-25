#ifndef RTC_CONTROLLERS_TRAJECTORY_CONTINUOUS_RETARGET_HPP_
#define RTC_CONTROLLERS_TRAJECTORY_CONTINUOUS_RETARGET_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

namespace rtc {
namespace trajectory {

/// What PlanContinuousRetarget decided, joint by joint.
struct ContinuousRetargetPlan {
  /// Shared duration of every joint's quintic [s]; >= the base duration.
  double duration{0.0};
  /// Joints that start with a non-zero velocity.
  int kept{0};
  /// Of those, joints whose velocity was lowered so they do not pass their goal.
  int reduced{0};
  /// Joints moving at the call whose start velocity was set to 0: the quintic
  /// would have left their position limits (or the velocity was not finite).
  int dropped{0};
};

namespace continuous_retarget_detail {

/// Relative size under which a coefficient counts as zero (a degenerate
/// extremum / quadratic), against the magnitudes it is built from.
inline constexpr double kDegenerateRel = 1e-12;
/// |v0|·T <= kNoOvershootRatio·|Δ| is exactly when a joint moving towards its
/// goal does not pass it (Q(1) = 30A - 12b >= 0 in StaysWithin's notation).
inline constexpr double kNoOvershootRatio = 2.5;

/// Velocity of q0 + Δ·h0(τ) + v0·T·h1(τ) at τ, with A = Δ/T and b = v0:
/// v = A·h0'(τ) + b·h1'(τ), h0' = 30τ²(1-τ)², h1' = (1-τ)²(1+2τ-15τ²).
inline double Velocity(double a, double b, double tau) noexcept {
  const double s = 1.0 - tau;
  return s * s * (30.0 * a * tau * tau + b * (1.0 + 2.0 * tau - 15.0 * tau * tau));
}

/// Position offset from q0 at τ: Δ·h0(τ) + v0·T·h1(τ),
/// h0 = 10τ³ - 15τ⁴ + 6τ⁵, h1 = τ(1-τ)³(1+3τ).
inline double Offset(double delta, double v0, double duration, double tau) noexcept {
  const double t2 = tau * tau;
  const double t3 = t2 * tau;
  const double s = 1.0 - tau;
  const double h0 = t3 * (10.0 - 15.0 * tau + 6.0 * t2);
  const double h1 = tau * s * s * s * (1.0 + 3.0 * tau);
  return delta * h0 + v0 * duration * h1;
}

/// Exact peak |v| over τ ∈ [0, 1]. dv/dτ = τ(1-τ)·[60A(1-2τ) + 12b(5τ-3)]: the
/// bracket is linear, so the one interior extremum is τ* = (5A-3b)/(10A-5b);
/// the ends give |b| and 0.
inline double PeakSpeed(double a, double b) noexcept {
  double peak = std::abs(b);
  const double den = 10.0 * a - 5.0 * b;
  // den = 0 makes the bracket the constant -6b: no interior extremum.
  if (std::abs(den) > kDegenerateRel * (std::abs(a) + std::abs(b))) {
    const double tau = (5.0 * a - 3.0 * b) / den;
    if (tau > 0.0 && tau < 1.0) {
      peak = std::max(peak, std::abs(Velocity(a, b, tau)));
    }
  }
  return peak;
}

/// Does q0 + offset stay within [lo - eps, hi + eps] for τ ∈ [0, 1]? The ends
/// are q0 and the goal (the caller's, both inside); interior extrema are the
/// roots of Q(τ) = (30A-15b)τ² + 2bτ + b, from v = (1-τ)²·Q(τ).
inline bool StaysWithin(double q0, double delta, double v0, double duration, double lo, double hi,
                        double eps) noexcept {
  const double a = delta / duration;
  const double qa = 30.0 * a - 15.0 * v0;
  const double qb = 2.0 * v0;
  const double qc = v0;
  std::array<double, 2> roots = {-1.0, -1.0};
  if (std::abs(qa) > kDegenerateRel * (std::abs(a) + std::abs(v0))) {
    const double disc = qb * qb - 4.0 * qa * qc;
    if (disc >= 0.0) {
      // Numerically stable pair: q = -(b + sign(b)·√disc) / 2.
      const double q = -0.5 * (qb + std::copysign(std::sqrt(disc), qb));
      roots[0] = q / qa;
      if (q != 0.0) {
        roots[1] = qc / q;
      }
    }
  }
  // Linear Q (qa = 0): 2bτ + b = 0 at τ = -1/2, outside [0, 1].
  for (const double tau : roots) {
    if (tau > 0.0 && tau < 1.0) {
      const double q = q0 + Offset(delta, v0, duration, tau);
      if (!(q >= lo - eps && q <= hi + eps)) {
        return false;
      }
    }
  }
  return true;
}

/// Largest same-direction start speed that never passes the goal over a
/// duration T: |v0|·T <= 2.5·|Δ| (Q(1) = 30A - 12b >= 0). Δ = 0 allows none.
inline double SameDirectionBound(double delta, double duration) noexcept {
  return kNoOvershootRatio * std::abs(delta) / duration;
}

/// The start velocity a joint gets over `duration`: reversals keep theirs, a
/// joint moving towards its goal is held under the no-overshoot bound.
inline double EffectiveVelocity(double delta, double v0, double duration) noexcept {
  if (v0 == 0.0 || v0 * delta < 0.0) {
    return v0;
  }
  const double bound = SameDirectionBound(delta, duration);
  return std::abs(v0) <= bound ? v0 : std::copysign(bound, v0);
}

/// Smallest duration on base · 1.25^k (k <= kMaxGrowthSteps) at which every
/// joint with a non-zero start velocity is within its cap; < 0 when none is.
/// Peak |v| never grows with T (it tends to |v0| <= cap), and the bound is
/// reached by ≈ 1.95·base, so 8 steps (5.96·base) always suffice.
inline constexpr int kMaxGrowthSteps = 8;

inline double SmallestCappedDuration(std::span<const double> q0, std::span<const double> v0,
                                     std::span<const double> goal, std::span<const double> cap,
                                     double base) noexcept {
  double duration = base;
  for (int k = 0; k <= kMaxGrowthSteps; ++k, duration *= 1.25) {
    bool ok = true;
    for (std::size_t i = 0; i < q0.size() && ok; ++i) {
      if (v0[i] == 0.0) {
        continue;  // rest-to-rest: the base duration already bounds its peak
      }
      const double delta = goal[i] - q0[i];
      const double b = EffectiveVelocity(delta, v0[i], duration);
      ok = PeakSpeed(delta / duration, b) <= cap[i];
    }
    if (ok) {
      return duration;
    }
  }
  return -1.0;
}

}  // namespace continuous_retarget_detail

/// Duration and start velocities for joint quintics that start MOVING, at
/// (q0_i, v0_i, acceleration 0), and end at rest at goal_i, all over one shared
/// duration — a retarget that keeps the velocity instead of stopping first.
///
/// `v0` is read and overwritten with the start velocity each joint is given:
/// - a joint moving towards its goal keeps |v0| <= 2.5·|Δ|/T, so it never
///   passes the goal and comes back (it is lowered, not zeroed, when faster);
/// - a joint reversing keeps its velocity and travels on past q0 before it
///   turns, unless that leaves [q_min, q_max] (+ `limit_eps`): then it starts
///   at 0 and moves monotonically, as a rest-to-rest quintic does;
/// - a non-finite velocity, or one under 1e-9·cap, starts at 0; one above the
///   cap is lowered to it.
///
/// The duration is the smallest of base · 1.25^k for which every joint's peak
/// |velocity| (exact, closed form) is within cap_i. `base` is the caller's
/// rest-to-rest duration (its own speed / cap / dt floor) and must be > 0;
/// with every v0 at 0 it is returned unchanged, so the caller's rest-to-rest
/// quintic is reproduced bit for bit. `cap` entries must be > 0, q0 and goal
/// inside the limits. Sizes must agree; if they do not, every v0 is zeroed and
/// `base` is returned. Allocation-free and bounded: O(dof · 9).
inline ContinuousRetargetPlan PlanContinuousRetarget(
    std::span<const double> q0, std::span<double> v0, std::span<const double> goal,
    std::span<const double> cap, std::span<const double> q_min, std::span<const double> q_max,
    double base, double limit_eps = 1e-9) noexcept {
  namespace d = continuous_retarget_detail;
  ContinuousRetargetPlan plan;
  plan.duration = base;
  const std::size_t n = q0.size();
  if (v0.size() != n || goal.size() != n || cap.size() != n || q_min.size() != n ||
      q_max.size() != n || !(base > 0.0) || !std::isfinite(base)) {
    std::fill(v0.begin(), v0.end(), 0.0);
    return plan;
  }
  int moving = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const bool was_moving = v0[i] != 0.0;
    moving += was_moving ? 1 : 0;
    if (!std::isfinite(v0[i]) || !(cap[i] > 0.0) || std::abs(v0[i]) < 1e-9 * cap[i]) {
      v0[i] = 0.0;
    } else if (std::abs(v0[i]) > cap[i]) {
      v0[i] = std::copysign(cap[i], v0[i]);
    }
  }
  const auto finish = [&](double duration) {
    plan.duration = duration;
    plan.kept = plan.reduced = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const double delta = goal[i] - q0[i];
      const double b = d::EffectiveVelocity(delta, v0[i], duration);
      plan.reduced += (b != v0[i] && b != 0.0) ? 1 : 0;
      v0[i] = b;
      plan.kept += (b != 0.0) ? 1 : 0;
    }
    plan.dropped = moving - plan.kept;
    return plan;
  };
  const auto any_moving = [&] {
    return std::any_of(v0.begin(), v0.end(), [](double v) { return v != 0.0; });
  };
  if (!any_moving()) {
    return finish(base);
  }
  // Pass 1: the cap. Pass 2: at that duration, a joint whose quintic leaves
  // its limits starts at rest (overshoot only grows with T, so a later,
  // shorter duration cannot bring it back). Pass 3: the cap again without
  // those joints — the duration can only shrink, and a shorter one moves the
  // remaining joints less far past q0.
  const double capped = d::SmallestCappedDuration(q0, v0, goal, cap, base);
  if (capped < 0.0) {
    std::fill(v0.begin(), v0.end(), 0.0);  // unreachable by the bound above
    return finish(base);
  }
  bool any_dropped = false;
  for (std::size_t i = 0; i < n; ++i) {
    if (v0[i] == 0.0) {
      continue;
    }
    const double delta = goal[i] - q0[i];
    const double b = d::EffectiveVelocity(delta, v0[i], capped);
    if (!d::StaysWithin(q0[i], delta, b, capped, q_min[i], q_max[i], limit_eps)) {
      v0[i] = 0.0;
      any_dropped = true;
    }
  }
  if (!any_dropped) {
    return finish(capped);
  }
  if (!any_moving()) {
    return finish(base);
  }
  const double shorter = d::SmallestCappedDuration(q0, v0, goal, cap, base);
  return finish(shorter > 0.0 ? std::min(shorter, capped) : capped);
}

}  // namespace trajectory
}  // namespace rtc

#endif  // RTC_CONTROLLERS_TRAJECTORY_CONTINUOUS_RETARGET_HPP_
