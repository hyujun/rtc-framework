// ── NLP catch search: the candidate lattice, the necessary conditions and the
//    outer cost, as pure functions ─────────────────────────────────────────────
// (dynamic_catching E1-F14, #740; reference
//  docs/dynamic_catching/ref/ball_catching_inverse_dynamics_mpc.md §8.4, §9.5,
//  §11.1, §11.3, §17.11)
//
// What NlpCatchSearch (nlp_catch_search.hpp) decides about a candidate catch
// instant BEFORE it spends a solve on it, and the two terms it adds to the
// solve's cost afterwards. Free functions of scalars and spans — no model, no
// solver, no state — so that each inequality is tested against the reference's
// formula on its own, and so that an offline map of "which throws does the
// search accept" calls exactly what the search calls.
//
// ── The lattice (§11.1) ───────────────────────────────────────────────────────
// Candidates sit on a lattice of ABSOLUTE instants, t_c(i) = t_ref + i·h, fixed
// for one catch attempt: a candidate keeps its index from wake to wake, which
// is what lets a wake start a candidate's solve from that candidate's previous
// solution. A wake evaluates the indices whose instant lies in
// [t_0 + T_min, t_0 + T_max], t_0 being the instant its result can reach the
// arm.
//
// ── A candidate's arm grid ────────────────────────────────────────────────────
// The arm problem's grid is anchored at the catch instant (the core's, see
// mpc_docking_segment_core.hpp): n_pre intervals of Δ_a before t_c. A candidate
// uses the most intervals that fit,
//
//   n_pre = ⌊(t_c − t_0) / Δ_a⌋,   t_s = t_c − n_pre·Δ_a ∈ [t_0, t_0 + Δ_a),
//
// and its node 0 is at t_s, not at t_0: there is no shorter first interval.
// All of it is integer nanoseconds — the same arithmetic on the same integers
// wherever it is repeated.
//
// ── The necessary conditions (§11.3) ──────────────────────────────────────────
// Each is NECESSARY: passing proves nothing, failing removes the candidate.
//   (S4) reach. From (q_0, v_0) the catch pose q^c of the IK cannot be reached
//        in T under |q̇| ≤ v_max unless |q^c − q_0| ≤ v_max·T, nor under
//        |q̈| ≤ a_max unless |q^c − q_0 − v_0·T| ≤ ½ a_max·T².
//   (S3) closing-speed window. The timing chance constraint wants the ball to
//        cross fast, the impact limits want it slow; with both bounds taken at
//        the IK pose the window max{c_min, c_t,lo} ≤ c ≤ min{c_cap,max, c_n,hi}
//        must not be empty.
//
// RT-safe: no heap, noexcept. A NaN anywhere FAILS the condition it enters
// (every comparison is written so that NaN is not "inside").
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace rtc::catching {

// ── Lattice ───────────────────────────────────────────────────────────────────

/// @brief ⌊a / b⌋ for b > 0, toward −∞ (the built-in `/` truncates toward 0).
[[nodiscard]] constexpr std::int64_t FloorDiv(std::int64_t a, std::int64_t b) noexcept {
  const std::int64_t q = a / b;
  return (a % b != 0 && a < 0) ? q - 1 : q;
}

/// @brief ⌈a / b⌉ for b > 0.
[[nodiscard]] constexpr std::int64_t CeilDiv(std::int64_t a, std::int64_t b) noexcept {
  const std::int64_t q = a / b;
  return (a % b != 0 && a > 0) ? q + 1 : q;
}

/// The lattice indices one wake evaluates: every i with
/// t_ref + i·h ∈ [window_lo, window_hi]. Empty when `last < first`.
struct NlpCandidateRange {
  std::int64_t first{0};
  std::int64_t last{-1};

  [[nodiscard]] constexpr std::int64_t Count() const noexcept {
    return last >= first ? last - first + 1 : 0;
  }
};

/// @brief The indices of the lattice instants inside [window_lo, window_hi].
/// @param t_ref_ns the lattice anchor (fixed for the catch attempt)
/// @param h_ns the lattice spacing, > 0
/// @return an empty range when h_ns ≤ 0 or the window is empty.
[[nodiscard]] constexpr NlpCandidateRange NlpCandidatesInWindow(
    std::int64_t t_ref_ns, std::int64_t h_ns, std::int64_t window_lo_ns,
    std::int64_t window_hi_ns) noexcept {
  if (h_ns <= 0 || window_hi_ns < window_lo_ns) {
    return {};
  }
  return {CeilDiv(window_lo_ns - t_ref_ns, h_ns), FloorDiv(window_hi_ns - t_ref_ns, h_ns)};
}

/// @brief The instant of lattice index i.
[[nodiscard]] constexpr std::int64_t NlpCandidateInstant(std::int64_t t_ref_ns, std::int64_t h_ns,
                                                         std::int64_t index) noexcept {
  return t_ref_ns + index * h_ns;
}

/// A candidate's arm grid: how many pre-catch intervals, and where node 0 is.
struct NlpCandidateGrid {
  int n_pre{0};             ///< ⌊(t_c − t_0)/Δ_a⌋; 0 when the candidate is not ahead of t_0
  std::int64_t t_s_ns{0};   ///< node 0's instant, t_c − n_pre·Δ_a
  std::int64_t wait_ns{0};  ///< t_s − t_0 ∈ [0, Δ_a): the arm keeps its previous motion
};

/// @brief The arm grid of the candidate at `t_c_ns` for a result that reaches
///        the arm at `t_0_ns`.
/// @param dt_pre_ns Δ_a, > 0
/// @return n_pre = 0 (no grid) when t_c ≤ t_0 or Δ_a ≤ 0.
[[nodiscard]] constexpr NlpCandidateGrid NlpCandidateGridAt(std::int64_t t_c_ns,
                                                            std::int64_t t_0_ns,
                                                            std::int64_t dt_pre_ns) noexcept {
  if (dt_pre_ns <= 0 || t_c_ns <= t_0_ns) {
    return {0, t_c_ns, 0};
  }
  const std::int64_t n = (t_c_ns - t_0_ns) / dt_pre_ns;
  const std::int64_t t_s = t_c_ns - n * dt_pre_ns;
  return {static_cast<int>(n), t_s, t_s - t_0_ns};
}

// ── (S4) Reach ────────────────────────────────────────────────────────────────

/// Which of the two reach conditions a joint failed.
enum class NlpReachLimit : std::uint8_t {
  kNone = 0,      ///< every joint passes both
  kVelocity,      ///< |q^c − q_0| > v_max·T
  kAcceleration,  ///< |q^c − q_0 − v_0·T| > ½ a_max·T²
  kInput,         ///< spans of different sizes, or T not positive
};

struct NlpReachVerdict {
  NlpReachLimit limit{NlpReachLimit::kNone};
  int joint{-1};  ///< the first joint that failed, in the spans' order; −1 when none

  [[nodiscard]] constexpr bool Reachable() const noexcept { return limit == NlpReachLimit::kNone; }
};

/// @brief The reference's reach conditions (§11.3 S4), joint by joint.
/// @param q_c the catch pose [rad]
/// @param q_0 where the motion starts [rad]
/// @param v_0 the velocity it starts with [rad/s]
/// @param v_max |q̇| limits [rad/s]
/// @param a_max |q̈| limits [rad/s²]; an EMPTY span switches the second
///        condition off (no acceleration limit is in force)
/// @param duration_s T: the time the motion has [s]
/// @return the first joint that fails — the velocity condition is tested
///         before the acceleration one on each joint. A NaN fails.
[[nodiscard]] inline NlpReachVerdict NlpReachCheck(
    std::span<const double> q_c, std::span<const double> q_0, std::span<const double> v_0,
    std::span<const double> v_max, std::span<const double> a_max, double duration_s) noexcept {
  const std::size_t n = q_c.size();
  if (q_0.size() != n || v_0.size() != n || v_max.size() != n ||
      (!a_max.empty() && a_max.size() != n) || !(duration_s > 0.0)) {
    return {NlpReachLimit::kInput, -1};
  }
  const double half_t_sq = 0.5 * duration_s * duration_s;
  for (std::size_t j = 0; j < n; ++j) {
    const double dq = q_c[j] - q_0[j];
    if (!(std::fabs(dq) <= v_max[j] * duration_s)) {
      return {NlpReachLimit::kVelocity, static_cast<int>(j)};
    }
    if (!a_max.empty() && !(std::fabs(dq - v_0[j] * duration_s) <= a_max[j] * half_t_sq)) {
      return {NlpReachLimit::kAcceleration, static_cast<int>(j)};
    }
  }
  return {};
}

// ── (S3) Closing-speed window ─────────────────────────────────────────────────

/// What the closing-speed window is built from, all evaluated at the IK pose.
struct NlpSpeedWindowInput {
  double c_min{0.0};      ///< [m/s] lower edge of the terminal velocity set
  double c_cap_max{0.0};  ///< [m/s] its upper edge
  /// The timing chance row is in force. Then c ≥ σ_s / √(σ_max² − σ_τ²).
  bool timing{false};
  double sigma_s{0.0};    ///< [m] ball position std along the approach axis (ε_σ included)
  double sigma_max{0.0};  ///< [s] largest total timing std the closure window allows
  double sigma_tau{0.0};  ///< [s] closure latency jitter
  /// The impact rows are in force. Then c ≤ √(2E_max/m_red) and
  /// c ≤ P_max/((1 + e)·m_red); an infinite threshold is a row that is off.
  bool impact{false};
  double m_red{0.0};        ///< [kg] reduced mass at the contact
  double e_max{0.0};        ///< [J]
  double p_max{0.0};        ///< [N·s]
  double restitution{0.0};  ///< e
};

struct NlpSpeedWindow {
  double lo{0.0};  ///< max{c_min, c_t,lo} [m/s]
  double hi{0.0};  ///< min{c_cap,max, c_n,hi} [m/s]
  /// The timing term's own lower bound and the impact term's own upper bound
  /// (0 and +inf when the term is off) — for the record.
  double c_t_lo{0.0};
  double c_n_hi{std::numeric_limits<double>::infinity()};
  bool empty{true};  ///< lo > hi, or a term could not be evaluated
};

/// @brief The closing-speed window of the reference's §8.4 (RT-safe).
/// @return `empty` true when lo > hi, and also when a term that is in force
///         cannot be evaluated: σ_max ≤ σ_τ (the latency jitter alone already
///         breaks the timing requirement), m_red not positive, or a NaN.
[[nodiscard]] inline NlpSpeedWindow NlpClosingSpeedWindow(const NlpSpeedWindowInput& in) noexcept {
  NlpSpeedWindow w;
  w.lo = in.c_min;
  w.hi = in.c_cap_max;
  // Before any fmax/fmin: those return the OTHER operand for a NaN, which
  // would let a NaN edge through as if a row had replaced it.
  if (std::isnan(in.c_min) || std::isnan(in.c_cap_max)) {
    return w;  // empty
  }
  if (in.timing) {
    const double k_sq = in.sigma_max * in.sigma_max - in.sigma_tau * in.sigma_tau;
    if (!(k_sq > 0.0) || !(in.sigma_s >= 0.0)) {
      return w;  // empty
    }
    w.c_t_lo = in.sigma_s / std::sqrt(k_sq);
    w.lo = std::fmax(w.lo, w.c_t_lo);
  }
  if (in.impact) {
    if (!(in.m_red > 0.0) || !(in.restitution >= 0.0)) {
      return w;  // empty
    }
    // +inf thresholds give +inf bounds: the row is off.
    const double by_energy = std::sqrt(2.0 * in.e_max / in.m_red);
    const double by_impulse = in.p_max / ((1.0 + in.restitution) * in.m_red);
    if (std::isnan(by_energy) || std::isnan(by_impulse)) {
      return w;  // empty
    }
    w.c_n_hi = std::fmin(by_energy, by_impulse);
    w.hi = std::fmin(w.hi, w.c_n_hi);
  }
  w.empty = !(w.lo <= w.hi);
  return w;
}

// ── The outer cost (§9.5) ─────────────────────────────────────────────────────

/// @brief J_time = w_T · T / T_ref: a mild preference for the earlier catch.
/// @param t_ref_s T_ref > 0 [s] (the caller validates it once)
[[nodiscard]] constexpr double NlpTimeCost(double w_time, double duration_s,
                                           double t_ref_s) noexcept {
  return w_time * duration_s / t_ref_s;
}

/// @brief J_switch = w_sw · ((t_c − t_c,prev) / T_ref)²: the price of moving
///        the ABSOLUTE catch instant away from the one in force.
/// @param has_previous false: there is no catch instant in force, the term is 0
[[nodiscard]] constexpr double NlpSwitchCost(double w_switch, std::int64_t t_c_ns,
                                             bool has_previous, std::int64_t t_c_prev_ns,
                                             double t_ref_s) noexcept {
  if (!has_previous) {
    return 0.0;
  }
  const double d = static_cast<double>(t_c_ns - t_c_prev_ns) * 1e-9 / t_ref_s;
  return w_switch * d * d;
}

// ── The start state and the core's box ────────────────────────────────────────

/// @brief Project a start state into the box a solve requires of it:
///        q ∈ [lo, hi], |q̇| ≤ v_hi, joint by joint (RT-safe).
///
/// A solve refuses a start outside its box, and a segment another planner
/// published may be held to a box with a different margin — so a state read
/// off such a segment is projected, and the projection is reported rather than
/// hidden. The accelerations are not touched (the solve does not check them).
/// @return true when any entry was moved. A NaN entry is left as it is (the
///         caller's finiteness check is separate).
[[nodiscard]] inline bool ProjectStartIntoBox(std::span<double> q, std::span<double> qd,
                                              std::span<const double> q_lo,
                                              std::span<const double> q_hi,
                                              std::span<const double> v_hi) noexcept {
  bool moved = false;
  const std::size_t n = q.size();
  if (qd.size() != n || q_lo.size() != n || q_hi.size() != n || v_hi.size() != n) {
    return false;
  }
  for (std::size_t j = 0; j < n; ++j) {
    if (q[j] < q_lo[j]) {
      q[j] = q_lo[j];
      moved = true;
    } else if (q[j] > q_hi[j]) {
      q[j] = q_hi[j];
      moved = true;
    }
    if (qd[j] < -v_hi[j]) {
      qd[j] = -v_hi[j];
      moved = true;
    } else if (qd[j] > v_hi[j]) {
      qd[j] = v_hi[j];
      moved = true;
    }
  }
  return moved;
}

}  // namespace rtc::catching
