// ── Hand-joint capture evidence (#537 S8-C, D-S8-8 (b)) ─────────────────────
// The attempt verdict (L7 §4.4) reads the fingertips: m_min of them agreeing
// on contact. A ball that came to rest on the finger links or the palm makes
// no fingertip contact and reads Missed although it is in the hand — in sim,
// 43–57 % of the truth successes (plan D-S8-8). This core is the second
// witness: a hand holding a ball has fingers that STOPPED on it.
//
// PER JOINT, every clause on the same joint i of the caging set C:
//
//   ρ_i       = (q_i − q_pre,i)·s_i / |q_close,i − q_pre,i|,   s_i = sign(q_close,i − q_pre,i)
//   stalled_i = rho_min ≤ ρ_i ≤ rho_max           — stopped part-way (the empty
//                                                   hand reaches ρ = 1; a finger
//                                                   that never left q_pre is ~0)
//             ∧ |q̇_i| ≤ qd_tol                    — stopped, not passing through
//             ∧ s_i·τ_i / τ_max,i ≥ effort_frac_min — and still pushing toward
//                                                   q_close (a finger at rest in
//                                                   free air holds ~0)
//   blocked   = #{i ∈ C : stalled_i} ≥ min_joints
//
// Per joint rather than the sequencer's min-ρ plus a pooled torque: the joints
// that touch the ball are few, and a pooled statistic is carried by the ones
// that do not. The torque is SIGNED so that a finger pressed open by something
// is not read as holding it.
//
// THE HOLD RULE IT RELIES ON. Only `hold.mode: close_target`: Hold then
// commands q_close, so an empty hand ends at ρ = 1. Under measured_offset an
// empty finger settles at its η crossing plus delta_rad, inside any useful
// band — the config refuses the combination.
//
// WHAT IS NOT HERE. Whether the hand is in Hold, whether the lanes are fresh
// (a caller that cannot vouch for q, q̇ or τ passes an empty span — the answer
// is then "not blocked"), and how long `blocked` has to last: the controller
// owns all three. `blocked` can only promote Missed to Captured; it never
// overrides an Undetermined lane.
//
// Pure numeric core: fixed-size, no allocation, noexcept, no ROS. A non-finite
// reading never makes a joint stalled.
#pragma once

#include "rtc_controllers/catching/catching_params.hpp"  // HandProfile, kMaxHandDof

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>

namespace rtc::catching {

/// Everything the evaluation reads, resolved at configure. `Valid()` is this
/// core's own fail-closed check: an invalid config answers "not blocked".
struct HandCaptureConfig {
  std::array<double, kMaxHandDof> q_pre{};
  std::array<double, kMaxHandDof> q_close{};
  std::array<double, kMaxHandDof> tau_max{};
  std::array<bool, kMaxHandDof> caging_mask{};
  int dof{0};
  double rho_min{std::numeric_limits<double>::quiet_NaN()};
  double rho_max{std::numeric_limits<double>::quiet_NaN()};
  double effort_frac_min{std::numeric_limits<double>::quiet_NaN()};
  double qd_tol{std::numeric_limits<double>::quiet_NaN()};
  int min_joints{1};

  [[nodiscard]] bool Valid() const noexcept {
    if (dof <= 0 || dof > static_cast<int>(kMaxHandDof) || min_joints < 1) {
      return false;
    }
    // Written as "within" so a NaN fails each.
    if (!(rho_min >= 0.0) || !(rho_max > rho_min) || !(rho_max < 1.0)) {
      return false;
    }
    if (!(effort_frac_min > 0.0) || !(effort_frac_min <= 1.0) || !(qd_tol > 0.0)) {
      return false;
    }
    int caging = 0;
    for (int i = 0; i < dof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      if (!caging_mask[u]) {
        continue;
      }
      // ρ_i divides by the gap and the torque by τ_max: both must be usable
      // on every joint that can vote.
      if (!std::isfinite(q_pre[u]) || !std::isfinite(q_close[u]) ||
          !(std::abs(q_close[u] - q_pre[u]) > 0.0) || !(tau_max[u] > 0.0) ||
          !std::isfinite(tau_max[u])) {
        return false;
      }
      ++caging;
    }
    return min_joints <= caging;
  }

  /// The profile's poses and thresholds plus the hand device's per-joint
  /// torque limit (`joint_limits.max_torque`). A disabled or unresolved
  /// capture block, or a limit list shorter than `dof`, yields a config that
  /// `Valid()` refuses rather than a guessed number.
  [[nodiscard]] static HandCaptureConfig FromProfile(const HandProfile& p,
                                                     std::span<const double> tau_max) noexcept {
    HandCaptureConfig c{};
    const HandCaptureParams& cap = p.capture;
    // close_target only: the witness assumes Hold commands q_close, so an empty
    // hand reaches ρ = 1 (the validator refuses the other mode too).
    if (!cap.enabled || p.tbd || cap.rho_min.tbd || cap.rho_max.tbd || cap.effort_frac_min.tbd ||
        p.hold_mode != HandHoldMode::kCloseTarget ||
        tau_max.size() < static_cast<std::size_t>(p.dof)) {
      return c;  // dof 0: invalid
    }
    c.q_pre = p.q_pre;
    c.q_close = p.q_close;
    c.caging_mask = p.caging_mask;
    c.dof = p.dof;
    for (std::size_t i = 0; i < static_cast<std::size_t>(p.dof); ++i) {
      c.tau_max[i] = tau_max[i];
    }
    c.rho_min = cap.rho_min.value;
    c.rho_max = cap.rho_max.value;
    c.effort_frac_min = cap.effort_frac_min.value;
    c.qd_tol = p.qd_tol;
    c.min_joints = cap.min_joints;
    return c;
  }
};

/// One evaluation. `effort_frac_max` is the largest s_i·τ_i/τ_max,i over the
/// caging joints with a finite reading (NaN when there is none) — recorded so
/// the threshold can be re-derived offline from what the controller saw.
struct HandCaptureReading {
  int stalled{0};
  double effort_frac_max{std::numeric_limits<double>::quiet_NaN()};
  bool blocked{false};
};

/// `q` / `qd` / `tau` are the hand's measured positions, velocities and
/// efforts (device order, at least `dof` wide — a narrower span is "not
/// readable" and makes nothing stalled).
[[nodiscard]] inline HandCaptureReading EvaluateHandCapture(const HandCaptureConfig& cfg,
                                                            std::span<const double> q,
                                                            std::span<const double> qd,
                                                            std::span<const double> tau) noexcept {
  HandCaptureReading out{};
  const auto n = static_cast<std::size_t>(cfg.dof);
  if (!cfg.Valid() || q.size() < n || qd.size() < n || tau.size() < n) {
    return out;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!cfg.caging_mask[i]) {
      continue;
    }
    const double span = cfg.q_close[i] - cfg.q_pre[i];
    const double s = span > 0.0 ? 1.0 : -1.0;
    const double frac = s * tau[i] / cfg.tau_max[i];
    if (std::isfinite(frac)) {
      out.effort_frac_max =
          std::isfinite(out.effort_frac_max) ? std::max(out.effort_frac_max, frac) : frac;
    }
    const double rho = JointClosureProgress(q[i], cfg.q_pre[i], cfg.q_close[i]);
    // Each clause written as "within" so a NaN fails it.
    // `frac` also needs isfinite: an infinite torque passes ">=".
    const bool stalled = rho >= cfg.rho_min && rho <= cfg.rho_max &&
                         std::abs(qd[i]) <= cfg.qd_tol && std::isfinite(frac) &&
                         frac >= cfg.effort_frac_min;
    out.stalled += stalled ? 1 : 0;
  }
  out.blocked = out.stalled >= cfg.min_joints;
  return out;
}

}  // namespace rtc::catching
