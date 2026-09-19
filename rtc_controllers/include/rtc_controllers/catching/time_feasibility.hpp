// ── Reachability gates of the catch planner (dynamic_catching S1.5, L3) ──────
// Pure functions the planner (S6) and the catchability map tool (S3.5) share:
//
//   TMinChecked / MaxJointTMin — per-joint minimum time (q_c, q̇_c) → (q*, 0)
//                                under box velocity / acceleration limits (§4.3)
//   ReachTimeFeasible          — t_k − now_lead − T_margin ≥ max_i t_min,i
//   ComputeGammaWindow         — γ window [γ_min, γ_max] (§4.5), D-9 TCP speed
//   DirectionalSpeedMax        — v_dir,max from a DLS unit-direction solution
//   MaxCatchableSpeed          — ball speed above which the γ window is empty
//   StoppingPoint              — stopping-distance reservation (§4.9)
//   CatchErrorSigma            — catch error budget σ_gap (§4.6)
//
// Ported from docs/dynamic_catching/time_feasibility.hpp with the reference's
// silent failures made explicit (fail-closed, NUM-2/NUM-4):
//  • invalid limits (a_max ≤ 0, w_max ≤ 0, NaN) used to return t = 0 with no
//    flag — a reach-time gate that always passed. Now `limits_invalid` is set
//    and t = +∞, so even a caller that ignores the flag rejects the candidate;
//  • a zero ball speed used to be floored at 1e-6 and reported a feasible
//    window; the window of a non-moving ball is undefined → input_invalid;
//  • v_dir,max uses the projection v̂ᵀJ_p q̇ᵘ (not ‖J_p q̇ᵘ‖, not 1), with
//    zero / NaN guards on the joint limits (L3 §4.5 v0.5).
//
// All functions are allocation-free and noexcept (the planner may run
// SCHED_FIFO, plan §7.2).
#pragma once

#include "rtc_controllers/catching/time_types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace rtc::catching {

inline constexpr double kInfTime = std::numeric_limits<double>::infinity();

/// Rest-to-rest minimum time over distance D ≥ 0. Limits must be valid
/// (positive, finite) — callers go through TMinChecked().
[[nodiscard]] inline double TRest(double D, double w_max, double a_max) noexcept {
  if (D <= 0.0)
    return 0.0;
  const double w_peak = std::sqrt(a_max * D);
  return (w_peak <= w_max) ? 2.0 * std::sqrt(D / a_max) : D / w_max + w_max / a_max;
}

struct TMinResult {
  double t{0.0};               // [s]; +∞ when limits_invalid or input_invalid
  bool w0_clamped{false};      // |w0| > w_max came in: t is a lower bound only
  bool limits_invalid{false};  // w_max or a_max not positive-finite
  bool input_invalid{false};   // q0, w0 or q1 non-finite

  /// The candidate may be kept only if none of the flags is set (L3 §4.3).
  [[nodiscard]] bool Usable() const noexcept {
    return !w0_clamped && !limits_invalid && !input_invalid;
  }
};

/// Minimum time from (q0, w0) to (q1, 0) under |q̇| ≤ w_max, |q̈| ≤ a_max.
///
/// |w0| ≤ w_max is a premise, and it is CHECKED: an initial speed above the
/// limit makes the problem ill-posed (the trapezoid branch would contain a
/// negative-duration segment). w0 is then clamped and `w0_clamped` set; the
/// caller drops the candidate. Branches: reverse (moving away — stop first),
/// overshoot (cannot stop in time — stop, come back), triangle, trapezoid.
[[nodiscard]] inline TMinResult TMinChecked(double q0, double w0, double q1, double w_max,
                                            double a_max) noexcept {
  TMinResult r{};
  if (!(a_max > 0.0) || !(w_max > 0.0) || !std::isfinite(a_max) || !std::isfinite(w_max)) {
    r.limits_invalid = true;
    r.t = kInfTime;
    return r;
  }
  if (!std::isfinite(q0) || !std::isfinite(w0) || !std::isfinite(q1)) {
    r.input_invalid = true;
    r.t = kInfTime;
    return r;
  }
  if (std::abs(w0) > w_max) {
    w0 = std::copysign(w_max, w0);
    r.w0_clamped = true;
  }

  const double d = q1 - q0;
  constexpr double kEps = 1e-12;
  if (std::abs(d) < kEps && std::abs(w0) < kEps)
    return r;
  const double s = (std::abs(d) >= kEps) ? std::copysign(1.0, d) : -std::copysign(1.0, w0);
  const double D = std::abs(d);
  const double w = s * w0;  // speed component towards the goal

  if (w < 0.0) {  // moving away: stop first
    r.t = -w / a_max + TRest(D + w * w / (2.0 * a_max), w_max, a_max);
    return r;
  }
  const double d_stop = w * w / (2.0 * a_max);
  if (d_stop > D) {  // overshoot, then come back
    r.t = w / a_max + TRest(d_stop - D, w_max, a_max);
    return r;
  }
  const double w_peak = std::sqrt(a_max * D + 0.5 * w * w);
  if (w_peak <= w_max) {  // triangle
    r.t = (w_peak - w) / a_max + w_peak / a_max;
    return r;
  }
  const double cruise =
      D - (w_max * w_max - w * w) / (2.0 * a_max) - w_max * w_max / (2.0 * a_max);  // trapezoid
  r.t = (w_max - w) / a_max + w_max / a_max + cruise / w_max;
  return r;
}

/// Convenience: the time only. Invalid inputs give +∞ (never a passing 0), but
/// a clamped w0 is invisible here — use TMinChecked() where that matters.
[[nodiscard]] inline double TMin(double q0, double w0, double q1, double w_max,
                                 double a_max) noexcept {
  return TMinChecked(q0, w0, q1, w_max, a_max).t;
}

/// max_i t_min,i over a joint set, with the flags OR-ed. Mismatched span
/// sizes or an empty set are input_invalid.
[[nodiscard]] inline TMinResult MaxJointTMin(std::span<const double> q0, std::span<const double> w0,
                                             std::span<const double> q1,
                                             std::span<const double> w_max,
                                             std::span<const double> a_max) noexcept {
  TMinResult out{};
  const std::size_t n = q0.size();
  if (n == 0 || w0.size() != n || q1.size() != n || w_max.size() != n || a_max.size() != n) {
    out.input_invalid = true;
    out.t = kInfTime;
    return out;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const TMinResult r = TMinChecked(q0[i], w0[i], q1[i], w_max[i], a_max[i]);
    out.t = std::max(out.t, r.t);
    out.w0_clamped = out.w0_clamped || r.w0_clamped;
    out.limits_invalid = out.limits_invalid || r.limits_invalid;
    out.input_invalid = out.input_invalid || r.input_invalid;
  }
  return out;
}

/// Reach-time gate (L3 §4.3): t_k − now − T_arm − T_margin ≥ max_i t_min,i.
/// Subtracting T_arm is comparing against now_lead (plan §3), hence NowLead.
/// Necessary, not sufficient — the γ rollout (§4.8) checks sufficiency.
[[nodiscard]] inline bool ReachTimeFeasible(BallTime t_k, NowLead now_lead,
                                            std::int64_t t_margin_ns,
                                            const TMinResult& joints) noexcept {
  if (!joints.Usable() || !std::isfinite(joints.t) || t_margin_ns < 0)
    return false;
  return LeadSecondsUntil(now_lead, t_k) - static_cast<double>(t_margin_ns) * kNsToS >= joints.t;
}

/// γ window [g_min, g_max] (L3 §4.5).
///   g_min = 1 − d_eff / (‖v‖·T_close,tot)          hand must close in the pocket
///   g_max = min(v_dir,max, v_tcp) / ‖v‖           arm speed limit
struct GammaWindow {
  double g_min{1.0};
  double g_max{0.0};
  bool input_invalid{true};

  [[nodiscard]] bool Feasible() const noexcept { return !input_invalid && g_min <= g_max; }
};

/// The TCP speed the planner may use: η_v · reference.v_max, 0 < η_v ≤ 1
/// (D-9). Planning at the limit itself would saturate L4 on the first
/// prediction change, and with γ derate out of v1 (D-8) this margin is the only
/// buffer. An invalid pair gives 0 (no arm speed — conservative).
[[nodiscard]] constexpr double PlanningTcpSpeed(double eta_v, double v_max) noexcept {
  if (!(eta_v > 0.0) || !(eta_v <= 1.0) || !(v_max > 0.0) || !(v_max < kInfTime))
    return 0.0;
  return eta_v * v_max;
}

/// Clamping to [0, 1] is equivalent to the unclamped test only for physical
/// inputs, so those are checked first: a negative v_dir,max (possible from the
/// DLS estimate) would otherwise be clamped UP to 0 and flip the verdict. A
/// ball speed ≤ 0 has no window (the reference floored it at 1e-6 and reported
/// [0, 1] feasible).
[[nodiscard]] inline GammaWindow ComputeGammaWindow(double v_ball, double v_dir_max,
                                                    double v_tcp_max, double d_eff,
                                                    double t_close_total) noexcept {
  GammaWindow w{};
  const bool bad = !(v_ball > 0.0) || !(d_eff >= 0.0) || !(v_dir_max >= 0.0) ||
                   !(v_tcp_max >= 0.0) || !(t_close_total > 0.0) || !std::isfinite(v_ball) ||
                   !std::isfinite(d_eff) || !std::isfinite(t_close_total);
  if (bad)
    return w;  // {1, 0, invalid}: not feasible
  w.g_min = std::clamp(1.0 - d_eff / (v_ball * t_close_total), 0.0, 1.0);
  w.g_max = std::clamp(std::min(v_dir_max, v_tcp_max) / v_ball, 0.0, 1.0);
  w.input_invalid = false;
  return w;
}

/// Ball speed above which no γ is feasible: min(v_dir,max, v_tcp) + d_eff /
/// T_close,tot. It can disagree with ComputeGammaWindow() by 1 ulp at the
/// boundary; a candidate gate uses one of the two plus a margin. Invalid
/// inputs give 0 (nothing catchable).
[[nodiscard]] inline double MaxCatchableSpeed(double v_dir_max, double v_tcp_max, double d_eff,
                                              double t_close_total) noexcept {
  if (!(v_dir_max >= 0.0) || !(v_tcp_max >= 0.0) || !(d_eff >= 0.0) || !(t_close_total > 0.0) ||
      !std::isfinite(d_eff) || !std::isfinite(t_close_total))
    return 0.0;
  return std::min(v_dir_max, v_tcp_max) + d_eff / t_close_total;
}

struct DirectionalSpeed {
  double v_dir_max{0.0};       // [m/s]; 0 whenever a flag is set (conservative)
  bool limits_invalid{false};  // some q̇_max,i ≤ 0 or non-finite
  bool input_invalid{false};  // non-finite / non-unit v̂, non-finite q̇ᵘ or J_p q̇ᵘ, size mismatch
  bool undetermined{false};  // q̇ᵘ = 0: no joint motion needed, speed not determinable
};

/// v_dir,max ≈ max(0, v̂ᵀ J_p q̇ᵘ) / max_i (|q̇ᵘ_i| / q̇_max,i)   (L3 §4.5)
///
/// q̇ᵘ is the damped least-squares joint velocity that asks for unit speed
/// along v̂ with zero approach-axis rotation; `jp_qdot_u` = J_p q̇ᵘ is what it
/// actually achieves. The numerator is the projection on v̂: DLS (λ > 0) does
/// not achieve [v̂; 0] exactly, the norm would count off-direction leakage as
/// speed, and 1 would report a speed the arm cannot reach near a singularity.
/// A reverse projection counts as 0.
[[nodiscard]] inline DirectionalSpeed DirectionalSpeedMax(
    const Eigen::Vector3d& v_hat, const Eigen::Vector3d& jp_qdot_u, std::span<const double> qdot_u,
    std::span<const double> qdot_max) noexcept {
  DirectionalSpeed r{};
  const std::size_t n = qdot_u.size();
  if (n == 0 || qdot_max.size() != n || !v_hat.allFinite() || !jp_qdot_u.allFinite() ||
      !(std::abs(v_hat.norm() - 1.0) <= 1e-6)) {
    r.input_invalid = true;
    return r;
  }
  double denom = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!(qdot_max[i] > 0.0) || !std::isfinite(qdot_max[i])) {
      r.limits_invalid = true;
      return r;
    }
    if (!std::isfinite(qdot_u[i])) {
      r.input_invalid = true;
      return r;
    }
    denom = std::max(denom, std::abs(qdot_u[i]) / qdot_max[i]);
  }
  if (!(denom > 0.0)) {
    r.undetermined = true;
    return r;
  }
  r.v_dir_max = std::max(0.0, v_hat.dot(jp_qdot_u)) / denom;
  return r;
}

struct StoppingReservation {
  Eigen::Vector3d p_stop{Eigen::Vector3d::Zero()};  // [m] world
  double distance{0.0};                             // [m]
  bool valid{false};
};

/// Where the reference comes to rest after DECEL at a_dec (L3 §4.9):
///   p_stop = p_c + (γ_f‖v‖)² / (2 a_dec) · v̂
/// a_dec is the single shared key supervisor.decel.a_dec (L7 §4.3). A ball at
/// rest at the catch point leaves nothing to stop (distance 0, p_stop = p_c).
[[nodiscard]] inline StoppingReservation StoppingPoint(const Eigen::Vector3d& p_c,
                                                       const Eigen::Vector3d& v_c, double gamma_f,
                                                       double a_dec) noexcept {
  StoppingReservation r{};
  if (!p_c.allFinite() || !v_c.allFinite() || !std::isfinite(gamma_f) || !(gamma_f >= 0.0) ||
      !(gamma_f <= 1.0) || !(a_dec > 0.0) || !std::isfinite(a_dec))
    return r;
  const double speed = v_c.norm();
  r.p_stop = p_c;
  if (speed > 0.0) {
    const double s = gamma_f * speed;
    r.distance = s * s / (2.0 * a_dec);
    r.p_stop += (r.distance / speed) * v_c;
  }
  r.valid = std::isfinite(r.distance) && r.p_stop.allFinite();
  return r;
}

/// Catch error budget σ_gap (L3 §4.6):
///   σ_gap² = (1−γ)²σ_c² + (2γ − γ²)σ_ℓ² + σ_trk² + (‖v‖δ)²
/// σ_c: commit-time sample std, σ_ℓ: latest sample std at the same instant
/// (planning uses σ_ℓ = σ_c, an upper bound), σ_trk: tracking residual,
/// δ: clock offset [s]. Returns NaN for invalid input (γ ∉ [0, 1], negative or
/// non-finite terms) so the gate below fails closed.
[[nodiscard]] inline double CatchErrorSigma(double gamma, double sigma_c, double sigma_l,
                                            double sigma_trk, double v_ball,
                                            double clock_delta) noexcept {
  if (!(gamma >= 0.0) || !(gamma <= 1.0) || !(sigma_c >= 0.0) || !(sigma_l >= 0.0) ||
      !(sigma_trk >= 0.0) || !(v_ball >= 0.0) || !std::isfinite(clock_delta) ||
      !std::isfinite(sigma_c) || !std::isfinite(sigma_l) || !std::isfinite(sigma_trk) ||
      !std::isfinite(v_ball))
    return std::numeric_limits<double>::quiet_NaN();
  const double one_minus = 1.0 - gamma;
  const double vd = v_ball * clock_delta;
  return std::sqrt(one_minus * one_minus * sigma_c * sigma_c +
                   (2.0 * gamma - gamma * gamma) * sigma_l * sigma_l + sigma_trk * sigma_trk +
                   vd * vd);
}

/// n_σ · σ_gap ≤ r_cap. NaN σ_gap or a non-positive r_cap / n_σ fails.
[[nodiscard]] inline bool ErrorBudgetOk(double n_sigma, double sigma_gap, double r_cap) noexcept {
  return n_sigma > 0.0 && r_cap > 0.0 && std::isfinite(r_cap) && n_sigma * sigma_gap <= r_cap;
}

}  // namespace rtc::catching
