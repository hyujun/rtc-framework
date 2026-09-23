// ── γ rollout: choosing (γ_f, T_w) by simulating the reference (L3 §4.8, S6-C) ──
//
// For one candidate the planner runs the L4 soft-catch reference WITHOUT
// saturation, on the RT's own time axis (now_lead = now + T_arm, plan §3),
// against the target the RT will see — the vision snapshot sampled by
// `SampleAt`, the same function the tick calls. It records the peak demanded
// acceleration ‖u_des‖ (pre-saturation, L4 §5.2) and speed ‖ẋ‖, and the error
// ‖e(t_c)‖ at the catch instant. A combination is ACCEPTED when
//
//     max‖u_des‖ ≤ η_a a_max,   max‖ẋ‖ ≤ η_v v_max,   ‖e(t_c)‖ ≤ ε_term.
//
// Among accepted combinations the largest γ_f wins (soft catch, §4.10); on a
// tie, the smaller peak acceleration. If none is accepted, γ_min is tried once
// more on its own (§4.8) — and if that fails too the rollout FAILS, which under
// D-27 is a RANK failure (a penalty), not a removal (decision C).
//
// WHOLE INTERVAL vs WINDOW (S6-C finding). §4.8 judges the whole interval
// [now_lead, t_c], and that is kept as the verdict. But the approach phase —
// the reference pulled from where the arm is toward p_c with γ = 0 — demands
// ω²·‖x0 − p_c‖ (≈ 50 m/s² for 0.5 m at ω = 10) whatever γ_f is, so for any
// real move every combination fails the whole-interval test and the choice of
// γ_f would carry no information. When nothing passes whole, γ_f is chosen by
// the same rule on the WINDOW peaks (inside [t_c − T_w, t_c], where γ acts —
// the L3 table's own measure) and the verdict stays "failed" (the approach
// will saturate: that is what the rank penalty and G3-E's sim count report).
//
// COARSE TO FINE (the budget, §4.8 "연산 예산"). Every combination is screened
// at `dt_coarse`; only the winner is re-run at the control period to confirm.
// The grid is the caller's (`planner.gamma.grid`, `window_grid`).
//
// The screen judges the PEAKS only; ‖e(t_c)‖ is judged by the confirmation.
// Semi-implicit Euler at the coarse step lags a moving target by about γ‖v‖·dt
// — 3–11 mm at 10 ms on the G3-C throws against 0.2–1.5 mm at the control
// period, with the peaks within 3 % of each other (2026-09-23). Judged coarse,
// ε_term = 2 mm refused every soft catch on its own discretisation error.
//
// Allocation-free and noexcept (the planner may run SCHED_FIFO).
#pragma once

#include "rtc_controllers/catching/soft_catch.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_sampler.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

namespace rtc::catching {

inline constexpr std::size_t kMaxGammaGrid = 16;
inline constexpr std::size_t kMaxWindowGrid = 8;

/// What one rollout measured.
struct RolloutPeaks {
  double u_max{0.0};         ///< max ‖u_des‖ over [start, t_c] [m/s²]
  double v_max{0.0};         ///< max ‖ẋ‖ over [start, t_c] [m/s]
  double u_max_window{0.0};  ///< the same, inside the γ window only (L3 §4.8 table)
  double v_max_window{0.0};
  double e_term{std::numeric_limits<double>::infinity()};  ///< ‖e(t_c)‖ [m]
  int steps{0};
  bool valid{false};  ///< false: an input was unusable or a step was rejected
};

/// One rollout of `ds` (a reference constructed with the profile's ω, ζ and
/// NO saturation — huge a_max / v_max) from (x0, ẋ0) at `start` to `t_c`.
/// The γ ramp runs g0 → gf over [t_c − T_w, t_c] (clipped to start).
[[nodiscard]] inline RolloutPeaks RunGammaRollout(
    SoftCatchTranslation& ds, const TrajectorySnapshot& traj, const Eigen::Vector3d& x0,
    const Eigen::Vector3d& xd0, const Eigen::Vector3d& p_c, NowLead start, BallTime t_c, double g0,
    double gf, double t_w, double dt) noexcept {
  RolloutPeaks out;
  if (!(dt > 0.0) || !std::isfinite(dt) || !(t_w > 0.0) || t_c.ns <= start.ns) {
    return out;
  }
  if (!ds.Reset(x0, xd0)) {
    return out;
  }
  const std::int64_t tw_ns = static_cast<std::int64_t>(std::llround(t_w * 1e9));
  const BallTime t0{std::max(start.ns, t_c.ns - tw_ns)};
  if (!ds.SetIntercept(p_c, MakeGammaProfile(g0, gf, t0, t_c, t0))) {
    return out;
  }
  const std::int64_t dt_ns = static_cast<std::int64_t>(std::llround(dt * 1e9));
  if (dt_ns <= 0) {
    return out;
  }
  int hint = 0;
  TranslationOutput o;
  for (std::int64_t t = start.ns; t <= t_c.ns; t += dt_ns) {
    const NowLead now_lead{t};
    const SampleEval s = SampleAt(traj, now_lead, hint);
    if (!s.valid || s.after_horizon) {
      // The prediction does not cover the interval: unjudgeable. An
      // extrapolated sample is exactly what the RT aborts on (HORIZON_EXTRAP).
      return out;
    }
    TargetState target;
    target.p = s.p;
    target.v = s.v;
    target.a = s.a;
    o = ds.Step(target, ProfileSeconds(now_lead, t0), dt);
    if (!o.valid) {
      return out;
    }
    const double u = o.u_des.norm();
    const double v = o.xd.norm();
    out.u_max = std::max(out.u_max, u);
    out.v_max = std::max(out.v_max, v);
    if (t >= t0.ns) {
      out.u_max_window = std::max(out.u_max_window, u);
      out.v_max_window = std::max(out.v_max_window, v);
    }
    ++out.steps;
  }
  out.e_term = o.e.norm();
  out.valid = std::isfinite(out.u_max) && std::isfinite(out.v_max) && std::isfinite(out.e_term);
  return out;
}

/// Acceptance thresholds (L3 §4.8) and the grids.
struct RolloutSettings {
  double a_max{0.0};                    ///< `reference.a_max` [m/s²]
  double v_max{0.0};                    ///< `reference.v_max` [m/s]
  double eta_a{0.8};                    ///< `planner.gamma.eta_a`
  double eta_v{0.9};                    ///< `planner.gamma.eta_v`
  double eps_term{0.002};               ///< `planner.gamma.eps_term` [m]
  double dt_coarse{0.01};               ///< screening step [s] (`planner.rollout.dt_coarse`)
  double dt_fine{0.002};                ///< confirmation step [s] (the control period)
  std::span<const double> gamma_grid;   ///< `planner.gamma.grid`
  std::span<const double> window_grid;  ///< `planner.gamma.window_grid` [s]
};

struct RolloutChoice {
  bool accepted{false};     ///< a combination passed on the WHOLE interval (fine-confirmed)
  bool window_only{false};  ///< γ_f chosen on window peaks — the approach saturates
  bool tried_min{false};    ///< the γ_min fallback was needed
  double gamma_f{0.0};
  double t_w{0.0};
  RolloutPeaks peaks{};  ///< the chosen combination's peaks (fine when accepted)
  int rollouts{0};       ///< how many rollouts this choice cost
};

[[nodiscard]] inline bool RolloutAccepts(const RolloutPeaks& p, const RolloutSettings& s) noexcept {
  return p.valid && p.u_max <= s.eta_a * s.a_max && p.v_max <= s.eta_v * s.v_max &&
         p.e_term <= s.eps_term;
}

/// The coarse screen: the peak thresholds without ‖e(t_c)‖, which the coarse
/// step cannot resolve (see the file header).
[[nodiscard]] inline bool RolloutPeaksAccept(const RolloutPeaks& p,
                                             const RolloutSettings& s) noexcept {
  return p.valid && p.u_max <= s.eta_a * s.a_max && p.v_max <= s.eta_v * s.v_max;
}

/// The same peak thresholds on the γ window only (see the file header). A
/// coarse-step judgement, so it leaves ‖e(t_c)‖ out as well.
[[nodiscard]] inline bool RolloutWindowAccepts(const RolloutPeaks& p,
                                               const RolloutSettings& s) noexcept {
  return p.valid && p.u_max_window <= s.eta_a * s.a_max && p.v_max_window <= s.eta_v * s.v_max;
}

/// Choose (γ_f, T_w) for one candidate over the grid restricted to
/// [g_min, g_max] (§4.8). `accepted` false is the caller's RANK failure; on
/// that path `gamma_f` is the window-rule choice if one exists, else γ_min
/// (clipped to [0, 1]) with the longest window — the least the hand needs.
[[nodiscard]] inline RolloutChoice ChooseGamma(
    SoftCatchTranslation& ds, const TrajectorySnapshot& traj, const Eigen::Vector3d& x0,
    const Eigen::Vector3d& xd0, const Eigen::Vector3d& p_c, NowLead start, BallTime t_c, double g0,
    double g_min, double g_max, const RolloutSettings& s) noexcept {
  RolloutChoice best;
  const double lo = std::clamp(g_min, 0.0, 1.0);
  const double hi = std::clamp(g_max, 0.0, 1.0);
  const double t_w_longest =
      s.window_grid.empty() ? 0.0 : *std::max_element(s.window_grid.begin(), s.window_grid.end());
  best.gamma_f = lo;
  best.t_w = t_w_longest;

  struct Pick {
    bool found{false};
    double g{0.0};
    double tw{0.0};
    double u{std::numeric_limits<double>::infinity()};
    RolloutPeaks peaks{};

    void Offer(double g_new, double tw_new, double u_new, const RolloutPeaks& p) noexcept {
      if (!found || g_new > g || (g_new == g && u_new < u)) {
        found = true;
        g = g_new;
        tw = tw_new;
        u = u_new;
        peaks = p;
      }
    }
  };

  Pick whole;
  Pick window;
  const auto consider = [&](double g, double tw) {
    const RolloutPeaks p =
        RunGammaRollout(ds, traj, x0, xd0, p_c, start, t_c, g0, g, tw, s.dt_coarse);
    ++best.rollouts;
    if (RolloutPeaksAccept(p, s)) {
      whole.Offer(g, tw, p.u_max, p);
    }
    if (RolloutWindowAccepts(p, s)) {
      window.Offer(g, tw, p.u_max_window, p);
    }
  };
  for (const double g : s.gamma_grid) {
    if (g < lo || g > hi) {
      continue;
    }
    for (const double tw : s.window_grid) {
      consider(g, tw);
    }
  }
  if (!whole.found && !window.found && lo <= hi) {
    // §4.8: nothing on the grid — try γ_min itself once.
    best.tried_min = true;
    for (const double tw : s.window_grid) {
      consider(lo, tw);
    }
  }
  if (whole.found) {
    // Fine confirmation of the winner at the control period.
    const RolloutPeaks fine =
        RunGammaRollout(ds, traj, x0, xd0, p_c, start, t_c, g0, whole.g, whole.tw, s.dt_fine);
    ++best.rollouts;
    if (RolloutAccepts(fine, s)) {
      best.gamma_f = whole.g;
      best.t_w = whole.tw;
      best.peaks = fine;
      best.accepted = true;
      return best;
    }
    // Refuted at the control period: the verdict is "failed", and γ_f falls
    // to the window rule below like any other unaccepted candidate.
  }
  if (window.found) {
    best.window_only = true;
    best.gamma_f = window.g;
    best.t_w = window.tw;
    best.peaks = window.peaks;
  }
  return best;
}

}  // namespace rtc::catching
