// ── The planner's rank gates, one allocation-free core (dynamic_catching S6-B) ──
//
// Reach time (L3 §4.3), γ window (§4.5) and the stopping point (§4.9) for ONE
// candidate that already has a catch posture. Two callers, one function:
//
//   - the offline gate map (`catch_gate_batch`, S3.5b) — start state = the wait
//     pose at rest;
//   - the runtime planner (`PlannerCycle`, S6-B) — start state = the RT's
//     current command (q_c, q̇_c), as §4.3 specifies.
//
// That the two call the SAME code is the equivalence G3-I asks for (map and
// runtime judge a candidate alike): before S6-B the body lived inside the
// batch's `JudgeGates`, over `std::vector` and `Eigen::VectorXd`, which the
// planner thread (RT-1~10) cannot use. It was moved here verbatim and
// `JudgeGates` now wraps it.
//
// Under D-27 these are RANK gates at runtime (decision C, 2026-09-23): a
// candidate that fails one is penalised in the score, not removed — the map
// keeps using them as a strict filter. The core only reports; the policy is the
// caller's.
#pragma once

#include "rtc_controllers/catching/time_feasibility.hpp"
#include "rtc_controllers/catching/time_types.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>

namespace rtc::catching {

/// First gate, in planner order, that turned the candidate away.
enum class GateReason : std::uint8_t {
  kNone = 0,          ///< every gate judged here passed
  kReachInvalid,      ///< reach-time inputs or limits unusable (fail closed)
  kReachTime,         ///< the arm cannot be at q* by t_c (L3 §4.3)
  kGammaInvalid,      ///< v_dir,max or the window inputs unusable (fail closed)
  kGammaWindowEmpty,  ///< g_min > g_max, or the ball is inside the speed margin (§4.5)
  kStopInvalid,       ///< the stopping point could not be formed (§4.9)
};

[[nodiscard]] constexpr std::string_view GateReasonName(GateReason reason) noexcept {
  switch (reason) {
    case GateReason::kNone:
      return "none";
    case GateReason::kReachInvalid:
      return "reach_invalid";
    case GateReason::kReachTime:
      return "reach_time";
    case GateReason::kGammaInvalid:
      return "gamma_invalid";
    case GateReason::kGammaWindowEmpty:
      return "gamma_window_empty";
    case GateReason::kStopInvalid:
      return "stop_invalid";
  }
  return "unknown";
}

/// Everything one judgement needs. Spans are the caller's storage, all in the
/// model's joint order and of equal length (the arm's nv).
struct RankGateInputs {
  std::span<const double> q0;         ///< start posture [rad]
  std::span<const double> w0;         ///< start velocity [rad/s]
  std::span<const double> q_star;     ///< catch posture [rad]
  std::span<const double> qdot_plan;  ///< η_v · q̇_max [rad/s] — the PLANNING limits (S4.4)
  std::span<const double> qddot_max;  ///< D-16 acceleration box [rad/s²]
  std::span<const double> qdot_u;     ///< DLS unit-speed joint velocity at q* (L3 §4.5)
  Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};        ///< model world [m]
  Eigen::Vector3d v_ball{Eigen::Vector3d::Zero()};     ///< model world [m/s]
  Eigen::Vector3d jp_qdot_u{Eigen::Vector3d::Zero()};  ///< J_p q̇ᵘ [m/s]
  BallTime t_k{0};                                     ///< catch instant
  NowLead now_lead{0};                                 ///< the planning 'now' + T_arm
  std::int64_t t_margin_ns{0};                         ///< `planner.time.margin`
  double v_tcp_plan{0.0};                              ///< η_v · `reference.v_max` [m/s]
  double d_eff{0.0};                                   ///< `planner.hand.d_eff` [m]
  double t_close_total{0.0};                           ///< T_close,e2e + h/2 [s]
  double gamma_margin{0.0};                            ///< `planner.gamma.margin` [m/s]
  double a_dec{0.0};                                   ///< `supervisor.decel.a_dec` [m/s²]
};

/// Every gate's own result, not just the first failure.
struct RankGateResult {
  double lead_s{0.0};  ///< t_c − now_lead [s]
  TMinResult reach{};
  bool reach_ok{false};
  DirectionalSpeed direction{};
  GammaWindow window{};
  double max_catchable{0.0};  ///< [m/s]
  bool gamma_usable{false};
  bool gamma_ok{false};
  StoppingReservation stop_gamma_min{};  ///< at g_min — the least the arm must retreat
  StoppingReservation stop_gamma_max{};  ///< at g_max — the most it may
  GateReason reason{GateReason::kReachInvalid};
};

/// Judge one candidate. Allocation-free, noexcept. Order and arithmetic are
/// the S3.5b map's exactly (the map now calls this), so a change here changes
/// both — which is the point.
[[nodiscard]] inline RankGateResult JudgeRankGates(const RankGateInputs& in) noexcept {
  RankGateResult r;

  // ── reach time (L3 §4.3): (q0, w0) → (q*, 0) ──────────────────────────────
  r.reach = MaxJointTMin(in.q0, in.w0, in.q_star, in.qdot_plan, in.qddot_max);
  r.lead_s = LeadSecondsUntil(in.now_lead, in.t_k);
  r.reach_ok = ReachTimeFeasible(in.t_k, in.now_lead, in.t_margin_ns, r.reach);

  // ── γ window (L3 §4.5) ─────────────────────────────────────────────────────
  const double speed = in.v_ball.norm();
  if (speed > 0.0 && std::isfinite(speed)) {
    const Eigen::Vector3d v_hat = in.v_ball / speed;
    r.direction = DirectionalSpeedMax(v_hat, in.jp_qdot_u, in.qdot_u, in.qdot_plan);
    // `undetermined` (q̇ᵘ = 0: no joint motion asks for unit speed) is not a
    // speed of 0 — it is a speed the estimate could not produce, so it must not
    // enter the window as physics.
    if (!r.direction.limits_invalid && !r.direction.input_invalid && !r.direction.undetermined) {
      r.window = ComputeGammaWindow(speed, r.direction.v_dir_max, in.v_tcp_plan, in.d_eff,
                                    in.t_close_total);
      r.max_catchable =
          MaxCatchableSpeed(r.direction.v_dir_max, in.v_tcp_plan, in.d_eff, in.t_close_total);
      r.gamma_usable = !r.window.input_invalid;
    }
  } else {
    r.direction.input_invalid = true;
  }
  r.gamma_ok = r.gamma_usable && r.window.Feasible() && speed + in.gamma_margin <= r.max_catchable;

  // ── stopping point (L3 §4.9), at both ends of the window ───────────────────
  if (r.gamma_usable) {
    r.stop_gamma_min = StoppingPoint(in.p_c, in.v_ball, r.window.g_min, in.a_dec);
    r.stop_gamma_max = StoppingPoint(in.p_c, in.v_ball, r.window.g_max, in.a_dec);
  }

  if (!r.reach.Usable() || !std::isfinite(r.reach.t)) {
    r.reason = GateReason::kReachInvalid;
  } else if (!r.reach_ok) {
    r.reason = GateReason::kReachTime;
  } else if (!r.gamma_usable) {
    r.reason = GateReason::kGammaInvalid;
  } else if (!r.gamma_ok) {
    r.reason = GateReason::kGammaWindowEmpty;
  } else if (!r.stop_gamma_min.valid) {
    r.reason = GateReason::kStopInvalid;
  } else {
    r.reason = GateReason::kNone;
  }
  return r;
}

}  // namespace rtc::catching
