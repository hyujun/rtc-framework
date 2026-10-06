// ── The record one search wake leaves (L3 §8) ────────────────────────────────
//
// What a CatchSearch (catch_search.hpp) fills on every wake and PlannerCycle
// carries out in its PlannerCycleRecord: the candidate funnel, why candidates
// were removed, what was chosen, and what the search decided about the plan
// the RT follows. It is the planner CSV's body.
//
// It lives apart from any one search so that a second implementation fills the
// same record without including the first. An implementation fills what it
// has and leaves the rest at the default; the cycle reads ONE field back,
// `publish`.
//
// Trivially copyable: the cycle copies it into a SeqLock and an SPSC ring.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rtc::catching {

/// Why a JUDGEMENT gate removed a candidate.
enum class JudgeReject : std::uint8_t {
  kNone = 0,
  kInput,           ///< non-finite p/v, or ‖v̂‖ below v_eps (NUM-7)
  kIk,              ///< IK did not converge / was refused
  kManipulability,  ///< the IK's catchability gate (D-18)
  kWorkspace,       ///< p_c or p_stop outside `workspace.catch_box`
  kNotEvaluated,    ///< outside the IK budget (pre-filter or budget_s)
};
inline constexpr std::size_t kJudgeRejectCount = 6;

/// What the switching rule decided about the plan the RT is following.
enum class SwitchDecision : std::uint8_t {
  kNoCurrent = 0,   ///< the RT follows no plan of ours — publish the best
  kReplaced,        ///< a better candidate replaced it (§4.7)
  kHeldHysteresis,  ///< not better by delta_J
  /// Better — or the followed candidate with a moved prediction — but the
  /// step the switch would put into u_des is over the budget (§4.7).
  kHeldJump,
  kHeldFreeze,       ///< within T_freeze of the current t_c (decision G)
  kHeldNoCandidate,  ///< no candidate passed; the RT keeps what it has
  /// The followed candidate is still the best, but its predicted catch point
  /// moved by more than `planner.search.grid.gamma.eps_term`: republished with the new
  /// p_c (jump limits and freeze still apply) — hysteresis must not pin a
  /// catch point the prediction has left (2026-09-23 /code-review).
  kRefreshed,
};

[[nodiscard]] constexpr const char* SwitchDecisionName(SwitchDecision d) noexcept {
  switch (d) {
    case SwitchDecision::kNoCurrent:
      return "no_current";
    case SwitchDecision::kReplaced:
      return "replaced";
    case SwitchDecision::kHeldHysteresis:
      return "held_hysteresis";
    case SwitchDecision::kHeldJump:
      return "held_jump";
    case SwitchDecision::kHeldFreeze:
      return "held_freeze";
    case SwitchDecision::kHeldNoCandidate:
      return "held_no_candidate";
    case SwitchDecision::kRefreshed:
      return "refreshed";
  }
  return "unknown";
}

/// One cycle's search diagnostics (L3 §8) — the planner CSV's body.
struct SearchStats {
  bool settling{false};          ///< inside n_settle after a track change
  std::uint16_t n_in_window{0};  ///< candidates whose lead is in the slice window
  std::uint16_t n_ik{0};         ///< IK solves run
  std::uint16_t n_pass{0};       ///< candidates past every judgement gate
  std::array<std::uint16_t, kJudgeRejectCount> judge_rejects{};
  bool budget_hit{false};  ///< budget_s ran out with IK candidates left
  std::int64_t search_ns{0};
  std::int64_t ik_ns_max{0};       ///< the slowest single IK this cycle
  std::int64_t rollout_ns_max{0};  ///< the slowest single candidate's rollout choice
  std::uint16_t n_rollouts{0};     ///< rollouts run this cycle (coarse + fine)
  /// The chosen candidate (valid only if a plan was produced).
  std::uint16_t chosen_rank_mask{0};
  double chosen_score{0.0};
  double chosen_lead_s{0.0};
  double chosen_gamma_f{0.0};
  double chosen_t_w{0.0};                  ///< the γ window the rollout chose [s]
  bool chosen_rollout_window_only{false};  ///< γ_f chosen on window peaks (approach saturates)
  /// The chosen candidate's γ window (L3 §4.5) as judged — what an offline
  /// analysis had to rebuild from FK before S8-I (#537 5847660329). NaN when
  /// the window was unjudgeable (`gamma_usable` false) or no plan was produced.
  double chosen_g_min{std::numeric_limits<double>::quiet_NaN()};
  double chosen_g_max{std::numeric_limits<double>::quiet_NaN()};
  double chosen_v_dir_max{std::numeric_limits<double>::quiet_NaN()};      ///< [m/s], DLS
  double chosen_max_catchable{std::numeric_limits<double>::quiet_NaN()};  ///< [m/s]
  SwitchDecision decision{SwitchDecision::kNoCurrent};
  /// Whether the caller should publish the returned snapshot. False when the
  /// switching rule holds the RT's current plan.
  bool publish{true};
  /// monitorOnly (§4.6): σ_ℓ at the committed t_c from the newest covariance.
  double sigma_l{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace rtc::catching
