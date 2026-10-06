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
#include <type_traits>

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

/// Why the NLP search (nlp_catch_search.hpp) removed a candidate, or why a wake
/// of it chose none. Ordered as the checks run, so that "how far did this
/// candidate get" is a comparison of two values: a wake that chose nothing
/// reports the reason of the candidate that got farthest.
enum class NlpReject : std::uint8_t {
  kNone = 0,  ///< a valid candidate; a wake that chose one
  /// Outside the window around the cell of the first plan the RT followed on
  /// this track (`follow_window`) — the first check, before any other is run.
  kFollowWindow,
  kLeadShort,    ///< t_c − t_0 below the minimum lead (S1)
  kBallInvalid,  ///< the prediction cannot be sampled at one of its nodes, or is too slow
  kWorkspace,    ///< the catch point is outside the catch box
  kCovariance,   ///< chance rows are on and the catch-node covariance is not usable
  /// No reported segment to start from at this candidate's node 0 — as a
  /// wake's reason: the RT follows a plan and reports no segment at all.
  kNoSource,
  kIk,              ///< the catch-pose IK did not converge or was refused
  kManipulability,  ///< the IK's catchability gate
  kReach,           ///< the catch pose cannot be reached in the time (S4)
  kSpeedWindow,     ///< the closing-speed window is empty (S3)
  kNotRanked,       ///< passed screening, outside the wake's solve budget
  kDeadline,        ///< its solve ran past its share of the budget
  kSolverRejected,  ///< the solve was refused before any iterate existed
  kHardRow,         ///< a hard row other than a chance row is violated
  kChance,          ///< only chance rows are violated
  kUnconverged,     ///< every hard row holds, the solve did not converge
  // ── A wake's reasons that are no candidate's ──
  kNoCandidate,  ///< no lattice instant is ahead of the arm inside the horizon
  kNotAtRest,    ///< the RT follows no plan and the arm's command is moving
  kRtInvalid,    ///< the RT's report cannot be planned from (width, age, NaN)
};
inline constexpr std::size_t kNlpRejectCount = 20;

[[nodiscard]] constexpr const char* NlpRejectName(NlpReject r) noexcept {
  switch (r) {
    case NlpReject::kNone:
      return "none";
    case NlpReject::kFollowWindow:
      return "follow_window";
    case NlpReject::kLeadShort:
      return "lead_short";
    case NlpReject::kBallInvalid:
      return "ball_invalid";
    case NlpReject::kWorkspace:
      return "workspace";
    case NlpReject::kCovariance:
      return "covariance";
    case NlpReject::kNoSource:
      return "no_source";
    case NlpReject::kIk:
      return "ik";
    case NlpReject::kManipulability:
      return "manipulability";
    case NlpReject::kReach:
      return "reach";
    case NlpReject::kSpeedWindow:
      return "speed_window";
    case NlpReject::kNotRanked:
      return "not_ranked";
    case NlpReject::kDeadline:
      return "deadline";
    case NlpReject::kSolverRejected:
      return "solver_rejected";
    case NlpReject::kHardRow:
      return "hard_row";
    case NlpReject::kChance:
      return "chance";
    case NlpReject::kUnconverged:
      return "unconverged";
    case NlpReject::kNoCandidate:
      return "no_candidate";
    case NlpReject::kNotAtRest:
      return "not_at_rest";
    case NlpReject::kRtInvalid:
      return "rt_invalid";
  }
  return "unknown";
}

/// What a wake of the NLP search adds to SearchStats (E1-F14 #740). Left at
/// its default by every other search (`ran` false).
struct NlpSearchStats {
  bool ran{false};                     ///< an NLP search filled this block
  NlpReject reason{NlpReject::kNone};  ///< the wake's reason (kNone: a plan was chosen)
  std::uint16_t n_lattice{0};          ///< lattice instants ahead of the arm, inside the horizon
  std::uint16_t n_screened{0};         ///< of those, past every necessary condition
  std::uint16_t n_solved{0};           ///< solves run
  std::uint16_t n_valid{0};            ///< solves that gave a valid candidate
  std::array<std::uint16_t, kNlpRejectCount> rejects{};  ///< candidates per reason
  // ── The chosen candidate (meaningful only if a plan was produced) ──
  std::int64_t chosen_index{0};        ///< its lattice index
  std::int32_t chosen_n_pre{0};        ///< pre-catch intervals of its arm grid
  std::int32_t chosen_iterations{0};   ///< SQP iterations of its solve
  std::uint32_t chosen_source_seq{0};  ///< the segment its start state was read from; 0 = rest
  bool chosen_x0_clamped{false};       ///< its start state was projected into the solve's box
  double chosen_lead_s{0.0};           ///< T = t_c − t_0 [s]
  double chosen_wait_s{0.0};           ///< t_s − t_0 [s], in [0, Δ_a)
  double chosen_phi{0.0};              ///< Φ = J⋆ + J_time + J_switch
  double chosen_j_reference{0.0};      ///< J⋆: the solve's cost up to the catch node
  double chosen_j_stop{0.0};           ///< the stop part's cost — recorded, not chosen on
  double chosen_j_time{0.0};
  double chosen_j_switch{0.0};
  // ── After adoption: the cell of the first plan the RT followed on this
  // track, and where the chosen candidate is from it. Filled while the RT
  // follows a plan of this track, whether or not a window is configured. ──
  bool follow_anchor_set{false};             ///< that cell is known on this wake
  std::int64_t follow_anchor_index{0};       ///< its lattice index i_a
  std::uint16_t n_follow_window{0};          ///< candidates the window removed
  std::int32_t chosen_cells_from_anchor{0};  ///< i − i_a of the chosen candidate
  std::int64_t chosen_ns_from_first{0};      ///< its t_c − that first plan's t_c [ns]
  bool chosen_at_window_edge{false};         ///< |i − i_a| is the window's width (window on)
  // ── The wake ──
  std::int64_t screen_ns{0};     ///< time spent before the first solve
  std::int64_t solve_ns_max{0};  ///< the slowest single solve
  /// max |q_cmd − q| and max |q̇_cmd − q̇| between the RT's command and the
  /// segment it reports following, at the instant that command was sampled
  /// for [rad], [rad/s]. NaN when the RT reports following none.
  double cmd_gap_q{std::numeric_limits<double>::quiet_NaN()};
  double cmd_gap_qd{std::numeric_limits<double>::quiet_NaN()};
};

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
  /// The NLP search's own account (E1-F14). Not a column of the planner CSV
  /// and not part of the trace digest of the fields above.
  NlpSearchStats nlp{};
};

static_assert(std::is_trivially_copyable_v<SearchStats>);

}  // namespace rtc::catching
