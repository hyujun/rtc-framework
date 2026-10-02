#ifndef INTEGRATED_BRINGUP_LOGGING_PLANNER_EVENTS_CSV_HPP_
#define INTEGRATED_BRINGUP_LOGGING_PLANNER_EVENTS_CSV_HPP_

// ── planner_events.csv — one row per non-idle planner wake (S6-B, decision E) ─
//
// The planner's own account of each wake, drained from the thread's SPSC ring
// by the controller's 1 Hz aux timer (never written on the planner thread).
// What the frozen `CatchingState` message cannot carry lives here (D-20):
// candidate counts, the judgement-reject histogram, the chosen candidate's
// rank-gate bitmask (one column per bit as well, for plotting), the switching
// decision, the search time and the receive → publish latency D-7a is judged on
// (plan §7.2, G3-J's event record).
//
// Written to `<session>/controllers/<config_key>/planner_events.csv`, next to
// the controller's tick record. Idle wakes (no RT state, a mode with nothing to
// plan for) are not recorded unless they saw a trial reset or produced a
// monitorOnly σ_ℓ — at 20 Hz they would drown the rows that say something.
//
// The `decel_*` columns (MPC E1-F03) are the decel step's account. Readers
// select columns by NAME — the set has changed (E1-F05 dropped the stop-only
// planner's `decel_h_s` / `decel_qdd_trusted`, MD-70) and a log from before a
// change simply lacks the newer names. A wake whose decel step only waited
// (off, up to date, past the replan window) does not earn a row on its own.

#include "rtc_controllers/catching/planner_cycle.hpp"

#include <cmath>
#include <cstddef>
#include <ostream>

namespace integrated_bringup {

inline void WritePlannerEventsHeader(std::ostream& os) {
  os << "wake_ns,publish_ns,recv_to_publish_ms,outcome,mode,reset_seen,cov_matched,"
        "plan_id,plan_valid,plan_reason,snapshot_sequence,track_generation,settling,"
        "n_in_window,n_ik,n_pass,rej_input,rej_ik,rej_manipulability,rej_workspace,"
        "rej_not_evaluated,budget_hit,search_us,ik_us_max,rank_mask,rank_uncertainty,"
        "rank_reach,rank_gamma,rank_commit_lead,rank_error_budget,score,lead_s,gamma_f,"
        "decision,sigma_l,rank_rollout,t_w,rollout_window_only,n_rollouts,rollout_us_max,"
        "g_min,g_max,v_dir_max,max_catchable,decel_outcome,decel_k,decel_n_nodes,decel_seq,"
        "decel_publish_ns,decel_x0_clamped,decel_from_segment,"
        "decel_presolved,decel_cold_retry,decel_iterations,decel_qp_status,decel_core_reason,"
        "decel_solve_us,decel_slack_max,decel_slack_terminal_max,decel_tau_ratio_max\n";
}

/// Whether the decel step did something worth a row on its own.
[[nodiscard]] inline bool DecelStepWorthRecording(rtc::catching::DecelOutcome o) noexcept {
  using rtc::catching::DecelOutcome;
  return o != DecelOutcome::kOff && o != DecelOutcome::kUpToDate &&
         o != DecelOutcome::kPastReplanWindow;
}

/// Whether a wake is worth a row (see the file header).
[[nodiscard]] inline bool PlannerEventWorthRecording(
    const rtc::catching::PlannerCycleRecord& r) noexcept {
  return r.outcome != rtc::catching::CycleOutcome::kIdle || r.reset_seen ||
         std::isfinite(r.search.sigma_l) || DecelStepWorthRecording(r.decel.outcome);
}

inline void WritePlannerEventsRow(std::ostream& os, const rtc::catching::PlannerCycleRecord& r) {
  using rtc::catching::JudgeReject;
  const auto& s = r.search;
  const auto rej = [&s](JudgeReject j) { return s.judge_rejects[static_cast<std::size_t>(j)]; };
  const auto bit = [&s](std::uint16_t b) { return (s.chosen_rank_mask & b) != 0 ? 1 : 0; };
  const bool published = r.outcome == rtc::catching::CycleOutcome::kPublished;
  // Latency only where it means something: a publish, from a real receive.
  const double latency_ms = published && r.traj_recv_ns > 0 && r.publish_ns > 0
                                ? static_cast<double>(r.publish_ns - r.traj_recv_ns) * 1e-6
                                : std::nan("");
  os << r.wake_ns << ',' << r.publish_ns << ',' << latency_ms << ','
     << rtc::catching::CycleOutcomeName(r.outcome) << ',' << static_cast<int>(r.mode) << ','
     << (r.reset_seen ? 1 : 0) << ',' << (r.cov_matched ? 1 : 0) << ',' << r.plan_id << ','
     << (r.plan_valid ? 1 : 0) << ',' << static_cast<int>(r.reason) << ',' << r.snapshot_sequence
     << ',' << r.track_generation << ',' << (s.settling ? 1 : 0) << ',' << s.n_in_window << ','
     << s.n_ik << ',' << s.n_pass << ',' << rej(JudgeReject::kInput) << ',' << rej(JudgeReject::kIk)
     << ',' << rej(JudgeReject::kManipulability) << ',' << rej(JudgeReject::kWorkspace) << ','
     << rej(JudgeReject::kNotEvaluated) << ',' << (s.budget_hit ? 1 : 0) << ','
     << s.search_ns / 1000 << ',' << s.ik_ns_max / 1000 << ',' << s.chosen_rank_mask << ','
     << bit(rtc::catching::kRankUncertainty) << ',' << bit(rtc::catching::kRankReach) << ','
     << bit(rtc::catching::kRankGamma) << ',' << bit(rtc::catching::kRankCommitLead) << ','
     << bit(rtc::catching::kRankErrorBudget) << ',' << s.chosen_score << ',' << s.chosen_lead_s
     << ',' << s.chosen_gamma_f << ',' << rtc::catching::SwitchDecisionName(s.decision) << ','
     << s.sigma_l << ',' << bit(rtc::catching::kRankRollout) << ',' << s.chosen_t_w << ','
     << (s.chosen_rollout_window_only ? 1 : 0) << ',' << s.n_rollouts << ','
     << s.rollout_ns_max / 1000 << ',' << s.chosen_g_min << ',' << s.chosen_g_max << ','
     << s.chosen_v_dir_max << ',' << s.chosen_max_catchable << ',';
  const auto& d = r.decel;
  os << rtc::catching::DecelOutcomeName(d.outcome) << ',' << d.k << ',' << d.n_nodes << ','
     << d.decel_seq << ',' << d.publish_ns << ',' << (d.x0_clamped ? 1 : 0) << ','
     << (d.from_segment ? 1 : 0) << ',' << (d.presolved ? 1 : 0) << ',' << (d.cold_retry ? 1 : 0)
     << ',' << d.iterations << ',' << d.qp_status << ','
     << rtc::catching::DecelMpcReasonName(d.core_reason) << ',' << d.solve_ns / 1000 << ','
     << d.slack_max << ',' << d.slack_terminal_max << ',' << d.tau_ratio_max << '\n';
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_PLANNER_EVENTS_CSV_HPP_
