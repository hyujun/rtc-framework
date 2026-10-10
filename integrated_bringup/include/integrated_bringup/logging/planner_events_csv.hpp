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
// (L3 §5.3, G3-J's event record).
//
// Written to `<session>/controllers/<config_key>/planner_events.csv`, next to
// the controller's tick record. Idle wakes (no RT state, a mode with nothing to
// plan for) are not recorded unless they saw a trial reset or produced a
// monitorOnly σ_ℓ — at 20 Hz they would drown the rows that say something.
//
// `search_valid` says whether this wake's search produced a valid plan;
// `plan_valid` says a valid plan was PUBLISHED. Under `mode: mpc` a plan whose
// first segment is withheld is not published (MD-62): that wake reads
// search_valid 1, plan_valid 0, outcome held.
//
// The `segment_*` columns are the MPC segment planner's account of the wake (MPC
// E1-F03 · E1-F08): which solve it was (`segment_kind`), how it ended
// (`segment_outcome`, `segment_core_reason`), and the catch node as solved —
// position [m] and axis [rad] error, γ, ‖v_rel‖ [m/s], the velocity slack. A
// value the wake did not compute is NaN (0 for a flag or a count).
// `segment_k` is the grid index of node 0: −n_pre for a pre-catch grid point,
// k ≥ 0 for the stop grid point t_c + k·Δ_s — `segment_kind` says which.
// A wake whose segment step only waited (off, up to date, past the replan
// window) does not earn a row on its own.
//
// The `nlp_*` columns are the NLP search's own account of the wake (E1-F18):
// its reason (`nlp_reason` — the reason of the candidate that got farthest
// when it chose none, `off` on a wake no NLP search ran), the candidate funnel,
// how many candidates each check removed (`nlp_rej_*`), and the chosen
// candidate's cost split — `nlp_j_reference` is J⋆, `nlp_phi` what it was
// chosen on. On a wake with `nlp_rej_deadline` > 0, `nlp_solve_us_max` may be
// the instant a candidate's solve was cut at rather than the time a solve
// took: that reason covers a solve the core cut, one that ended past its
// deadline, and a valid one the wake finished too late for — the wake's row
// cannot tell which the slowest was. The grid search's funnel stays in the
// columns before them.
//
// The columns from `segment_qp_solves` on are the mpc_docking core's account of
// a solve that reached an iterate (NaN — 0 for a count — for every other
// planner, and for a solve the core refused outright): QP counts, where the
// time went, the last QP's stationarity, the violation of each row group at
// the returned iterate (`segment_viol_*`, the group's unit, 0 = holds) and the
// QP's elastic per group, the group the core named when it ended infeasible,
// and the catch node — closing speed, σ_s [m], σ_t [s], and the SIGNED room of
// the lateral rows and of the timing row [m] (`segment_chance_*`: negative is
// the violation). `segment_solve_us` of a row whose `segment_core_reason` is
// `deadline` is the instant the core was cut at, not the time the solve takes.
// `segment_cut_site` names the QP that solve was kept from starting
// (MpcDockingCutSiteName: `init_qp`, `iteration`, `cold_retry`,
// `penalty_reset`, `penalty_probe`; `none` when the core did not cut it), and
// `segment_start_from_memory` is 1 on a first solve the planner handed the
// iterate the previous wake's solve of the same plan ended on (0: the
// search's catch pose, and every other solve; the core still runs its
// initialisation QP toward that iterate when the start state has moved —
// `segment_qp_solves` − `segment_iterations` says so). These two are written
// for a solve without an iterate too: cut before its initialisation QP
// (`init_qp`) a solve has none, and the rest of the block is NaN (0 for a
// count). The last-QP columns (`segment_kkt_residual`, `segment_grad_norm`,
// `segment_complementarity`, `segment_elastic_*`) are NaN on a row with
// `segment_iterations` 0 — an evaluation, or a solve cut before its first
// iteration's QP: no QP's step was judged.
//
// `replace_step` says where a wake's attempt to replace the followed plan
// ended (ReplaceStepName; `none` on a wake that attempted none), and the
// `replacement_*` columns are the first solve of a replacement that was
// withheld — the wake's `segment_*` columns are then the replan's. Only its
// outcome, core reason, iterations and time are written: the docking block of
// that solve is not (a second copy of those columns for a rare row).
//
// `cov_n` and `chosen_sigma_c` (#798, after the replacement columns) are the
// wake's covariance box (its point count) and the raw √λ_max of the ball's
// position covariance at the chosen candidate's catch node — NaN when no plan
// was produced or the node had no valid covariance. `sigma_l` is the monitor's
// column (a search wake leaves it NaN, #800): a search wake's σ is here.
//
// nlp_candidates.csv (same directory, #798 — E1-F19 part 2's instrument):
// one row per candidate the NLP search SOLVED on a wake, drained from the same
// record by the same timer. `wake_ns` and `snapshot_sequence` join it to the
// wake's row here; `cand` is the solve order. `reject` is the candidate's
// verdict (NlpRejectName), `worst_group` the row group of the largest
// violation and `viol_<group>` one flag per group whose violation exceeded
// tol_violation — every group that refused, which the verdict's worst group
// does not say. `solve_us` is the wall time of the solve the candidate uses —
// the fixed-grid one, or the continuous one when `continuous` is 1 (a
// deadline cut it, as in planner_events); `sigma_c` is the raw σ at the catch
// node as the solve was given it (NaN: no valid covariance there). `cut_site`
// is `segment_cut_site`'s name for that solve — a candidate cut before its
// initialisation QP has a row with `iterations` and `qp_solves` 0. Nothing in
// the record changes shape per wake: the candidate rows are a fixed array in
// NlpSearchStats.
//
// Readers select columns by NAME: the set has grown and shrunk, and a log
// from before a change lacks the newer names.
//
// The header below is ONE statement of adjacent string literals:
// rtc_tools' test_cpp_header_matches_this_list reads the column list back out
// of this file by joining them.

#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <ostream>

namespace integrated_bringup {

inline void WritePlannerEventsHeader(std::ostream& os) {
  os << "wake_ns,publish_ns,recv_to_publish_ms,outcome,mode,reset_seen,cov_matched,"
        "plan_id,plan_valid,search_valid,plan_reason,snapshot_sequence,track_generation,settling,"
        "n_in_window,n_ik,n_pass,rej_input,rej_ik,rej_manipulability,"
        "rej_not_evaluated,rej_too_far,budget_hit,search_us,ik_us_max,rank_mask,rank_uncertainty,"
        "rank_reach,rank_gamma,rank_commit_lead,rank_error_budget,score,lead_s,gamma_f,"
        "decision,sigma_l,rank_rollout,t_w,rollout_window_only,n_rollouts,rollout_us_max,"
        "g_min,g_max,v_dir_max,max_catchable,segment_outcome,segment_k,segment_n_nodes,segment_seq,"
        "segment_publish_ns,segment_x0_clamped,segment_x0_from_segment,"
        "segment_presolved,segment_cold_retry,segment_iterations,segment_qp_status,segment_core_"
        "reason,"
        "segment_solve_us,segment_slack_max,segment_slack_terminal_max,segment_tau_ratio_max,"
        "segment_kind,segment_cold_start,segment_solver_retried,segment_ref_clamped,segment_ref_"
        "scaled,"
        "segment_ref_scale,segment_ref_shortfall,segment_x0_speed,segment_catch_pos_err,"
        "segment_catch_axis_err,segment_catch_gamma,segment_catch_v_rel,segment_slack_v,"
        "segment_speed_ratio_max,segment_w_p_fallback,segment_w_delta_scale,segment_source_seq,"
        "nlp_ran,nlp_reason,nlp_n_lattice,nlp_n_screened,nlp_n_solved,nlp_n_valid,"
        "nlp_rej_follow_window,nlp_rej_lead_short,nlp_rej_ball_invalid,"
        "nlp_rej_covariance,nlp_rej_no_source,nlp_rej_too_far,nlp_rej_ik,nlp_rej_manipulability,"
        "nlp_rej_reach,"
        "nlp_rej_speed_window,nlp_rej_not_ranked,nlp_rej_deadline,nlp_rej_solver_rejected,"
        "nlp_rej_hard_row,nlp_rej_chance,nlp_rej_unconverged,"
        "nlp_index,nlp_n_pre,nlp_iterations,nlp_source_seq,nlp_x0_clamped,nlp_lead_s,nlp_wait_s,"
        "nlp_phi,nlp_j_reference,nlp_j_stop,nlp_j_time,nlp_j_switch,"
        "nlp_follow_anchor_set,nlp_follow_anchor_index,nlp_cells_from_anchor,nlp_ns_from_first,"
        "nlp_at_window_edge,"
        "nlp_n_continuous_run,nlp_n_continuous,nlp_n_fallback,nlp_continuous,nlp_delta_ns,"
        "nlp_sigma_c_cell,"
        "nlp_screen_us,nlp_solve_us_max,nlp_cmd_gap_q,nlp_cmd_gap_qd,"
        "segment_qp_solves,segment_qp_iterations,segment_backtracks,segment_mu_updates,"
        "segment_cut_site,segment_start_from_memory,"
        "segment_start_us,segment_linearize_us,segment_assemble_us,segment_qp_us,segment_merit_us,"
        "segment_kkt_residual,segment_grad_norm,segment_complementarity,segment_infeasible_group,"
        "segment_viol_torque,segment_viol_gap,segment_viol_entrance,segment_viol_lateral,"
        "segment_viol_timing,segment_viol_velocity_set,segment_viol_impact,segment_viol_box,"
        "segment_viol_terminal,"
        "segment_elastic_torque,segment_elastic_gap,segment_elastic_entrance,"
        "segment_elastic_lateral,segment_elastic_timing,segment_elastic_velocity_set,"
        "segment_elastic_impact,"
        "segment_c_catch,segment_c_guarded,segment_sigma_s,segment_sigma_t,"
        "segment_chance_lateral,segment_chance_timing,segment_cost_reference,segment_cost_stop,"
        "replace_step,replacement_outcome,replacement_core_reason,replacement_iterations,"
        "replacement_solve_us,cov_n,chosen_sigma_c\n";
}

/// nlp_candidates.csv — one row per solved candidate of a wake (header note).
inline void WriteNlpCandidatesHeader(std::ostream& os) {
  os << "wake_ns,snapshot_sequence,cand,index,t_c_ns,reject,worst_group,worst_violation,";
  for (int g = 0; g < rtc::catching::kNumDockingRowGroups; ++g) {
    os << "viol_"
       << rtc::catching::DockingRowGroupName(static_cast<rtc::catching::DockingRowGroup>(g)) << ',';
  }
  os << "continuous,iterations,qp_solves,solve_us,c_catch,sigma_c,phi,cut_site\n";
}

inline void WriteNlpCandidatesRows(std::ostream& os, const rtc::catching::PlannerCycleRecord& r) {
  const rtc::catching::NlpSearchStats& n = r.search.nlp;
  if (!n.ran) {
    return;
  }
  const int count = std::min<int>(n.n_cands, rtc::catching::kNlpCandidateStatCount);
  for (int i = 0; i < count; ++i) {
    const rtc::catching::NlpCandidateStat& c = n.cands[static_cast<std::size_t>(i)];
    os << r.wake_ns << ',' << r.snapshot_sequence << ',' << i << ',' << c.index << ',' << c.t_c_ns
       << ',' << rtc::catching::NlpRejectName(c.reject) << ','
       << rtc::catching::DockingRowGroupName(
              static_cast<rtc::catching::DockingRowGroup>(c.worst_group))
       << ',' << c.worst_violation << ',';
    for (int g = 0; g < rtc::catching::kNumDockingRowGroups; ++g) {
      os << ((c.violated_mask >> static_cast<unsigned>(g)) & 1U) << ',';
    }
    os << (c.continuous_used ? 1 : 0) << ',' << c.iterations << ',' << c.qp_solves << ','
       << c.solve_ns / 1000 << ',' << c.c_catch << ',' << c.sigma_c << ',' << c.phi << ','
       << rtc::catching::MpcDockingCutSiteName(
              static_cast<rtc::catching::MpcDockingCutSite>(c.cut_site))
       << '\n';
  }
}

/// The `nlp_rej_*` columns, in the header's order: every reason a CANDIDATE can
/// carry (search_stats.hpp — NlpReject between kNone and the wake-only ones).
inline constexpr std::array<rtc::catching::NlpReject, 16> kNlpRejectColumns{
    rtc::catching::NlpReject::kFollowWindow,
    rtc::catching::NlpReject::kLeadShort,
    rtc::catching::NlpReject::kBallInvalid,
    rtc::catching::NlpReject::kCovariance,
    rtc::catching::NlpReject::kNoSource,
    rtc::catching::NlpReject::kTooFar,
    rtc::catching::NlpReject::kIk,
    rtc::catching::NlpReject::kManipulability,
    rtc::catching::NlpReject::kReach,
    rtc::catching::NlpReject::kSpeedWindow,
    rtc::catching::NlpReject::kNotRanked,
    rtc::catching::NlpReject::kDeadline,
    rtc::catching::NlpReject::kSolverRejected,
    rtc::catching::NlpReject::kHardRow,
    rtc::catching::NlpReject::kChance,
    rtc::catching::NlpReject::kUnconverged,
};

/// Whether the segment step did something worth a row on its own.
[[nodiscard]] inline bool SegmentStepWorthRecording(rtc::catching::SegmentOutcome o) noexcept {
  using rtc::catching::SegmentOutcome;
  return o != SegmentOutcome::kOff && o != SegmentOutcome::kUpToDate &&
         o != SegmentOutcome::kPastReplanWindow;
}

/// Whether a wake is worth a row (see the file header).
[[nodiscard]] inline bool PlannerEventWorthRecording(
    const rtc::catching::PlannerCycleRecord& r) noexcept {
  return r.outcome != rtc::catching::CycleOutcome::kIdle || r.reset_seen ||
         std::isfinite(r.search.sigma_l) || SegmentStepWorthRecording(r.segment.outcome);
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
     << (r.plan_valid ? 1 : 0) << ',' << (r.search_valid ? 1 : 0) << ','
     << static_cast<int>(r.reason) << ',' << r.snapshot_sequence << ',' << r.track_generation << ','
     << (s.settling ? 1 : 0) << ',' << s.n_in_window << ',' << s.n_ik << ',' << s.n_pass << ','
     << rej(JudgeReject::kInput) << ',' << rej(JudgeReject::kIk) << ','
     << rej(JudgeReject::kManipulability) << ',' << rej(JudgeReject::kNotEvaluated) << ','
     << rej(JudgeReject::kTooFar) << ',' << (s.budget_hit ? 1 : 0) << ',' << s.search_ns / 1000
     << ',' << s.ik_ns_max / 1000 << ',' << s.chosen_rank_mask << ','
     << bit(rtc::catching::kRankUncertainty) << ',' << bit(rtc::catching::kRankReach) << ','
     << bit(rtc::catching::kRankGamma) << ',' << bit(rtc::catching::kRankCommitLead) << ','
     << bit(rtc::catching::kRankErrorBudget) << ',' << s.chosen_score << ',' << s.chosen_lead_s
     << ',' << s.chosen_gamma_f << ',' << rtc::catching::SwitchDecisionName(s.decision) << ','
     << s.sigma_l << ',' << bit(rtc::catching::kRankRollout) << ',' << s.chosen_t_w << ','
     << (s.chosen_rollout_window_only ? 1 : 0) << ',' << s.n_rollouts << ','
     << s.rollout_ns_max / 1000 << ',' << s.chosen_g_min << ',' << s.chosen_g_max << ','
     << s.chosen_v_dir_max << ',' << s.chosen_max_catchable << ',';
  const auto& d = r.segment;
  os << rtc::catching::SegmentOutcomeName(d.outcome) << ',' << d.k << ',' << d.n_nodes << ','
     << d.segment_seq << ',' << d.publish_ns << ',' << (d.x0_clamped ? 1 : 0) << ','
     << (d.x0_from_segment ? 1 : 0) << ',' << (d.presolved ? 1 : 0) << ',' << (d.cold_retry ? 1 : 0)
     << ',' << d.iterations << ',' << d.qp_status << ',' << d.core_reason_name << ','
     << d.solve_ns / 1000 << ',' << d.slack_max << ',' << d.slack_terminal_max << ','
     << d.tau_ratio_max << ',';
  os << rtc::catching::SegmentKindName(d.kind) << ',' << (d.cold_start ? 1 : 0) << ','
     << (d.solver_retried ? 1 : 0) << ',' << (d.ref_clamped ? 1 : 0) << ','
     << (d.ref_scaled ? 1 : 0) << ',' << d.ref_scale << ',' << d.ref_shortfall << ',' << d.x0_speed
     << ',' << d.catch_pos_err << ',' << d.catch_axis_err << ',' << d.catch_gamma << ','
     << d.catch_v_rel << ',' << d.slack_v << ',' << d.speed_ratio_max << ','
     << (d.w_p_fallback ? 1 : 0) << ',' << d.w_delta_scale << ',' << d.source_seq << ',';
  // A value the wake did not compute is NaN (file header).
  const auto num = [&os](bool have, double v) {
    if (have) {
      os << v;
    } else {
      os << "nan";
    }
    os << ',';
  };
  // An integer the wake did not compute is nan as well; one it did is written
  // as an integer (through `num` it would be rounded to six digits).
  const auto whole = [&os](bool have, std::int64_t v) {
    if (have) {
      os << v;
    } else {
      os << "nan";
    }
    os << ',';
  };
  // ── The NLP search's account ──
  const rtc::catching::NlpSearchStats& n = s.nlp;
  // The chosen candidate's fields mean something only on a wake that chose one.
  const bool chose = n.ran && n.reason == rtc::catching::NlpReject::kNone;
  os << (n.ran ? 1 : 0) << ',' << (n.ran ? rtc::catching::NlpRejectName(n.reason) : "off") << ','
     << n.n_lattice << ',' << n.n_screened << ',' << n.n_solved << ',' << n.n_valid << ',';
  for (const rtc::catching::NlpReject why : kNlpRejectColumns) {
    os << n.rejects[static_cast<std::size_t>(why)] << ',';
  }
  whole(chose, n.chosen_index);
  whole(chose, n.chosen_n_pre);
  whole(chose, n.chosen_iterations);
  whole(chose, n.chosen_source_seq);
  os << (chose && n.chosen_x0_clamped ? 1 : 0) << ',';
  num(chose, n.chosen_lead_s);
  num(chose, n.chosen_wait_s);
  num(chose, n.chosen_phi);
  num(chose, n.chosen_j_reference);
  num(chose, n.chosen_j_stop);
  num(chose, n.chosen_j_time);
  num(chose, n.chosen_j_switch);
  os << (n.follow_anchor_set ? 1 : 0) << ',';
  whole(n.follow_anchor_set, n.follow_anchor_index);
  whole(chose && n.follow_anchor_set, n.chosen_cells_from_anchor);
  whole(chose && n.follow_anchor_set, n.chosen_ns_from_first);
  os << (chose && n.chosen_at_window_edge ? 1 : 0) << ',' << n.n_continuous_run << ','
     << n.n_continuous << ',' << n.n_fallback << ',' << (chose && n.chosen_continuous ? 1 : 0)
     << ',';
  whole(chose, n.chosen_delta_ns);
  num(chose, n.chosen_sigma_c_cell);
  whole(n.ran, n.screen_ns / 1000);
  whole(n.ran, n.solve_ns_max / 1000);
  num(n.ran, n.cmd_gap_q);
  num(n.ran, n.cmd_gap_qd);
  // ── The mpc_docking core's account of the solve ──
  const rtc::catching::DockingSolveStats& k = d.docking;
  os << k.qp_solves << ',' << k.qp_iterations << ',' << k.backtracks << ',' << k.mu_updates << ','
     << k.cut_site_name << ',' << (d.start_from_memory ? 1 : 0) << ',';
  num(k.ran, k.start_us);
  num(k.ran, k.linearize_us);
  num(k.ran, k.assemble_us);
  num(k.ran, k.qp_us);
  num(k.ran, k.merit_us);
  num(k.ran, k.kkt_residual);
  num(k.ran, k.grad_norm);
  num(k.ran, k.complementarity);
  os << k.infeasible_group_name << ',';
  for (const double v : k.violation) {
    num(k.ran, v);
  }
  for (const double v : k.elastic) {
    num(k.ran, v);
  }
  num(k.ran, k.c_catch);
  os << (k.c_guarded ? 1 : 0) << ',';
  num(k.ran, k.sigma_s);
  num(k.ran, k.sigma_t);
  num(k.ran, k.lateral_margin);
  num(k.ran, k.timing_margin);
  num(k.ran, k.cost_reference);
  num(k.ran, k.cost_stop);
  // ── A replacement, and the first solve of one that was withheld ──
  const auto& w = r.replacement;
  os << rtc::catching::ReplaceStepName(r.replace_step) << ','
     << rtc::catching::SegmentOutcomeName(w.outcome) << ',' << w.core_reason_name << ','
     << w.iterations << ',' << w.solve_ns / 1000 << ',' << s.cov_n << ',' << s.chosen_sigma_c
     << '\n';
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_PLANNER_EVENTS_CSV_HPP_
