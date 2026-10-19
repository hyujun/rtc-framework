// ── planner_events.csv and nlp_candidates.csv, every column by name (#886) ──
//
// test_catching_planner_lane.cpp's PlannerEventsCsv.* tests pin the width and
// a set of columns by name. The tests here give every column of both writers a
// value no other column of the row has and assert all of them by name
// (log_csv_asserts.hpp), so two row statements that trade places read a wrong
// cell whichever columns they are. Then:
//
//   - each of the six conditions a column is written under (a publish with a
//     real receive, n.ran, a chosen candidate, a follow anchor, both of the
//     last two, the docking core having run) is broken alone, and exactly the
//     columns it gates turn to nan — a column gated on the wrong condition
//     shows;
//   - every bool column is walked on alone;
//   - a candidate row is asserted the same way, and each viol_<group> flag is
//     walked on alone.
//
// Both writers end a row with '\n' (the device POD writers leave it to the
// logger): FirstLine strips it before the row reaches the assertions.

#include "integrated_bringup/logging/planner_events_csv.hpp"
#include "log_csv_asserts.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using integrated_bringup::testfx::BoolCase;
using integrated_bringup::testfx::BoolsAreOneHot;
using integrated_bringup::testfx::CellBuilder;
using integrated_bringup::testfx::Cells;
using integrated_bringup::testfx::RowMatchesByName;
using rtc::catching::CycleOutcome;
using rtc::catching::JudgeReject;
using rtc::catching::NlpCandidateStat;
using rtc::catching::NlpReject;
using rtc::catching::PlannerCycleRecord;

std::vector<std::string> Lines(const std::string& text) {
  std::vector<std::string> out;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    out.push_back(line);
  }
  return out;
}

std::string FirstLine(const std::string& text) {
  return text.substr(0, text.find('\n'));
}

std::string PlannerHeader() {
  std::ostringstream os;
  integrated_bringup::WritePlannerEventsHeader(os);
  EXPECT_EQ(os.str().back(), '\n');
  return FirstLine(os.str());
}

std::string PlannerRow(const PlannerCycleRecord& r) {
  std::ostringstream os;
  integrated_bringup::WritePlannerEventsRow(os, r);
  EXPECT_EQ(os.str().back(), '\n');
  EXPECT_EQ(Lines(os.str()).size(), 1U);
  return FirstLine(os.str());
}

std::string CandidatesHeader() {
  std::ostringstream os;
  integrated_bringup::WriteNlpCandidatesHeader(os);
  return FirstLine(os.str());
}

std::vector<std::string> CandidateRows(const PlannerCycleRecord& r) {
  std::ostringstream os;
  integrated_bringup::WriteNlpCandidatesRows(os, r);
  return Lines(os.str());
}

// Replace the expected cell of `name` (it must already be expected).
void Expect(Cells& cells, const std::string& name, const std::string& value) {
  for (auto& [n, v] : cells) {
    if (n == name) {
      v = value;
      return;
    }
  }
  ADD_FAILURE() << "'" << name << "' is not an expected column";
}

// Column names written literally, not derived from the writer's own tables
// (kNlpRejectColumns, DockingRowGroupName): a test that builds its names the
// way the writer does cannot see the writer's table being wrong.
const std::array<const char*, 9> kViolGroups{
    "torque", "gap", "entrance", "lateral", "timing", "velocity_set", "impact", "box", "terminal"};
const std::array<const char*, 7> kElasticGroups{"torque", "gap",          "entrance", "lateral",
                                                "timing", "velocity_set", "impact"};
const std::array<std::pair<const char*, NlpReject>, 16> kNlpRejects{{
    {"nlp_rej_follow_window", NlpReject::kFollowWindow},
    {"nlp_rej_lead_short", NlpReject::kLeadShort},
    {"nlp_rej_ball_invalid", NlpReject::kBallInvalid},
    {"nlp_rej_covariance", NlpReject::kCovariance},
    {"nlp_rej_no_source", NlpReject::kNoSource},
    {"nlp_rej_too_far", NlpReject::kTooFar},
    {"nlp_rej_ik", NlpReject::kIk},
    {"nlp_rej_manipulability", NlpReject::kManipulability},
    {"nlp_rej_reach", NlpReject::kReach},
    {"nlp_rej_speed_window", NlpReject::kSpeedWindow},
    {"nlp_rej_not_ranked", NlpReject::kNotRanked},
    {"nlp_rej_deadline", NlpReject::kDeadline},
    {"nlp_rej_solver_rejected", NlpReject::kSolverRejected},
    {"nlp_rej_hard_row", NlpReject::kHardRow},
    {"nlp_rej_chance", NlpReject::kChance},
    {"nlp_rej_unconverged", NlpReject::kUnconverged},
}};

// Every bool column of planner_events.csv.
const std::vector<std::string> kPlannerBools{
    "reset_seen",
    "cov_matched",
    "plan_valid",
    "search_valid",
    "settling",
    "budget_hit",
    "rank_uncertainty",
    "rank_reach",
    "rank_gamma",
    "rank_commit_lead",
    "rank_error_budget",
    "rank_rollout",
    "rollout_window_only",
    "segment_x0_clamped",
    "segment_x0_from_segment",
    "segment_presolved",
    "segment_cold_retry",
    "segment_cold_start",
    "segment_solver_retried",
    "segment_ref_clamped",
    "segment_ref_scaled",
    "segment_w_p_fallback",
    "nlp_ran",
    "nlp_x0_clamped",
    "nlp_follow_anchor_set",
    "nlp_at_window_edge",
    "nlp_continuous",
    "segment_c_guarded",
    "segment_start_from_memory",
    "segment_start_from_search",
};

// A record in which every conditional column was computed, and the cells its
// row must print. Integers come from 300 up (the small enums and the rank mask
// are set apart below 300), reals from CellBuilder's x.25 / x.75 sequence, and
// a nanosecond field written in µs holds 999 ns past its µs value so a
// rounding division shows.
struct StrictRecord {
  PlannerCycleRecord rec{};
  Cells want;
};

StrictRecord MakeStrictRecord() {
  StrictRecord out;
  PlannerCycleRecord& r = out.rec;
  CellBuilder b;
  long long next_int = 300;
  const auto Int = [&](const std::string& name, auto& field) { b.Int(name, field, next_int++); };
  const auto Us = [&](const std::string& name, std::int64_t& ns_field) {
    const long long us = next_int++;
    ns_field = us * 1000 + 999;
    b.want.emplace_back(name, std::to_string(us));
  };
  const auto Flag = [&](const std::string& name, bool& field, bool value) {
    field = value;
    b.want.emplace_back(name, value ? "1" : "0");
  };
  const auto Name = [&](const std::string& name, const std::string& value) {
    b.want.emplace_back(name, value);
  };

  // ── The wake ──
  Int("wake_ns", r.wake_ns);
  r.outcome = CycleOutcome::kPublished;
  r.traj_recv_ns = 1'000'000;
  r.publish_ns = 4'250'000;  // 3.25 ms after the receive
  Name("publish_ns", "4250000");
  Name("recv_to_publish_ms", "3.25");
  Name("outcome", "published");
  r.mode = 200;
  Name("mode", "200");
  Flag("reset_seen", r.reset_seen, false);
  Flag("cov_matched", r.cov_matched, false);
  Int("plan_id", r.plan_id);
  Flag("plan_valid", r.plan_valid, false);
  Flag("search_valid", r.search_valid, false);
  r.reason = rtc::catching::PlanReason::kImpulse;
  Name("plan_reason", "10");
  Int("snapshot_sequence", r.snapshot_sequence);
  Int("track_generation", r.track_generation);

  // ── The grid search ──
  auto& s = r.search;
  Flag("settling", s.settling, false);
  Int("n_in_window", s.n_in_window);
  Int("n_ik", s.n_ik);
  Int("n_pass", s.n_pass);
  const auto judge = [&](JudgeReject j) -> std::uint16_t& {
    return s.judge_rejects[static_cast<std::size_t>(j)];
  };
  Int("rej_input", judge(JudgeReject::kInput));
  Int("rej_ik", judge(JudgeReject::kIk));
  Int("rej_manipulability", judge(JudgeReject::kManipulability));
  Int("rej_not_evaluated", judge(JudgeReject::kNotEvaluated));
  Int("rej_too_far", judge(JudgeReject::kTooFar));
  Flag("budget_hit", s.budget_hit, false);
  Us("search_us", s.search_ns);
  Us("ik_us_max", s.ik_ns_max);
  // reach | commit_lead | rollout: three bits on, three off, so a bit column
  // that reads the wrong bit flips.
  s.chosen_rank_mask =
      rtc::catching::kRankReach | rtc::catching::kRankCommitLead | rtc::catching::kRankRollout;
  Name("rank_mask", "42");
  Name("rank_uncertainty", "0");
  Name("rank_reach", "1");
  Name("rank_gamma", "0");
  Name("rank_commit_lead", "1");
  Name("rank_error_budget", "0");
  Name("rank_rollout", "1");
  b.Real("score", s.chosen_score);
  b.Real("lead_s", s.chosen_lead_s);
  b.Real("gamma_f", s.chosen_gamma_f);
  s.decision = rtc::catching::SwitchDecision::kHeldJump;
  Name("decision", "held_jump");
  b.Real("sigma_l", s.sigma_l);
  b.Real("t_w", s.chosen_t_w);
  Flag("rollout_window_only", s.chosen_rollout_window_only, false);
  Int("n_rollouts", s.n_rollouts);
  Us("rollout_us_max", s.rollout_ns_max);
  b.Real("g_min", s.chosen_g_min);
  b.Real("g_max", s.chosen_g_max);
  b.Real("v_dir_max", s.chosen_v_dir_max);
  b.Real("max_catchable", s.chosen_max_catchable);
  Int("cov_n", s.cov_n);
  b.Real("chosen_sigma_c", s.chosen_sigma_c);

  // ── The wake's own segment solve ──
  auto& d = r.segment;
  d.outcome = rtc::catching::SegmentOutcome::kSlack;
  Name("segment_outcome", "slack");
  d.k = -307;
  Name("segment_k", "-307");
  Int("segment_n_nodes", d.n_nodes);
  Int("segment_seq", d.segment_seq);
  Int("segment_publish_ns", d.publish_ns);
  Flag("segment_x0_clamped", d.x0_clamped, false);
  Flag("segment_x0_from_segment", d.x0_from_segment, false);
  Flag("segment_presolved", d.presolved, false);
  Flag("segment_cold_retry", d.cold_retry, false);
  Int("segment_iterations", d.iterations);
  Int("segment_qp_status", d.qp_status);
  d.core_reason_name = "deadline";
  Name("segment_core_reason", "deadline");
  Us("segment_solve_us", d.solve_ns);
  b.Real("segment_slack_max", d.slack_max);
  b.Real("segment_slack_terminal_max", d.slack_terminal_max);
  b.Real("segment_tau_ratio_max", d.tau_ratio_max);
  d.kind = rtc::catching::SegmentKind::kStop;
  Name("segment_kind", "stop");
  Flag("segment_cold_start", d.cold_start, false);
  Flag("segment_solver_retried", d.solver_retried, false);
  Flag("segment_ref_clamped", d.ref_clamped, false);
  Flag("segment_ref_scaled", d.ref_scaled, false);
  b.Real("segment_ref_scale", d.ref_scale);
  b.Real("segment_ref_shortfall", d.ref_shortfall);
  b.Real("segment_x0_speed", d.x0_speed);
  b.Real("segment_catch_pos_err", d.catch_pos_err);
  b.Real("segment_catch_axis_err", d.catch_axis_err);
  b.Real("segment_catch_gamma", d.catch_gamma);
  b.Real("segment_catch_v_rel", d.catch_v_rel);
  b.Real("segment_slack_v", d.slack_v);
  b.Real("segment_speed_ratio_max", d.speed_ratio_max);
  Flag("segment_w_p_fallback", d.w_p_fallback, false);
  b.Real("segment_w_delta_scale", d.w_delta_scale);
  Int("segment_source_seq", d.source_seq);
  Flag("segment_start_from_memory", d.start_from_memory, false);
  Flag("segment_start_from_search", d.start_from_search, false);
  const std::array<const char*, 3> axes{"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    b.Real(std::string("segment_stop_line_p_") + axes[i], d.stop_line_p[i]);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    b.Real(std::string("segment_stop_line_d_") + axes[i], d.stop_line_d[i]);
  }
  b.Real("segment_stop_dev_max", d.stop_dev_max);
  b.Real("segment_stop_dev_end", d.stop_dev_end);
  b.Real("segment_stop_along_end", d.stop_along_end);
  b.Real("segment_mpc_presolve_us", d.mpc_presolve_us);
  b.Real("segment_mpc_linearize_us", d.mpc_linearize_us);
  b.Real("segment_mpc_condense_us", d.mpc_condense_us);
  b.Real("segment_mpc_qp_us", d.mpc_qp_us);
  Int("segment_path_window", d.path_window);
  Int("segment_path_nodes", d.path_nodes);
  b.Real("segment_path_dev_max", d.path_dev_max);
  b.Real("segment_path_dev_mean", d.path_dev_mean);

  // ── The docking core's account of that solve ──
  auto& k = d.docking;
  k.ran = true;
  Int("segment_qp_solves", k.qp_solves);
  Int("segment_qp_iterations", k.qp_iterations);
  Int("segment_backtracks", k.backtracks);
  Int("segment_mu_updates", k.mu_updates);
  k.cut_site_name = "penalty_probe";
  Name("segment_cut_site", "penalty_probe");
  b.Real("segment_start_us", k.start_us);
  b.Real("segment_linearize_us", k.linearize_us);
  b.Real("segment_assemble_us", k.assemble_us);
  b.Real("segment_qp_us", k.qp_us);
  b.Real("segment_merit_us", k.merit_us);
  b.Real("segment_kkt_residual", k.kkt_residual);
  b.Real("segment_grad_norm", k.grad_norm);
  b.Real("segment_complementarity", k.complementarity);
  k.infeasible_group_name = "lateral";
  Name("segment_infeasible_group", "lateral");
  for (std::size_t g = 0; g < kViolGroups.size(); ++g) {
    b.Real(std::string("segment_viol_") + kViolGroups[g], k.violation[g]);
  }
  for (std::size_t g = 0; g < kElasticGroups.size(); ++g) {
    b.Real(std::string("segment_elastic_") + kElasticGroups[g], k.elastic[g]);
  }
  b.Real("segment_c_catch", k.c_catch);
  Flag("segment_c_guarded", k.c_guarded, false);
  b.Real("segment_sigma_s", k.sigma_s);
  b.Real("segment_sigma_t", k.sigma_t);
  b.Real("segment_chance_lateral", k.lateral_margin);
  b.Real("segment_chance_timing", k.timing_margin);
  b.Real("segment_cost_reference", k.cost_reference);
  b.Real("segment_cost_stop", k.cost_stop);

  // ── A withheld replacement ──
  r.replace_step = rtc::catching::ReplaceStep::kWithheld;
  Name("replace_step", "withheld");
  auto& w = r.replacement;
  w.outcome = rtc::catching::SegmentOutcome::kBudget;
  Name("replacement_outcome", "budget");
  w.core_reason_name = "max_iterations";
  Name("replacement_core_reason", "max_iterations");
  Int("replacement_iterations", w.iterations);
  Us("replacement_solve_us", w.solve_ns);

  // ── The NLP search, which chose a candidate and has a follow anchor ──
  auto& n = s.nlp;
  Flag("nlp_ran", n.ran, true);
  n.reason = NlpReject::kNone;
  Name("nlp_reason", "none");
  Int("nlp_n_lattice", n.n_lattice);
  Int("nlp_n_screened", n.n_screened);
  Int("nlp_n_solved", n.n_solved);
  Int("nlp_n_valid", n.n_valid);
  for (const auto& [name, why] : kNlpRejects) {
    Int(name, n.rejects[static_cast<std::size_t>(why)]);
  }
  Int("nlp_index", n.chosen_index);
  Int("nlp_n_pre", n.chosen_n_pre);
  Int("nlp_iterations", n.chosen_iterations);
  Int("nlp_source_seq", n.chosen_source_seq);
  Flag("nlp_x0_clamped", n.chosen_x0_clamped, true);
  b.Real("nlp_lead_s", n.chosen_lead_s);
  b.Real("nlp_wait_s", n.chosen_wait_s);
  b.Real("nlp_phi", n.chosen_phi);
  b.Real("nlp_j_reference", n.chosen_j_reference);
  b.Real("nlp_j_stop", n.chosen_j_stop);
  b.Real("nlp_j_time", n.chosen_j_time);
  b.Real("nlp_j_switch", n.chosen_j_switch);
  Flag("nlp_follow_anchor_set", n.follow_anchor_set, true);
  Int("nlp_follow_anchor_index", n.follow_anchor_index);
  Int("nlp_cells_from_anchor", n.chosen_cells_from_anchor);
  Int("nlp_ns_from_first", n.chosen_ns_from_first);
  Flag("nlp_at_window_edge", n.chosen_at_window_edge, true);
  Int("nlp_n_continuous_run", n.n_continuous_run);
  Int("nlp_n_continuous", n.n_continuous);
  Int("nlp_n_fallback", n.n_fallback);
  Flag("nlp_continuous", n.chosen_continuous, true);
  Int("nlp_delta_ns", n.chosen_delta_ns);
  b.Real("nlp_sigma_c_cell", n.chosen_sigma_c_cell);
  Us("nlp_screen_us", n.screen_ns);
  Us("nlp_solve_us_max", n.solve_ns_max);
  b.Real("nlp_cmd_gap_q", n.cmd_gap_q);
  b.Real("nlp_cmd_gap_qd", n.cmd_gap_qd);

  out.want = std::move(b.want);
  return out;
}

const std::vector<std::string> kChoseColumns{
    "nlp_index",    "nlp_n_pre",    "nlp_iterations",  "nlp_source_seq", "nlp_lead_s",
    "nlp_wait_s",   "nlp_phi",      "nlp_j_reference", "nlp_j_stop",     "nlp_j_time",
    "nlp_j_switch", "nlp_delta_ns", "nlp_sigma_c_cell"};
const std::vector<std::string> kChoseFlags{"nlp_x0_clamped", "nlp_at_window_edge",
                                           "nlp_continuous"};
const std::vector<std::string> kChoseAndAnchorColumns{"nlp_cells_from_anchor", "nlp_ns_from_first"};
const std::vector<std::string> kRanColumns{"nlp_screen_us", "nlp_solve_us_max", "nlp_cmd_gap_q",
                                           "nlp_cmd_gap_qd"};

std::vector<std::string> DockingColumns() {
  std::vector<std::string> out{
      "segment_start_us",      "segment_linearize_us",    "segment_assemble_us",
      "segment_qp_us",         "segment_merit_us",        "segment_kkt_residual",
      "segment_grad_norm",     "segment_complementarity", "segment_c_catch",
      "segment_sigma_s",       "segment_sigma_t",         "segment_chance_lateral",
      "segment_chance_timing", "segment_cost_reference",  "segment_cost_stop"};
  for (const char* g : kViolGroups) {
    out.push_back(std::string("segment_viol_") + g);
  }
  for (const char* g : kElasticGroups) {
    out.push_back(std::string("segment_elastic_") + g);
  }
  return out;
}

TEST(PlannerEvents, EveryColumnByNameWhenEverythingWasComputed) {
  const auto strict = MakeStrictRecord();
  const std::string hdr = PlannerHeader();
  EXPECT_EQ(integrated_bringup::testfx::SplitCsv(hdr).size(), 191U);
  EXPECT_TRUE(RowMatchesByName(hdr, PlannerRow(strict.rec), strict.want, kPlannerBools,
                               /*strict=*/true));
}

// Break one condition at a time: the columns it gates turn to nan (its flags
// to 0), and every other column keeps the strict record's value.
TEST(PlannerEvents, EachConditionLeavesExactlyItsColumnsUncomputed) {
  const std::string hdr = PlannerHeader();

  struct Case {
    std::string label;
    std::function<void(PlannerCycleRecord&)> breaks;
    std::vector<std::pair<std::string, std::string>> cells;
  };

  const auto nan_all = [](std::vector<std::pair<std::string, std::string>>& cells,
                          const std::vector<std::string>& names, const std::string& v) {
    for (const auto& name : names) {
      cells.emplace_back(name, v);
    }
  };
  std::vector<Case> cases;
  {  // Latency: not a publish.
    Case c{"held", [](PlannerCycleRecord& r) { r.outcome = CycleOutcome::kHeld; }, {}};
    c.cells = {{"outcome", "held"}, {"recv_to_publish_ms", "nan"}};
    cases.push_back(c);
  }
  {  // Latency: no receive to measure from.
    Case c{"no receive", [](PlannerCycleRecord& r) { r.traj_recv_ns = 0; }, {}};
    c.cells = {{"recv_to_publish_ms", "nan"}};
    cases.push_back(c);
  }
  {  // n.ran: no NLP search ran — nothing it would have computed is written,
     // but the follow anchor (not the search's) still is.
    Case c{"nlp off", [](PlannerCycleRecord& r) { r.search.nlp.ran = false; }, {}};
    c.cells = {{"nlp_ran", "0"}, {"nlp_reason", "off"}};
    nan_all(c.cells, kChoseColumns, "nan");
    nan_all(c.cells, kChoseFlags, "0");
    nan_all(c.cells, kChoseAndAnchorColumns, "nan");
    nan_all(c.cells, kRanColumns, "nan");
    cases.push_back(c);
  }
  {  // chose: the search ran and chose none.
    Case c{"none chosen",
           [](PlannerCycleRecord& r) { r.search.nlp.reason = NlpReject::kHardRow; },
           {}};
    c.cells = {{"nlp_reason", "hard_row"}};
    nan_all(c.cells, kChoseColumns, "nan");
    nan_all(c.cells, kChoseFlags, "0");
    nan_all(c.cells, kChoseAndAnchorColumns, "nan");
    cases.push_back(c);
  }
  {  // follow_anchor_set, and with it chose && follow_anchor_set.
    Case c{"no anchor", [](PlannerCycleRecord& r) { r.search.nlp.follow_anchor_set = false; }, {}};
    c.cells = {{"nlp_follow_anchor_set", "0"}, {"nlp_follow_anchor_index", "nan"}};
    nan_all(c.cells, kChoseAndAnchorColumns, "nan");
    cases.push_back(c);
  }
  {  // k.ran: the docking core reached no iterate.
    Case c{"no docking iterate", [](PlannerCycleRecord& r) { r.segment.docking.ran = false; }, {}};
    nan_all(c.cells, DockingColumns(), "nan");
    cases.push_back(c);
  }
  for (const auto& c : cases) {
    auto strict = MakeStrictRecord();
    c.breaks(strict.rec);
    Cells want = strict.want;
    for (const auto& [name, value] : c.cells) {
      Expect(want, name, value);
    }
    EXPECT_TRUE(RowMatchesByName(hdr, PlannerRow(strict.rec), want, kPlannerBools,
                                 /*strict=*/false))
        << c.label;
  }
}

TEST(PlannerEvents, BoolColumnsAreOneHot) {
  // Base: every bool off — no NLP search, no anchor, no rank bit.
  PlannerCycleRecord base{};
  using Rec = PlannerCycleRecord;
  const auto bit = [](std::uint16_t b) { return [b](Rec& r) { r.search.chosen_rank_mask = b; }; };
  const std::vector<BoolCase<Rec>> cases{
      {{"reset_seen"}, [](Rec& r) { r.reset_seen = true; }},
      {{"cov_matched"}, [](Rec& r) { r.cov_matched = true; }},
      {{"plan_valid"}, [](Rec& r) { r.plan_valid = true; }},
      {{"search_valid"}, [](Rec& r) { r.search_valid = true; }},
      {{"settling"}, [](Rec& r) { r.search.settling = true; }},
      {{"budget_hit"}, [](Rec& r) { r.search.budget_hit = true; }},
      {{"rank_uncertainty"}, bit(rtc::catching::kRankUncertainty)},
      {{"rank_reach"}, bit(rtc::catching::kRankReach)},
      {{"rank_gamma"}, bit(rtc::catching::kRankGamma)},
      {{"rank_commit_lead"}, bit(rtc::catching::kRankCommitLead)},
      {{"rank_error_budget"}, bit(rtc::catching::kRankErrorBudget)},
      {{"rank_rollout"}, bit(rtc::catching::kRankRollout)},
      {{"rollout_window_only"}, [](Rec& r) { r.search.chosen_rollout_window_only = true; }},
      {{"segment_x0_clamped"}, [](Rec& r) { r.segment.x0_clamped = true; }},
      {{"segment_x0_from_segment"}, [](Rec& r) { r.segment.x0_from_segment = true; }},
      {{"segment_presolved"}, [](Rec& r) { r.segment.presolved = true; }},
      {{"segment_cold_retry"}, [](Rec& r) { r.segment.cold_retry = true; }},
      {{"segment_cold_start"}, [](Rec& r) { r.segment.cold_start = true; }},
      {{"segment_solver_retried"}, [](Rec& r) { r.segment.solver_retried = true; }},
      {{"segment_ref_clamped"}, [](Rec& r) { r.segment.ref_clamped = true; }},
      {{"segment_ref_scaled"}, [](Rec& r) { r.segment.ref_scaled = true; }},
      {{"segment_w_p_fallback"}, [](Rec& r) { r.segment.w_p_fallback = true; }},
      {{"segment_c_guarded"}, [](Rec& r) { r.segment.docking.c_guarded = true; }},
      {{"segment_start_from_memory"}, [](Rec& r) { r.segment.start_from_memory = true; }},
      {{"segment_start_from_search"}, [](Rec& r) { r.segment.start_from_search = true; }},
      {{"nlp_follow_anchor_set"}, [](Rec& r) { r.search.nlp.follow_anchor_set = true; }},
      // The chosen candidate's flags are written only beside nlp_ran: a search
      // that ran and chose (reason none) lights nlp_ran alone, and each flag on
      // top of it lights that flag as well.
      {{"nlp_ran"}, [](Rec& r) { r.search.nlp.ran = true; }},
      {{"nlp_ran", "nlp_x0_clamped"},
       [](Rec& r) {
         r.search.nlp.ran = true;
         r.search.nlp.chosen_x0_clamped = true;
       }},
      {{"nlp_ran", "nlp_at_window_edge"},
       [](Rec& r) {
         r.search.nlp.ran = true;
         r.search.nlp.chosen_at_window_edge = true;
       }},
      {{"nlp_ran", "nlp_continuous"},
       [](Rec& r) {
         r.search.nlp.ran = true;
         r.search.nlp.chosen_continuous = true;
       }},
  };
  EXPECT_TRUE(BoolsAreOneHot(PlannerHeader(), base, PlannerRow, kPlannerBools, cases));
}

// ── nlp_candidates.csv ──

const std::vector<std::string>& CandidateBools() {
  static const std::vector<std::string> bools = [] {
    std::vector<std::string> out;
    for (const char* g : kViolGroups) {
      out.push_back(std::string("viol_") + g);
    }
    out.emplace_back("continuous");
    return out;
  }();
  return bools;
}

TEST(NlpCandidates, EveryColumnByNamePerRow) {
  PlannerCycleRecord r{};
  r.wake_ns = 300;
  r.snapshot_sequence = 301;
  auto& n = r.search.nlp;
  n.ran = true;
  n.n_cands = 3;
  const std::array<NlpReject, 3> rejects{NlpReject::kHardRow, NlpReject::kChance, NlpReject::kNone};
  const std::array<rtc::catching::MpcDockingCutSite, 3> sites{
      rtc::catching::MpcDockingCutSite::kIteration, rtc::catching::MpcDockingCutSite::kInitQp,
      rtc::catching::MpcDockingCutSite::kPenaltyReset};
  const std::array<const char*, 3> site_names{"iteration", "init_qp", "penalty_reset"};
  const std::array<const char*, 3> reject_names{"hard_row", "chance", "none"};
  std::vector<Cells> want(3);
  for (std::size_t i = 0; i < 3; ++i) {
    NlpCandidateStat& c = n.cands[i];
    CellBuilder b;
    b.next = 100.25 + 10.0 * static_cast<double>(i);
    long long next_int = 400 + 20 * static_cast<long long>(i);
    b.want.emplace_back("wake_ns", "300");
    b.want.emplace_back("snapshot_sequence", "301");
    b.want.emplace_back("cand", std::to_string(i));
    b.Int("index", c.index, next_int++);
    b.Int("t_c_ns", c.t_c_ns, next_int++);
    c.reject = rejects[i];
    b.want.emplace_back("reject", reject_names[i]);
    // Worst group by name, every group's flag off but the worst one's — which
    // BoolsAreOneHot below walks group by group.
    c.worst_group = static_cast<std::uint8_t>(i + 2);  // entrance, lateral, timing
    b.want.emplace_back("worst_group", kViolGroups[i + 2]);
    b.Real("worst_violation", c.worst_violation);
    c.violated_mask = static_cast<std::uint16_t>(1U << (i + 2));
    for (std::size_t g = 0; g < kViolGroups.size(); ++g) {
      b.want.emplace_back(std::string("viol_") + kViolGroups[g], g == i + 2 ? "1" : "0");
    }
    c.continuous_used = (i == 1);
    b.want.emplace_back("continuous", i == 1 ? "1" : "0");
    b.Int("iterations", c.iterations, next_int++);
    b.Int("qp_solves", c.qp_solves, next_int++);
    const long long us = next_int++;
    c.solve_ns = us * 1000 + 999;
    b.want.emplace_back("solve_us", std::to_string(us));
    b.Real("c_catch", c.c_catch);
    b.Real("sigma_c", c.sigma_c);
    b.Real("phi", c.phi);
    c.cut_site = static_cast<std::uint8_t>(sites[i]);
    b.want.emplace_back("cut_site", site_names[i]);
    want[i] = std::move(b.want);
  }

  const std::string hdr = CandidatesHeader();
  const auto rows = CandidateRows(r);
  ASSERT_EQ(rows.size(), 3U);
  for (std::size_t i = 0; i < rows.size(); ++i) {
    // `cand` of the first two rows prints 0 and 1: asserted, but exempt from
    // the strict rule that no non-bool reads like a bool.
    auto exempt = CandidateBools();
    if (i < 2) {
      exempt.emplace_back("cand");
    }
    EXPECT_TRUE(RowMatchesByName(hdr, rows[i], want[i], exempt, /*strict=*/true)) << "row " << i;
  }

  // No NLP search on the wake: no candidate rows, whatever the array holds.
  n.ran = false;
  EXPECT_TRUE(CandidateRows(r).empty());
}

TEST(NlpCandidates, ViolationFlagsAreOneHot) {
  const auto row = [](const NlpCandidateStat& c) {
    PlannerCycleRecord r{};
    r.search.nlp.ran = true;
    r.search.nlp.n_cands = 1;
    r.search.nlp.cands[0] = c;
    const auto rows = CandidateRows(r);
    EXPECT_EQ(rows.size(), 1U);
    return rows.empty() ? std::string{} : rows.front();
  };
  std::vector<BoolCase<NlpCandidateStat>> cases;
  for (std::size_t g = 0; g < kViolGroups.size(); ++g) {
    cases.push_back({{std::string("viol_") + kViolGroups[g]}, [g](NlpCandidateStat& c) {
                       c.violated_mask = static_cast<std::uint16_t>(1U << g);
                     }});
  }
  cases.push_back({{"continuous"}, [](NlpCandidateStat& c) { c.continuous_used = true; }});
  EXPECT_TRUE(BoolsAreOneHot(CandidatesHeader(), NlpCandidateStat{}, row, CandidateBools(), cases));
}

}  // namespace
