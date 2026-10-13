// ── The offline batch of the planner's search (L3 §4.1) ──────────────────────
//
//   1. A row is the direct call's result: the same wake handed to
//      `CatchSearch::Plan` by hand gives the row the batch wrote, cell for cell.
//   2. A throw's rows do not depend on which throws ran before it.
//   3. The wakes of a throw stop at the first plan.
//   4. The CSV carries the plan's doubles bit-exactly, and a wake without a
//      plan leaves the plan cells empty.
//   5. What a wake is handed: an arm at rest that follows no plan, a zero
//      covariance of the prediction, a report as old as the wake.
//   6. The wake CSV and the binding file are fail-closed.
//   7. A whole-wake batch (a binding with `segment`) is `PlannerCycle::Run`:
//      its search cells are the search-only batch's, a throw stops at its
//      first published pair, a withheld pair runs every wake and leaves the
//      plan cells empty, and a throw's rows do not depend on the throws
//      before it.
//
// Drives the grid search on the 6R wrist arm, built through the binding the
// executable uses; the whole-wake batch drives a 6-joint arm with inertias
// (the mpc segment planner solves its dynamics).
#include "rtc_controllers/catching/catch_search_batch.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rtc::catching::CycleOutcome;
using rtc::catching::kCap;
using rtc::catching::MakePlanBatch;
using rtc::catching::MakeSearchBatchInputs;
using rtc::catching::MakeSearchBatchSearch;
using rtc::catching::NowReal;
using rtc::catching::ParseSearchBatchBinding;
using rtc::catching::ParseSearchWakeCsv;
using rtc::catching::PlanBatchBuild;
using rtc::catching::PlanBatchCsvHeader;
using rtc::catching::PlanBatchCsvRow;
using rtc::catching::ReachBound;
using rtc::catching::ReachBoundJson;
using rtc::catching::ReportedSegments;
using rtc::catching::RunSearchBatch;
using rtc::catching::SearchBatchBinding;
using rtc::catching::SearchBatchCsvHeader;
using rtc::catching::SearchBatchCsvRow;
using rtc::catching::SearchBatchInputs;
using rtc::catching::SearchBatchKind;
using rtc::catching::SearchBatchRow;
using rtc::catching::SearchBatchSearch;
using rtc::catching::SearchBatchWake;
using rtc::catching::SearchStats;
using rtc::catching::SegmentBatchKind;
using rtc::catching::SegmentKind;
using rtc::catching::SegmentOutcome;
using rtc::catching::TrajSample;
using rtc::catching::WakeClock;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 10'000 * kMs;
constexpr int kSettle = 2;

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    out.push_back(line.substr(start, comma == std::string::npos ? comma : comma - start));
    if (comma == std::string::npos) {
      return out;
    }
    start = comma + 1;
  }
}

/// A result row without its last cell, `wall_us`: the one column that is the
/// machine's timing and not the search's verdict, so two runs never agree on it.
std::string WithoutWallUs(const std::string& row) {
  return row.substr(0, row.rfind(','));
}

std::map<std::string, std::string> Cells(const SearchBatchRow& row, int nv) {
  const auto names = Split(SearchBatchCsvHeader(nv));
  const auto values = Split(SearchBatchCsvRow(row, nv));
  EXPECT_EQ(names.size(), values.size());
  std::map<std::string, std::string> out;
  for (std::size_t i = 0; i < std::min(names.size(), values.size()); ++i) {
    out[names[i]] = values[i];
  }
  return out;
}

/// The 6R wrist arm and a grid search on it, built from a catching tree and a
/// binding as the executable builds it.
struct Rig {
  rtc::testing::Arm arm = rtc::testing::Arm6R();
  Eigen::VectorXd q_ref;
  rtc::testing::Target target;
  std::vector<double> wait_pose;

  Rig() {
    q_ref = Eigen::VectorXd::Zero(arm.nv);
    for (int i = 0; i < arm.nv; ++i) {
      q_ref(i) = 0.3 * ((i % 2 == 0) ? 1.0 : -1.0) + 0.1 * i;
      // Near the reference: the IK has real but short work.
      wait_pose.push_back(q_ref(i) + 0.15);
    }
    target = rtc::testing::TargetAt(arm, q_ref, /*speed=*/3.0);
  }

  [[nodiscard]] YAML::Node Tree() const {
    std::ostringstream s;
    s.precision(17);
    s << "planner:\n  enabled: true\n  wait_pose: [";
    for (std::size_t i = 0; i < wait_pose.size(); ++i) {
      s << (i == 0 ? "" : ", ") << wait_pose[i];
    }
    s << "]\n  freeze: {T_freeze: 0.2}\n"
         "  search:\n    grid:\n      budget_s: 0.05\n      max_ik: 8\n      n_settle: "
      << kSettle
      << "\n      slice: {t_max: 0.95}\n      hand: {d_eff: 0.2, r_cap: 0.03}\n"
         "      ik: {max_iter: 60}\n"
         "      catchability: {manipulability_min: {arm_5row: 1.0e-6}}\n";
    return YAML::Load(s.str());
  }

  [[nodiscard]] YAML::Node BindingFile() const {
    std::ostringstream s;
    s << "search: grid\ndevice_of_model: [0, 1, 2, 3, 4, 5]\ngrid:\n"
         "  qdot_max: [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]\n"
         "  qddot_max: [20, 20, 20, 20, 20, 20]\n"
         "  eta_v: 0.9\n  v_max: 3.0\n  a_dec: 10.0\n  t_arm_s: 0.0\n  t_close_lead: 0.1\n"
         "  t_close_total: 0.101\n  ball_mass: 0.057\n  ref_omega: 10.0\n  ref_zeta: 1.0\n"
         "  ref_a_max: 21.0\n  control_dt: 0.002\n  follows_segments: false\n";
    return YAML::Load(s.str());
  }

  [[nodiscard]] SearchBatchSearch Build() {
    const SearchBatchBinding binding = ParseSearchBatchBinding(BindingFile(), arm.nv);
    return MakeSearchBatchSearch(arm.model, *arm.handle, arm.frame, Tree(), binding);
  }

  /// A throw that passes through `p_c` with `v` at `kNow + 0.5 s`: `n_wakes`
  /// wakes 50 ms apart, each a prediction of 20 samples 50 ms apart from its
  /// own instant.
  [[nodiscard]] static std::vector<SearchBatchWake> Throw(std::int64_t id, const Eigen::Vector3d& p,
                                                          const Eigen::Vector3d& v, int n_wakes) {
    std::vector<SearchBatchWake> out;
    const std::int64_t t_c = kNow + 500 * kMs;
    for (int w = 0; w < n_wakes; ++w) {
      SearchBatchWake wake;
      wake.throw_id = id;
      wake.wake = w;
      wake.now_ns = kNow + w * 50 * kMs;
      for (int k = 0; k < 20; ++k) {
        TrajSample s;
        s.t_ns = wake.now_ns + k * 50 * kMs;
        const double dt = static_cast<double>(s.t_ns - t_c) / 1e9;
        const Eigen::Vector3d at = p + v * dt;
        s.p = {at.x(), at.y(), at.z()};
        s.v = {v.x(), v.y(), v.z()};
        wake.samples.push_back(s);
      }
      out.push_back(wake);
    }
    return out;
  }

  [[nodiscard]] std::vector<SearchBatchWake> Catchable(std::int64_t id, int n_wakes = 6) const {
    return Throw(id, target.p_c, target.v_ball, n_wakes);
  }

  /// Ten metres off: no pose reaches any of its samples.
  [[nodiscard]] std::vector<SearchBatchWake> OutOfReach(std::int64_t id, int n_wakes = 4) const {
    return Throw(id, target.p_c + Eigen::Vector3d(10.0, 0.0, 0.0), target.v_ball, n_wakes);
  }
};

std::vector<SearchBatchWake> Concat(std::vector<std::vector<SearchBatchWake>> throws) {
  std::vector<SearchBatchWake> out;
  for (auto& t : throws) {
    out.insert(out.end(), t.begin(), t.end());
  }
  return out;
}

std::string WakeCsv(const std::vector<SearchBatchWake>& wakes) {
  std::ostringstream s;
  s.precision(17);
  s << "throw_id,wake,now_ns,t_ns,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z\n";
  for (const auto& w : wakes) {
    for (const auto& k : w.samples) {
      s << w.throw_id << ',' << w.wake << ',' << w.now_ns << ',' << k.t_ns;
      for (const auto& block : {k.p, k.v, k.a}) {
        for (const double v : block) {
          s << ',' << v;
        }
      }
      s << '\n';
    }
  }
  return s.str();
}

// ── 1, 3. A row is the direct call's; the wakes stop at the first plan ───────

TEST(CatchSearchBatch, ARowIsTheDirectPlanCallAndTheThrowStopsAtItsFirstPlan) {
  Rig rig;
  SearchBatchSearch batch = rig.Build();
  SearchBatchSearch direct = rig.Build();
  ASSERT_NE(batch.search, nullptr) << batch.error;
  ASSERT_NE(direct.search, nullptr) << direct.error;
  ASSERT_EQ(batch.q_rest, rig.wait_pose);

  const auto wakes = rig.Catchable(7);
  const auto rows = RunSearchBatch(*batch.search, batch.q_rest, wakes);
  // The search settles for kSettle predictions, then plans: one row per wake
  // up to that plan, none after it.
  ASSERT_EQ(rows.size(), static_cast<std::size_t>(kSettle + 1));
  EXPECT_TRUE(rows.back().plan.valid);
  for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
    EXPECT_FALSE(rows[i].plan.valid) << i;
    EXPECT_TRUE(rows[i].stats.settling) << i;
  }

  const ReportedSegments none{};
  direct.search->ResetTrial();
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const SearchBatchInputs in = MakeSearchBatchInputs(wakes[i], direct.q_rest);
    SearchBatchRow row;
    row.throw_id = wakes[i].throw_id;
    row.wake = wakes[i].wake;
    row.now_ns = wakes[i].now_ns;
    row.plan = direct.search->Plan(in.traj, in.cov, true, in.rt, none, NowReal{wakes[i].now_ns}, 0,
                                   row.stats);
    EXPECT_EQ(WithoutWallUs(SearchBatchCsvRow(row, rig.arm.nv)),
              WithoutWallUs(SearchBatchCsvRow(rows[i], rig.arm.nv)))
        << i;
  }
}

// ── 2. Order of the throws ───────────────────────────────────────────────────

TEST(CatchSearchBatch, AThrowsRowsDoNotDependOnTheThrowsBeforeIt) {
  Rig rig;
  SearchBatchSearch search = rig.Build();
  ASSERT_NE(search.search, nullptr) << search.error;
  // A second catchable throw, a little off the first.
  const auto other =
      Rig::Throw(3, rig.target.p_c + Eigen::Vector3d(0.0, 0.02, -0.02), rig.target.v_ball * 0.9, 6);
  const auto forward = Concat({rig.Catchable(1), rig.OutOfReach(2), other});
  const auto backward = Concat({other, rig.OutOfReach(2), rig.Catchable(1)});

  const auto by_throw = [&](const std::vector<SearchBatchWake>& wakes) {
    std::map<std::int64_t, std::vector<std::string>> out;
    for (const auto& row : RunSearchBatch(*search.search, search.q_rest, wakes)) {
      out[row.throw_id].push_back(WithoutWallUs(SearchBatchCsvRow(row, rig.arm.nv)));
    }
    return out;
  };
  const auto a = by_throw(forward);
  const auto b = by_throw(backward);
  ASSERT_EQ(a.size(), 3U);
  EXPECT_EQ(a, b);
  // The unreachable throw ran every wake and none planned; the others planned.
  EXPECT_EQ(a.at(2).size(), 4U);
  EXPECT_EQ(a.at(1).size(), static_cast<std::size_t>(kSettle + 1));
  EXPECT_EQ(a.at(3).size(), static_cast<std::size_t>(kSettle + 1));
}

// ── 4. The CSV ───────────────────────────────────────────────────────────────

TEST(CatchSearchBatch, TheCsvCarriesThePlanBitExactlyAndLeavesNoPlanEmpty) {
  Rig rig;
  SearchBatchSearch search = rig.Build();
  ASSERT_NE(search.search, nullptr) << search.error;
  const auto rows =
      RunSearchBatch(*search.search, search.q_rest, Concat({rig.Catchable(1), rig.OutOfReach(2)}));
  const SearchBatchRow& planned = rows[static_cast<std::size_t>(kSettle)];
  ASSERT_TRUE(planned.plan.valid);
  const auto cells = Cells(planned, rig.arm.nv);
  const auto bits = [](const std::string& text, double value) {
    return rtc::testing::BitsEqual(std::strtod(text.c_str(), nullptr), value);
  };
  EXPECT_EQ(cells.at("throw_id"), "1");
  EXPECT_EQ(cells.at("search_valid"), "1");
  EXPECT_EQ(cells.at("plan_reason"), "0");
  EXPECT_EQ(cells.at("settling"), "0");
  EXPECT_EQ(cells.at("nlp_ran"), "0");
  EXPECT_EQ(cells.at("nlp_reason"), "off");
  EXPECT_EQ(cells.at("t_c_ns"), std::to_string(planned.plan.t_c_ns));
  EXPECT_TRUE(bits(cells.at("lead_s"), planned.stats.chosen_lead_s));
  EXPECT_TRUE(bits(cells.at("p_c_x"), planned.plan.p_c[0]));
  EXPECT_TRUE(bits(cells.at("p_c_z"), planned.plan.p_c[2]));
  EXPECT_TRUE(bits(cells.at("v_c_y"), planned.plan.v_c[1]));
  EXPECT_TRUE(bits(cells.at("score"), planned.plan.score));
  EXPECT_TRUE(bits(cells.at("w5"), planned.plan.w5));
  EXPECT_TRUE(bits(cells.at("gamma_f"), planned.stats.chosen_gamma_f));
  for (int j = 0; j < rig.arm.nv; ++j) {
    EXPECT_TRUE(bits(cells.at("q_star" + std::to_string(j)),
                     planned.plan.q_star[static_cast<std::size_t>(j)]))
        << j;
  }
  EXPECT_EQ(cells.at("nlp_lead_s"), "");  // not the NLP search's wake

  // The last row is the unreachable throw's last wake: counted, not planned.
  // The first wake settles (n_settle): no candidate, and the column says so.
  const auto settling = Cells(rows.front(), rig.arm.nv);
  EXPECT_EQ(settling.at("settling"), "1");
  EXPECT_EQ(settling.at("search_valid"), "0");
  EXPECT_EQ(settling.at("n_in_window"), "0");

  const SearchBatchRow& refused = rows.back();
  ASSERT_FALSE(refused.plan.valid);
  const auto empty = Cells(refused, rig.arm.nv);
  EXPECT_EQ(empty.at("search_valid"), "0");
  EXPECT_NE(empty.at("plan_reason"), "0");
  EXPECT_NE(empty.at("n_in_window"), "0");
  // Ten metres off is past the arm's reach: the pre-filter refused every
  // candidate and the IK ran on none (L3 §4.1).
  EXPECT_EQ(empty.at("rej_too_far"), empty.at("n_in_window"));
  EXPECT_EQ(empty.at("n_ik"), "0");
  EXPECT_EQ(empty.at("rej_ik"), "0");
  for (const char* name : {"t_c_ns", "lead_s", "p_c_x", "v_c_z", "score", "w5", "w6", "rank_mask",
                           "gamma_f", "nlp_lead_s", "q_star0", "q_star5"}) {
    EXPECT_EQ(empty.at(name), "") << name;
  }
}

// ── 7. The wall time column; the reach bound report ──────────────────────────

TEST(CatchSearchBatch, WallUsIsTheLastColumnAndEveryOtherColumnKeepsItsPlace) {
  Rig rig;
  const auto names = Split(SearchBatchCsvHeader(rig.arm.nv));
  ASSERT_GT(names.size(), 2U);
  EXPECT_EQ(names.back(), "wall_us");
  EXPECT_EQ(names[names.size() - 2], "q_star" + std::to_string(rig.arm.nv - 1));
  // The columns before `wall_us`, in their order.
  const std::vector<std::string> head{"throw_id",     "wake",        "now_ns",
                                      "search_valid", "plan_reason", "settling",
                                      "n_in_window",  "n_ik",        "n_pass"};
  for (std::size_t i = 0; i < head.size(); ++i) {
    EXPECT_EQ(names[i], head[i]) << i;
  }
  EXPECT_EQ(names[names.size() - 1 - static_cast<std::size_t>(rig.arm.nv) - 1], "nlp_lead_s");

  SearchBatchSearch search = rig.Build();
  ASSERT_NE(search.search, nullptr) << search.error;
  const auto rows =
      RunSearchBatch(*search.search, search.q_rest, Concat({rig.Catchable(1), rig.OutOfReach(2)}));
  ASSERT_FALSE(rows.empty());
  for (const SearchBatchRow& row : rows) {
    const auto cells = Cells(row, rig.arm.nv);
    const double wall = std::strtod(cells.at("wall_us").c_str(), nullptr);
    EXPECT_TRUE(std::isfinite(wall));
    EXPECT_GE(wall, 0.0);
    EXPECT_EQ(Split(SearchBatchCsvRow(row, rig.arm.nv)).back(), cells.at("wall_us"));
  }
}

TEST(CatchSearchBatchReachBound, ABoundedBoundIsOneJsonLineAtRoundTripPrecision) {
  ReachBound bound;
  bound.centre = Eigen::Vector3d(0.1, -0.25, 0.5);
  bound.radius = 1.0;
  bound.joints = 6;
  EXPECT_EQ(ReachBoundJson(bound, "catch_frame", "arm", {0.002, 0.002, 0.004}),
            "{\"schema\": \"catch_reach_bound/1\", \"frame\": \"catch_frame\", "
            "\"sub_model\": \"arm\", \"centre\": [0.10000000000000001, -0.25, 0.5], "
            "\"radius\": 1, \"tolerance\": 0.002, "
            "\"tolerance_by_search\": {\"grid\": 0.002, \"nlp\": 0.0040000000000000001}, "
            "\"joints\": 6, \"unbounded_by\": \"\"}");
}

TEST(CatchSearchBatchReachBound, AnUnboundedBoundWritesANullRadius) {
  ReachBound bound;  // radius is +inf by default
  bound.joints = 3;
  bound.unbounded_by = "slide";
  const std::string json = ReachBoundJson(bound, "f", "m", {0.5, 0.5, 0.25});
  EXPECT_EQ(json,
            "{\"schema\": \"catch_reach_bound/1\", \"frame\": \"f\", \"sub_model\": \"m\", "
            "\"centre\": [0, 0, 0], \"radius\": null, \"tolerance\": 0.5, "
            "\"tolerance_by_search\": {\"grid\": 0.5, \"nlp\": 0.25}, \"joints\": 3, "
            "\"unbounded_by\": \"slide\"}");
  EXPECT_EQ(json.find('\n'), std::string::npos);
}

TEST(CatchSearchBatchReachBound, TheBoundAndToleranceAreTheOnesTheSearchesUse) {
  Rig rig;
  const ReachBound bound = rtc::catching::ComputeReachBound(*rig.arm.model, rig.arm.frame);
  ASSERT_TRUE(bound.Bounded());
  const rtc::catching::ReachTolerances tol = rtc::catching::SearchReachTolerances(rig.Tree());
  EXPECT_GT(tol.grid, 0.0);
  EXPECT_GT(tol.nlp, 0.0);
  // The selected search's is one of the two.
  EXPECT_TRUE(tol.selected == tol.grid || tol.selected == tol.nlp);
  const std::string json = ReachBoundJson(bound, "catch_frame", "arm", tol);
  char radius[40];
  std::snprintf(radius, sizeof radius, "%.17g", bound.radius);
  EXPECT_NE(json.find(std::string("\"radius\": ") + radius + ","), std::string::npos);
}

// ── 5. What a wake is handed ─────────────────────────────────────────────────

TEST(CatchSearchBatch, AWakeIsAnArmAtRestWithAZeroCovarianceOfThePrediction) {
  Rig rig;
  const auto wakes = rig.Catchable(4);
  const SearchBatchInputs first = MakeSearchBatchInputs(wakes[0], rig.wait_pose);
  const SearchBatchInputs third = MakeSearchBatchInputs(wakes[2], rig.wait_pose);

  EXPECT_TRUE(first.traj.valid);
  EXPECT_EQ(first.traj.n, 20);
  EXPECT_EQ(first.traj.s[3].t_ns, wakes[0].samples[3].t_ns);
  EXPECT_EQ(first.traj.s[3].p, wakes[0].samples[3].p);
  // One track, a new snapshot per wake.
  EXPECT_EQ(first.traj.token.generation, third.traj.token.generation);
  EXPECT_LT(first.traj.token.snapshot_sequence, third.traj.token.snapshot_sequence);

  EXPECT_TRUE(first.cov.valid);
  EXPECT_EQ(first.cov.n, first.traj.n);
  EXPECT_EQ(first.cov.token.generation, first.traj.token.generation);
  EXPECT_EQ(first.cov.token.snapshot_sequence, first.traj.token.snapshot_sequence);
  for (const auto& sample : first.cov.c) {
    for (const double v : sample) {
      EXPECT_EQ(v, 0.0);
    }
  }

  EXPECT_TRUE(first.rt.valid);
  EXPECT_TRUE(first.rt.cmd_seeded);
  EXPECT_EQ(first.rt.nv, rig.arm.nv);
  EXPECT_EQ(first.rt.rt_state_ns, wakes[0].now_ns);  // age 0
  EXPECT_FALSE(first.rt.plan_active);
  EXPECT_FALSE(first.rt.segment_active);
  EXPECT_FALSE(first.rt.ref_valid);
  EXPECT_FALSE(first.rt.wait_pose_adopted);
  for (int j = 0; j < rig.arm.nv; ++j) {
    const auto u = static_cast<std::size_t>(j);
    EXPECT_EQ(first.rt.q_cmd[u], rig.wait_pose[u]);
    EXPECT_EQ(first.rt.qd_cmd[u], 0.0);
  }
  EXPECT_EQ(first.rt.activation_generation, first.traj.token.activation_generation);

  SearchBatchWake empty = wakes[0];
  empty.samples.clear();
  EXPECT_THROW(static_cast<void>(MakeSearchBatchInputs(empty, rig.wait_pose)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(MakeSearchBatchInputs(wakes[0], {})), std::invalid_argument);
}

TEST(CatchSearchBatch, TheClockDoesNotAdvance) {
  EXPECT_EQ(rtc::catching::StoppedClock(), rtc::catching::StoppedClock());
}

// ── 6. The wake CSV ──────────────────────────────────────────────────────────

TEST(CatchSearchBatchWakeCsv, RoundTripsTheWakes) {
  Rig rig;
  const auto wakes = Concat({rig.Catchable(5, 3), rig.OutOfReach(9, 2)});
  std::istringstream in(WakeCsv(wakes));
  const auto parsed = ParseSearchWakeCsv(in);
  ASSERT_EQ(parsed.size(), wakes.size());
  for (std::size_t i = 0; i < wakes.size(); ++i) {
    EXPECT_EQ(parsed[i].throw_id, wakes[i].throw_id);
    EXPECT_EQ(parsed[i].wake, wakes[i].wake);
    EXPECT_EQ(parsed[i].now_ns, wakes[i].now_ns);
    ASSERT_EQ(parsed[i].samples.size(), wakes[i].samples.size());
    for (std::size_t k = 0; k < wakes[i].samples.size(); ++k) {
      EXPECT_EQ(parsed[i].samples[k].t_ns, wakes[i].samples[k].t_ns);
      EXPECT_EQ(parsed[i].samples[k].p, wakes[i].samples[k].p);
      EXPECT_EQ(parsed[i].samples[k].v, wakes[i].samples[k].v);
      EXPECT_EQ(parsed[i].samples[k].a, wakes[i].samples[k].a);
    }
  }
}

TEST(CatchSearchBatchWakeCsv, RefusesWhatItCannotReadAsWakes) {
  const std::string header = "throw_id,wake,now_ns,t_ns,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z\n";
  const auto row = [](int id, int wake, int now, int t, const std::string& px = "0") {
    return std::to_string(id) + ',' + std::to_string(wake) + ',' + std::to_string(now) + ',' +
           std::to_string(t) + ',' + px + ",0,0,1,0,0,0,0,0\n";
  };
  const auto refused = [&](const std::string& body) {
    std::istringstream in(header + body);
    try {
      static_cast<void>(ParseSearchWakeCsv(in));
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(refused(row(1, 0, 10, 10) + row(1, 0, 10, 20) + row(1, 1, 20, 20)));
  EXPECT_TRUE(refused(row(1, 0, 10, 10) + row(1, 0, 11, 20))) << "now_ns changes inside a wake";
  EXPECT_TRUE(refused(row(1, 0, 10, 10) + row(1, 0, 10, 10))) << "an instant repeats";
  EXPECT_TRUE(refused(row(1, 1, 10, 10) + row(1, 0, 20, 20))) << "a wake index goes back";
  EXPECT_TRUE(refused(row(1, 0, 10, 10) + row(2, 0, 10, 10) + row(1, 1, 20, 20)))
      << "a throw comes back";
  EXPECT_TRUE(refused(row(1, -1, 10, 10))) << "a negative wake index";
  EXPECT_TRUE(refused(row(1, 0, 10, 10, "nan"))) << "a non-finite cell";
  EXPECT_TRUE(refused(row(1, 0, 10, 10, "x"))) << "not a number";
  std::string too_many;
  for (int k = 0; k <= kCap; ++k) {
    too_many += row(1, 0, 10, 10 + k);
  }
  EXPECT_TRUE(refused(too_many)) << "more samples than a snapshot holds";
  std::istringstream no_column("throw_id,wake,now_ns,t_ns,p_x,p_y,p_z\n1,0,10,10,0,0,0\n");
  EXPECT_THROW(static_cast<void>(ParseSearchWakeCsv(no_column)), std::invalid_argument);
}

// ── 6. The binding file ──────────────────────────────────────────────────────

const char* const kNlpBinding =
    "search: nlp\ndevice_of_model: [1, 0, 2]\nnlp:\n"
    "  t_arm_s: 0.05\n  control_dt: 0.002\n  t_close_lead: .nan\n  hand_t_close_e2e: 0.2\n"
    "  hand_t_close_lead: 0.15\n  ball_mass: 0.057\n"
    "  limits:\n    q_min: [-1, -2, -3]\n    q_max: [1, 2, 3]\n    qd_max: [1, 1, 1]\n"
    "    qdd_max: []\n    tau_max: [10, 20, 30]\n    tau_lo: [-9, -18, -27]\n"
    "    tau_hi: [9, 18, 27]\n";

TEST(CatchSearchBatchBinding, ReadsBothSearches) {
  Rig rig;
  const SearchBatchBinding grid = ParseSearchBatchBinding(rig.BindingFile(), rig.arm.nv);
  EXPECT_EQ(grid.kind, SearchBatchKind::kGrid);
  EXPECT_EQ(grid.device_of_model, (std::vector<int>{0, 1, 2, 3, 4, 5}));
  EXPECT_EQ(grid.qdot_max.size(), 6U);
  EXPECT_EQ(grid.qddot_max.size(), 6U);
  EXPECT_EQ(grid.grid.v_max, 3.0);
  EXPECT_EQ(grid.grid.t_close_total, 0.101);
  EXPECT_FALSE(grid.grid.follows_segments);

  const SearchBatchBinding nlp = ParseSearchBatchBinding(YAML::Load(kNlpBinding), 3);
  EXPECT_EQ(nlp.kind, SearchBatchKind::kNlp);
  EXPECT_EQ(nlp.device_of_model, (std::vector<int>{1, 0, 2}));
  EXPECT_EQ(nlp.nlp.t_arm_s, 0.05);
  EXPECT_TRUE(std::isnan(nlp.nlp.t_close_lead));  // a TBD value stays one
  EXPECT_EQ(nlp.hand_t_close_e2e, 0.2);
  EXPECT_EQ(nlp.hand_t_close_lead, 0.15);
  EXPECT_EQ(nlp.limits.q_min.size(), 3);
  EXPECT_EQ(nlp.limits.qdd_max.size(), 0);  // no acceleration box
  EXPECT_EQ(nlp.limits.tau_hi[2], 27.0);
  ASSERT_EQ(nlp.limits.armature.size(), 3);
  EXPECT_EQ(nlp.limits.armature.cwiseAbs().maxCoeff(), 0.0);
}

// A search on a CHAIN of a wider device (E3-F01, #640): `n_dev` is the
// device's joint count, `device_of_model` an injection into it; the segment
// planner's map is sized by ITS model (`segment_nv`) and may give its own
// device map — a permutation of the whole device.
TEST(CatchSearchBatchBinding, ASearchModelMayBeAChainOfAWiderDevice) {
  const std::string chain = std::string("n_dev: 5\n") + kNlpBinding;
  const std::string at = "device_of_model: [1, 0, 2]";
  const auto with_map = [&](const std::string& map) {
    std::string s = chain;
    s.replace(s.find(at), at.size(), "device_of_model: " + map);
    return s;
  };
  const SearchBatchBinding b = ParseSearchBatchBinding(YAML::Load(with_map("[4, 0, 2]")), 3);
  EXPECT_EQ(b.n_dev, 5);
  EXPECT_EQ(b.device_of_model, (std::vector<int>{4, 0, 2}));
  // Without `n_dev` the model is the whole device: 0 = "the model's".
  EXPECT_EQ(ParseSearchBatchBinding(YAML::Load(kNlpBinding), 3).n_dev, 0);
  const auto refused = [](const std::string& text, int nv, int segment_nv = 0) {
    try {
      static_cast<void>(ParseSearchBatchBinding(YAML::Load(text), nv, segment_nv));
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(refused(with_map("[4, 0, 2]"), 3));
  EXPECT_TRUE(refused(with_map("[5, 0, 2]"), 3)) << "an index past the device";
  EXPECT_TRUE(refused(with_map("[4, 4, 2]"), 3)) << "one device joint twice";
  EXPECT_TRUE(refused(with_map("[4, 0]"), 3)) << "not one entry per model joint";
  EXPECT_TRUE(refused(std::string("n_dev: 2\n") + kNlpBinding, 3)) << "narrower than the model";
  EXPECT_TRUE(refused(std::string("n_dev: 33\n") + kNlpBinding, 3)) << "wider than a plan names";
  // Without `n_dev` an index past the model is still refused.
  std::string no_dev = kNlpBinding;
  no_dev.replace(no_dev.find(at), at.size(), "device_of_model: [4, 0, 2]");
  EXPECT_TRUE(refused(no_dev, 3));

  // The segment planner's map: vectors of the SEGMENT model's width, and its
  // own device map.
  const std::string segment =
      "segment:\n  mode: mpc\n  device_of_model: [0, 1, 2, 3, 4]\n  mpc:\n"
      "    q_min: [-1, -1, -1, -1, -1]\n    q_max: [1, 1, 1, 1, 1]\n"
      "    qdot_max: [2, 2, 2, 2, 2]\n    tau_max: [9, 9, 9, 9, 9]\n"
      "    eta_v: 0.9\n    t_arm_s: 0.0\n    control_dt: 0.002\n";
  const SearchBatchBinding two =
      ParseSearchBatchBinding(YAML::Load(with_map("[4, 0, 2]") + segment), 3, 5);
  ASSERT_TRUE(two.segment.has_value());
  EXPECT_EQ(two.segment->device_of_model, (std::vector<int>{0, 1, 2, 3, 4}));
  EXPECT_EQ(two.segment->q_min.size(), 5U);
  EXPECT_TRUE(refused(with_map("[4, 0, 2]") + segment, 3))
      << "segment vectors of five joints against a segment model of three";
  std::string repeated = segment;
  repeated.replace(repeated.find("[0, 1, 2, 3, 4]"), 15, "[0, 1, 2, 3, 3]");
  EXPECT_TRUE(refused(with_map("[4, 0, 2]") + repeated, 3, 5));
  EXPECT_TRUE(refused(with_map("[4, 0, 2]") + segment, 3, 2))
      << "a segment model below the search's";
  // A segment model wider than the search's is the device: the binding has to
  // give `n_dev`, and the refusal names that key — not a device-wide value
  // (the wait pose) that would then be judged against the search's width.
  const auto why = [](const std::string& text, int nv, int segment_nv) {
    try {
      static_cast<void>(ParseSearchBatchBinding(YAML::Load(text), nv, segment_nv));
    } catch (const std::invalid_argument& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  const std::string absent = why(std::string(kNlpBinding) + segment, 3, 5);
  EXPECT_NE(absent.find("'n_dev' must be 5"), std::string::npos) << absent;
  EXPECT_NE(absent.find("absent"), std::string::npos) << absent;
  std::string narrower = with_map("[3, 0, 2]") + segment;
  narrower.replace(narrower.find("n_dev: 5"), 8, "n_dev: 4");
  const std::string four = why(narrower, 3, 5);
  EXPECT_NE(four.find("'n_dev' must be 5"), std::string::npos) << four;
  EXPECT_NE(four.find("got 4"), std::string::npos) << four;
}

TEST(CatchSearchBatchBinding, IsFailClosed) {
  Rig rig;
  const auto refused = [&](const std::string& edit_from, const std::string& edit_to) {
    std::ostringstream text;
    text << rig.BindingFile();
    std::string s = text.str();
    const std::size_t at = s.find(edit_from);
    EXPECT_NE(at, std::string::npos) << edit_from;
    if (at == std::string::npos) {
      return false;
    }
    s.replace(at, edit_from.size(), edit_to);
    try {
      static_cast<void>(ParseSearchBatchBinding(YAML::Load(s), rig.arm.nv));
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(refused("eta_v: 0.9", "eta_v: 0.8"));
  EXPECT_FALSE(refused("v_max: 3.0", "v_max: .nan")) << "a TBD scalar is a value";
  EXPECT_FALSE(refused("qddot_max: [20, 20, 20, 20, 20, 20]", "qddot_max: []"));
  EXPECT_TRUE(refused("search: grid", "search: both"));
  EXPECT_TRUE(refused("search: grid", "search: nlp")) << "the selected search's map is absent";
  EXPECT_TRUE(refused("eta_v: 0.9", "eta_vv: 0.9")) << "an unknown key, and a missing one";
  EXPECT_TRUE(refused("eta_v: 0.9", "eta_v: [0.9]"));
  EXPECT_TRUE(refused("follows_segments: false", "follows_segments: 0.5"));
  EXPECT_TRUE(refused("qdot_max: [3.14, ", "qdot_max: ["));
  EXPECT_TRUE(refused("qdot_max: [3.14, ", "qdot_max: [.nan, "));
  EXPECT_TRUE(
      refused("device_of_model: [0, 1, 2, 3, 4, 5]", "device_of_model: [0, 1, 2, 3, 4, 4]"));
  EXPECT_TRUE(refused("device_of_model: [0, 1, 2, 3, 4, 5]", "device_of_model: [0, 1, 2]"));
  EXPECT_THROW(static_cast<void>(ParseSearchBatchBinding(YAML::Load("[1, 2]"), rig.arm.nv)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(ParseSearchBatchBinding(rig.BindingFile(), 0)),
               std::invalid_argument);
}

// The parsers default an absent map; the batch does not let a tree without
// the selected search's map be judged on compiled-in values.
TEST(CatchSearchBatchBinding, ATreeWithoutTheSelectedSearchsMapIsRefused) {
  Rig rig;
  const SearchBatchBinding binding = ParseSearchBatchBinding(rig.BindingFile(), rig.arm.nv);
  YAML::Node tree = rig.Tree();
  tree["planner"]["search"].remove("grid");
  EXPECT_THROW(static_cast<void>(MakeSearchBatchSearch(rig.arm.model, *rig.arm.handle,
                                                       rig.arm.frame, tree, binding)),
               std::invalid_argument);
}

TEST(CatchSearchBatchBinding, AWaitPoseOfAnotherWidthIsRefused) {
  Rig rig;
  rig.wait_pose.pop_back();
  const SearchBatchBinding binding = ParseSearchBatchBinding(rig.BindingFile(), rig.arm.nv);
  EXPECT_THROW(static_cast<void>(MakeSearchBatchSearch(rig.arm.model, *rig.arm.handle,
                                                       rig.arm.frame, rig.Tree(), binding)),
               std::invalid_argument);
}

// A profile whose hand has no docking values is one the NLP search cannot run
// on: reported, not thrown — the caller has a map to leave empty.
TEST(CatchSearchBatchBinding, AnNlpProfileWithoutTheHandsValuesIsReported) {
  Rig rig;
  std::string text = kNlpBinding;
  text.replace(text.find("[1, 0, 2]"), 9, "[0, 1, 2, 3, 4, 5]");
  for (const char* key : {"q_min", "q_max", "qd_max", "tau_max", "tau_lo", "tau_hi"}) {
    const std::size_t at = text.find(std::string(key) + ": [");
    const std::size_t end = text.find(']', at);
    text.replace(at, end - at + 1, std::string(key) + ": [1, 1, 1, 1, 1, 1]");
  }
  const SearchBatchBinding binding = ParseSearchBatchBinding(YAML::Load(text), rig.arm.nv);
  YAML::Node tree = rig.Tree();
  tree["planner"]["search"]["nlp"] = YAML::Load("{budget_s: 0.05}");
  const SearchBatchSearch search =
      MakeSearchBatchSearch(rig.arm.model, *rig.arm.handle, rig.arm.frame, tree, binding);
  EXPECT_EQ(search.search, nullptr);
  EXPECT_NE(search.error.find("robot.hand.docking"), std::string::npos) << search.error;
}

// ── 7. The whole wake ────────────────────────────────────────────────────────

const char* const kDockingSegment =
    "segment:\n  mode: mpc_docking\n  mpc_docking:\n"
    "    t_arm_s: 0.05\n    control_dt: 0.002\n    qd_rating: [2, 2, 3]\n"
    "    hand_t_close_e2e: 0.2\n    hand_t_close_lead: .nan\n    ball_mass: 0.057\n"
    "    limits:\n      q_min: [-1, -2, -3]\n      q_max: [1, 2, 3]\n      qd_max: [1, 1, 1]\n"
    "      qdd_max: []\n      tau_max: [10, 20, 30]\n      tau_lo: [-9, -18, -27]\n"
    "      tau_hi: [9, 18, 27]\n";

/// A 6-joint arm with inertias, a grid search and the mpc segment planner on
/// it, built from a catching tree and a binding as the executable builds a
/// whole-wake batch. The arm, the catch posture and the ball are those of the
/// approach-cycle suite, whose first wake publishes a pair.
struct CycleRig {
  std::shared_ptr<const pinocchio::Model> model;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  pinocchio::FrameIndex frame{0};
  int nv{0};
  Eigen::Vector3d p_c;
  Eigen::Vector3d v_ball;
  std::vector<double> wait_pose;
  /// `planner.segment.mpc.publish.catch_pos_err_max` of the tree [m].
  double catch_err_max{0.02};

  CycleRig() {
    rtc_urdf_bridge::ModelConfig config;
    config.urdf_path = std::string(RTC_TEST_ROBOT_DESCRIPTIONS_DIR) + "/ur5e/urdf/ur5e.urdf";
    config.root_joint_type = "fixed";
    rtc_urdf_bridge::PinocchioModelBuilder builder(config);
    model = builder.GetFullModel();
    handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model);
    frame = model->getFrameId("tool0");
    nv = model->nv;
    Eigen::VectorXd q_ref(nv);
    q_ref << 0.2, -1.4, 1.1, -1.9, -1.6, 0.1;
    pinocchio::Data data(*model);
    pinocchio::forwardKinematics(*model, data, q_ref);
    pinocchio::updateFramePlacement(*model, data, frame);
    p_c = data.oMf[frame].translation();
    // A slow ball along the frame's −z: a short reach from the wait pose.
    v_ball = -1.0 * data.oMf[frame].rotation().col(2);
    for (int j = 0; j < nv; ++j) {
      wait_pose.push_back(q_ref[j] + (j % 2 == 0 ? 0.03 : -0.03));
    }
  }

  [[nodiscard]] static std::string List(const std::vector<double>& v) {
    std::ostringstream s;
    s.precision(17);
    s << '[';
    for (std::size_t i = 0; i < v.size(); ++i) {
      s << (i == 0 ? "" : ", ") << v[i];
    }
    s << ']';
    return s.str();
  }

  [[nodiscard]] YAML::Node Tree() const {
    std::ostringstream s;
    s.precision(17);
    s << "planner:\n  enabled: true\n  wait_pose: " << List(wait_pose)
      << "\n  freeze: {T_freeze: 0.2}\n"
         "  search:\n    grid:\n      budget_s: 0.05\n      max_ik: 8\n      n_settle: 0\n"
         "      slice: {t_lead_min: 0.6, t_max: 0.8}\n      hand: {d_eff: 0.2, r_cap: 0.03}\n"
         "      ik: {max_iter: 60}\n"
         "      catchability: {manipulability_min: {arm_5row: 1.0e-6}}\n"
         "  segment:\n    mode: mpc\n    mpc:\n"
         "      horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, 2, 3]}\n"
         "      approach: {n_pre_max: 6, dt_pre_s: 0.1}\n"
         "      replan: {k_max: 2}\n"
         "      publish: {catch_pos_err_max: "
      << catch_err_max << "}\n";
    return YAML::Load(s.str());
  }

  [[nodiscard]] YAML::Node BindingFile(bool with_segment = true) const {
    std::vector<double> q_min;
    std::vector<double> q_max;
    for (int j = 0; j < nv; ++j) {
      q_min.push_back(model->lowerPositionLimit[j]);
      q_max.push_back(model->upperPositionLimit[j]);
    }
    std::ostringstream s;
    s << "search: grid\ndevice_of_model: [0, 1, 2, 3, 4, 5]\ngrid:\n"
         "  qdot_max: [2, 2, 3, 3, 3, 3]\n  qddot_max: [20, 20, 20, 20, 20, 20]\n"
         "  eta_v: 0.9\n  v_max: 3.0\n  a_dec: 10.0\n  t_arm_s: 0.0\n  t_close_lead: 0.1\n"
         "  t_close_total: 0.101\n  ball_mass: 0.057\n  ref_omega: 10.0\n  ref_zeta: 1.0\n"
         "  ref_a_max: 21.0\n  control_dt: 0.002\n  follows_segments: true\n";
    if (with_segment) {
      s << "segment:\n  mode: mpc\n  mpc:\n    q_min: " << List(q_min)
        << "\n    q_max: " << List(q_max)
        << "\n    qdot_max: [2, 2, 3, 3, 3, 3]\n    tau_max: [150, 150, 150, 28, 28, 28]\n"
           "    eta_v: 0.9\n    t_arm_s: 0.0\n    control_dt: 0.002\n";
    }
    return YAML::Load(s.str());
  }

  [[nodiscard]] PlanBatchBuild Build() const {
    return MakePlanBatch(model, *handle, frame, Tree(), ParseSearchBatchBinding(BindingFile(), nv));
  }

  [[nodiscard]] SearchBatchSearch BuildSearch() const {
    return MakeSearchBatchSearch(model, *handle, frame, Tree(),
                                 ParseSearchBatchBinding(BindingFile(false), nv));
  }

  /// A throw that passes through `p` with `v` 0.7 s after its first wake:
  /// `n_wakes` wakes 50 ms apart, each a prediction of 20 samples 50 ms apart
  /// from its own instant.
  [[nodiscard]] static std::vector<SearchBatchWake> Throw(std::int64_t id, const Eigen::Vector3d& p,
                                                          const Eigen::Vector3d& v, int n_wakes) {
    std::vector<SearchBatchWake> out;
    const std::int64_t t_c = kNow + 700 * kMs;
    for (int w = 0; w < n_wakes; ++w) {
      SearchBatchWake wake;
      wake.throw_id = id;
      wake.wake = w;
      wake.now_ns = kNow + w * 50 * kMs;
      for (int k = 0; k < 20; ++k) {
        TrajSample s;
        s.t_ns = wake.now_ns + k * 50 * kMs;
        const double dt = static_cast<double>(s.t_ns - t_c) / 1e9;
        const Eigen::Vector3d at = p + v * dt;
        s.p = {at.x(), at.y(), at.z()};
        s.v = {v.x(), v.y(), v.z()};
        wake.samples.push_back(s);
      }
      out.push_back(wake);
    }
    return out;
  }

  [[nodiscard]] std::vector<SearchBatchWake> Catchable(std::int64_t id, int n_wakes = 4) const {
    return Throw(id, p_c, v_ball, n_wakes);
  }
};

std::map<std::string, std::string> CycleCells(const SearchBatchRow& row, int nv) {
  const auto names = Split(PlanBatchCsvHeader(nv));
  const auto values = Split(PlanBatchCsvRow(row, nv));
  EXPECT_EQ(names.size(), values.size());
  std::map<std::string, std::string> out;
  for (std::size_t i = 0; i < std::min(names.size(), values.size()); ++i) {
    out[names[i]] = values[i];
  }
  return out;
}

TEST(CatchPlanBatch, AThrowStopsAtItsFirstPublishedPairAndItsSearchCellsAreTheSearchBatchs) {
  const CycleRig rig;
  PlanBatchBuild built = rig.Build();
  ASSERT_NE(built.batch, nullptr) << built.error;
  ASSERT_EQ(built.batch->RestPose(), rig.wait_pose);
  const auto wakes = rig.Catchable(7);
  const auto rows = built.batch->Run(wakes);
  ASSERT_FALSE(rows.empty());
  ASSERT_LT(rows.size(), wakes.size()) << "the wakes after the pair are not run";
  const SearchBatchRow& last = rows.back();
  EXPECT_TRUE(last.cycle);
  ASSERT_EQ(last.outcome, CycleOutcome::kPublished)
      << rtc::catching::SegmentOutcomeName(last.segment.outcome) << " / "
      << last.segment.core_reason_name;
  EXPECT_TRUE(last.search_valid);
  EXPECT_TRUE(last.plan.valid);
  EXPECT_EQ(last.segment.outcome, SegmentOutcome::kPublished);
  EXPECT_EQ(last.segment.kind, SegmentKind::kFirst);
  // The clock stood at the wake's instant, and is back at zero outside a run.
  EXPECT_EQ(last.plan.publish_ns, last.now_ns);
  EXPECT_EQ(WakeClock(), 0);
  for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
    EXPECT_FALSE(rows[i].outcome == CycleOutcome::kPublished && rows[i].plan.valid) << i;
  }

  // The search of a whole wake decides what the search alone decides: the
  // search-only batch stops at the same wake with the same cells.
  SearchBatchSearch search = rig.BuildSearch();
  ASSERT_NE(search.search, nullptr) << search.error;
  const auto search_rows = RunSearchBatch(*search.search, search.q_rest, wakes);
  ASSERT_FALSE(search_rows.empty());
  ASSERT_TRUE(search_rows.back().plan.valid);
  ASSERT_LE(search_rows.size(), rows.size());
  const std::size_t at = search_rows.size() - 1;
  ASSERT_TRUE(rows[at].search_valid);
  if (rows[at].outcome == CycleOutcome::kPublished) {
    EXPECT_EQ(WithoutWallUs(SearchBatchCsvRow(rows[at], rig.nv)),
              WithoutWallUs(SearchBatchCsvRow(search_rows[at], rig.nv)));
  }
  for (std::size_t i = 0; i < at; ++i) {
    EXPECT_EQ(WithoutWallUs(SearchBatchCsvRow(rows[i], rig.nv)),
              WithoutWallUs(SearchBatchCsvRow(search_rows[i], rig.nv)))
        << i;
  }

  const auto cells = CycleCells(last, rig.nv);
  EXPECT_EQ(cells.at("search_valid"), "1");
  EXPECT_EQ(cells.at("outcome"), "published");
  EXPECT_EQ(cells.at("segment_outcome"), "published");
  EXPECT_EQ(cells.at("segment_kind"), "first");
  EXPECT_FALSE(cells.at("t_c_ns").empty());
  EXPECT_FALSE(cells.at("segment_catch_pos_err").empty());
}

TEST(CatchPlanBatch, AWithheldPairRunsEveryWakeAndLeavesThePlanCellsEmpty) {
  CycleRig rig;
  rig.catch_err_max = 1e-6;  // no solve lands the catch node that close
  PlanBatchBuild built = rig.Build();
  ASSERT_NE(built.batch, nullptr) << built.error;
  const auto wakes = rig.Catchable(4);
  const auto rows = built.batch->Run(wakes);
  ASSERT_EQ(rows.size(), wakes.size());
  for (const auto& row : rows) {
    EXPECT_FALSE(row.outcome == CycleOutcome::kPublished && row.plan.valid) << row.wake;
  }
  const SearchBatchRow& first = rows.front();
  ASSERT_TRUE(first.search_valid);
  EXPECT_EQ(first.outcome, CycleOutcome::kHeld);
  EXPECT_EQ(first.segment.outcome, SegmentOutcome::kCatchError);
  EXPECT_FALSE(first.plan.valid);
  const auto cells = CycleCells(first, rig.nv);
  EXPECT_EQ(cells.at("search_valid"), "1");
  EXPECT_EQ(cells.at("outcome"), "held");
  EXPECT_EQ(cells.at("segment_outcome"), "catch_error");
  EXPECT_EQ(cells.at("segment_kind"), "first");
  EXPECT_EQ(cells.at("t_c_ns"), "");
  EXPECT_EQ(cells.at("q_star0"), "");
  EXPECT_NE(cells.at("segment_catch_pos_err"), "nan");
}

TEST(CatchPlanBatch, AThrowsRowsDoNotDependOnTheThrowsBeforeIt) {
  const CycleRig rig;
  PlanBatchBuild built = rig.Build();
  ASSERT_NE(built.batch, nullptr) << built.error;
  const auto other =
      CycleRig::Throw(3, rig.p_c + Eigen::Vector3d(0.0, 0.02, -0.02), rig.v_ball * 0.9, 4);
  const auto far = CycleRig::Throw(2, rig.p_c + Eigen::Vector3d(10.0, 0.0, 0.0), rig.v_ball, 3);
  const auto forward = Concat({rig.Catchable(1), far, other});
  const auto backward = Concat({other, far, rig.Catchable(1)});
  const auto by_throw = [&](const std::vector<SearchBatchWake>& wakes) {
    std::map<std::int64_t, std::vector<std::string>> out;
    for (const auto& row : built.batch->Run(wakes)) {
      out[row.throw_id].push_back(WithoutWallUs(PlanBatchCsvRow(row, rig.nv)));
    }
    return out;
  };
  const auto a = by_throw(forward);
  const auto b = by_throw(backward);
  ASSERT_EQ(a.size(), 3U);
  EXPECT_EQ(a, b);
  EXPECT_EQ(a.at(2).size(), 3U) << "the unreachable throw ran every wake";
}

TEST(CatchPlanBatch, TheCsvKeepsTheSearchColumnsAndEndsWithWallUs) {
  const std::string search = SearchBatchCsvHeader(6);
  const std::string whole = PlanBatchCsvHeader(6);
  const std::string prefix = WithoutWallUs(search);
  ASSERT_EQ(whole.compare(0, prefix.size(), prefix), 0);
  const auto names = Split(whole);
  EXPECT_EQ(names.back(), "wall_us");
  EXPECT_EQ(names[Split(prefix).size()], "outcome");
  for (const char* name :
       {"segment_outcome", "segment_kind", "segment_core_reason", "segment_catch_pos_err"}) {
    EXPECT_NE(std::find(names.begin(), names.end(), name), names.end()) << name;
  }
}

// A first segment solved from the search's solution of a chain, widened
// (E3-F01): the batch says so beside `segment_from_search`, which stays 0 for
// it — the segment is the planner's own solve.
TEST(CatchPlanBatch, TheCsvSaysWhenTheSegmentStartedFromTheSearchsSolution) {
  const int nv = 6;
  const auto names = Split(PlanBatchCsvHeader(nv));
  const auto from = std::find(names.begin(), names.end(), "segment_from_search");
  ASSERT_NE(from, names.end());
  ASSERT_NE(from + 1, names.end());
  EXPECT_EQ(*(from + 1), "segment_start_from_search");
  SearchBatchRow row{};
  row.cycle = true;
  EXPECT_EQ(CycleCells(row, nv).at("segment_start_from_search"), "0");
  row.segment.start_from_search = true;
  const auto cells = CycleCells(row, nv);
  EXPECT_EQ(cells.at("segment_start_from_search"), "1");
  EXPECT_EQ(cells.at("segment_from_search"), "0");
  // ... and the other flag is still its own column.
  row.segment.start_from_search = false;
  row.segment.from_search = true;
  const auto adopted = CycleCells(row, nv);
  EXPECT_EQ(adopted.at("segment_from_search"), "1");
  EXPECT_EQ(adopted.at("segment_start_from_search"), "0");
}

TEST(CatchPlanBatchBinding, ReadsBothSegmentPlanners) {
  const CycleRig rig;
  const SearchBatchBinding mpc = ParseSearchBatchBinding(rig.BindingFile(), rig.nv);
  ASSERT_TRUE(mpc.segment.has_value());
  EXPECT_EQ(mpc.segment->kind, SegmentBatchKind::kMpc);
  EXPECT_EQ(mpc.segment->q_min.size(), 6U);
  EXPECT_EQ(mpc.segment->tau_max[3], 28.0);
  EXPECT_EQ(mpc.segment->eta_v, 0.9);
  EXPECT_EQ(mpc.segment->control_dt, 0.002);
  EXPECT_FALSE(ParseSearchBatchBinding(rig.BindingFile(false), rig.nv).segment.has_value());

  const SearchBatchBinding docking =
      ParseSearchBatchBinding(YAML::Load(std::string(kNlpBinding) + kDockingSegment), 3);
  ASSERT_TRUE(docking.segment.has_value());
  EXPECT_EQ(docking.segment->kind, SegmentBatchKind::kMpcDocking);
  EXPECT_EQ(docking.segment->t_arm_s, 0.05);
  EXPECT_EQ(docking.segment->qd_rating, (std::vector<double>{2.0, 2.0, 3.0}));
  EXPECT_TRUE(std::isnan(docking.segment->hand_t_close_lead));  // a TBD value stays one
  EXPECT_EQ(docking.segment->limits.qdd_max.size(), 0);
  EXPECT_EQ(docking.segment->limits.tau_lo[1], -18.0);
  ASSERT_EQ(docking.segment->limits.armature.size(), 3);
}

TEST(CatchPlanBatchBinding, TheSegmentMapIsFailClosed) {
  const CycleRig rig;
  const auto refused = [&](const std::string& edit_from, const std::string& edit_to) {
    std::ostringstream text;
    text << rig.BindingFile();
    std::string s = text.str();
    const std::size_t at = s.rfind(edit_from);
    EXPECT_NE(at, std::string::npos) << edit_from;
    if (at == std::string::npos) {
      return false;
    }
    s.replace(at, edit_from.size(), edit_to);
    try {
      static_cast<void>(ParseSearchBatchBinding(YAML::Load(s), rig.nv));
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(refused("eta_v: 0.9", "eta_v: 0.8"));
  EXPECT_FALSE(refused("eta_v: 0.9", "eta_v: .nan")) << "a TBD scalar is a value";
  EXPECT_TRUE(refused("mode: mpc", "mode: closed_form"));
  EXPECT_TRUE(refused("mode: mpc", "mode: mpc_docking")) << "the selected planner's map is absent";
  EXPECT_TRUE(refused("eta_v: 0.9", "eta_vv: 0.9")) << "an unknown key, and a missing one";
  EXPECT_TRUE(refused("tau_max: [150, ", "tau_max: ["));
  EXPECT_TRUE(refused("tau_max: [150, ", "tau_max: [.nan, "));
  EXPECT_TRUE(refused("segment:", "segmentt:")) << "an unknown top-level key";
}

TEST(CatchPlanBatchBinding, ABindingOrATreeWithoutTheSegmentPlannerIsRefused) {
  const CycleRig rig;
  EXPECT_THROW(
      static_cast<void>(MakePlanBatch(rig.model, *rig.handle, rig.frame, rig.Tree(),
                                      ParseSearchBatchBinding(rig.BindingFile(false), rig.nv))),
      std::invalid_argument);
  YAML::Node tree = rig.Tree();
  tree["planner"]["segment"].remove("mpc");
  EXPECT_THROW(static_cast<void>(MakePlanBatch(rig.model, *rig.handle, rig.frame, tree,
                                               ParseSearchBatchBinding(rig.BindingFile(), rig.nv))),
               std::invalid_argument);
}

TEST(CatchPlanBatch, AProfileAControllerParksIsRefusedInThatControllersWords) {
  // The grid search with the mpc_docking planner on a tree that has the
  // planner's map and none of the hand's identified capture set: a controller's
  // configure parks on it (docking_config.hpp), and the batch judges nothing.
  const CycleRig rig;
  YAML::Node binding = rig.BindingFile(false);
  binding["segment"] = YAML::Load(
      "mode: mpc_docking\n"
      "mpc_docking:\n"
      "  t_arm_s: 0.0\n  control_dt: 0.002\n  qd_rating: [2, 2, 3, 3, 3, 3]\n"
      "  hand_t_close_e2e: 0.2\n  hand_t_close_lead: 0.05\n  ball_mass: 0.057\n"
      "  limits:\n    q_min: [-6, -6, -3, -6, -6, -6]\n    q_max: [6, 6, 3, 6, 6, 6]\n"
      "    qd_max: [2, 2, 3, 3, 3, 3]\n    qdd_max: []\n"
      "    tau_max: [150, 150, 150, 28, 28, 28]\n"
      "    tau_lo: [-150, -150, -150, -28, -28, -28]\n"
      "    tau_hi: [150, 150, 150, 28, 28, 28]\n");
  YAML::Node tree = rig.Tree();
  tree["planner"]["segment"]["mode"] = "mpc_docking";
  tree["planner"]["segment"]["mpc_docking"] = YAML::Load("{approach: {n_pre_max: 6}}");
  const PlanBatchBuild built = MakePlanBatch(rig.model, *rig.handle, rig.frame, tree,
                                             ParseSearchBatchBinding(binding, rig.nv));
  EXPECT_EQ(built.batch, nullptr);
  EXPECT_NE(built.error.find("'catching.robot.hand.docking.s_ent' is unset or TBD"),
            std::string::npos)
      << built.error;
}

TEST(CatchPlanBatch, TheWakeClockIsZeroAgainAfterARunThatThrew) {
  const CycleRig rig;
  const PlanBatchBuild built = rig.Build();
  ASSERT_NE(built.batch, nullptr) << built.error;
  // A second throw whose wake has no sample is refused — after the first ran
  // and set the clock.
  std::vector<SearchBatchWake> wakes = rig.Catchable(1, 1);
  wakes.push_back(rig.Catchable(2, 1).front());
  wakes.back().samples.clear();
  EXPECT_THROW(static_cast<void>(built.batch->Run(wakes)), std::invalid_argument);
  EXPECT_EQ(rtc::catching::WakeClock(), 0);
}

}  // namespace
