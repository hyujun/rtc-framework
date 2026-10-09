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
//
// Drives the grid search on the 6R wrist arm, built through the binding the
// executable uses.
#include "rtc_controllers/catching/catch_search_batch.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rtc::catching::kCap;
using rtc::catching::MakeSearchBatchInputs;
using rtc::catching::MakeSearchBatchSearch;
using rtc::catching::NowReal;
using rtc::catching::ParseSearchBatchBinding;
using rtc::catching::ParseSearchWakeCsv;
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
using rtc::catching::TrajSample;

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
    EXPECT_EQ(SearchBatchCsvRow(row, rig.arm.nv), SearchBatchCsvRow(rows[i], rig.arm.nv)) << i;
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
      out[row.throw_id].push_back(SearchBatchCsvRow(row, rig.arm.nv));
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

}  // namespace
