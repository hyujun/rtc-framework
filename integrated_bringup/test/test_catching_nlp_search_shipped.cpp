// NLP catch search on the SHIPPED catch sub-models: what one wake costs
// (E1-F14, #740).
//
// rtc_controllers' own suite runs the search on a synthetic 6R, with a clock
// that steps on every read, to pin WHAT it decides. How LONG it takes depends
// on the arm — joint count, and the hand's mass in the torque rows — so it is
// recorded here, on `ur5e_catch` and `iiwa7_catch` with their shipped catch
// frame and joint ratings, against the real clock.
//
// RECORDED, not judged. The capture-set parameters are the docking fixture's
// synthetic ones (the identified values are E1-F15's) and the host is whatever
// this runs on, so a threshold here would be a verdict on made-up numbers on an
// arbitrary machine. What IS asserted is only that the search configures on
// the shipped models, that the wakes that are timed did the work they are
// named for (solves ran, of the named kind), and that a wake returns.
//
// What is recorded, per arm (gtest properties, integers):
//   • Configure: wall time, resident memory it added.
//   • one candidate's screening (catch-pose IK included) and the IK alone.
//   • a COLD solve (no memory: from the IK pose) and a WARM one (the same
//     candidate's previous solution), by the number of pre-catch intervals —
//     the QP grows with the node count.
//   • a whole wake with the solves capped at 1, 2, 4 and 8.
//   • how far past its share a solve runs when the share is short: the core
//     reads its deadline between iterations only.
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "shipped_catch_arm_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace dk = rtc::testing::mpc_docking;
using integrated_bringup::testfx::ShippedArm;
using integrated_bringup::testfx::ShippedArms;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::NlpCatchSearch;
using rtc::catching::NlpCatchSearchConstants;
using rtc::catching::NlpCatchSearchModel;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::NlpReject;
using rtc::catching::NlpRejectName;
using rtc::catching::NowReal;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::ReportedSegments;
using rtc::catching::SearchStats;
using rtc::catching::TrajectorySnapshot;
using Candidate = NlpCatchSearch::CandidateRecord;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 10'000 * kMs;
constexpr double kBudgetS = 2.0;
// The ball is on the home pose's entrance plane this long after a wake's t_0:
// mid-window, so candidates on both sides of it are solved.
constexpr double kStarAfterT0S = 0.55;

std::int64_t SteadyNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Resident set size [kB] (statm's second field, in pages).
long ResidentKb() {
  long pages_total = 0;
  long pages_resident = 0;
  if (std::FILE* f = std::fopen("/proc/self/statm", "r")) {
    if (std::fscanf(f, "%ld %ld", &pages_total, &pages_resident) != 2) {
      pages_resident = 0;
    }
    std::fclose(f);
  }
  return pages_resident * (sysconf(_SC_PAGESIZE) / 1024);
}

struct Rig {
  ShippedArm arm;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  NlpCatchSearchModel model{};
  NlpCatchSearchConstants constants{};
  NlpCatchSearchParams params{};
  CatchPoseIkOptions ik{};
  std::unique_ptr<NlpCatchSearch> search = std::make_unique<NlpCatchSearch>();
  std::array<double, 8> home_device{};

  explicit Rig(ShippedArm shipped) : arm(std::move(shipped)) {
    const int nv = static_cast<int>(arm.rig.model.nv);
    handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(arm.rig.arm.model);
    model.arm = arm.rig.arm.model;
    model.handle = handle.get();
    model.catch_frame = arm.rig.arm.frame;
    model.nv = nv;
    for (int m = 0; m < nv; ++m) {
      const auto u = static_cast<std::size_t>(m);
      model.device_of_model[u] = arm.device_of_model[u];
      home_device[static_cast<std::size_t>(arm.device_of_model[u])] = arm.rig.arm.q_nominal[m];
    }
    constants.t_arm_s = 0.0;
    constants.control_dt = 0.002;

    // One candidate per vision period; the shipped segment grid — 0.1 s
    // pre-catch intervals, a 7 × 0.05 s stop in blocks 1, 1, 2, 3 — with as
    // many pre-catch intervals as the grid search's shipped horizon (0.95 s)
    // needs, so that the node count's effect is visible up to it.
    params.cand_dt = 1.0 / 30.0;
    params.dt_pre = 0.1;
    params.n_pre_min = 2;
    params.n_pre_max = 10;
    params.t_lead_min = 0.2;
    params.t_max = 0.95;
    params.cand_capacity = 32;
    params.n_stop = 7;
    params.dt_stop = 0.05;
    params.n_stop_blocks = 4;
    params.stop_block_sizes = {1, 1, 2, 3};
    params.catch_box.set = true;
    params.catch_box.min = {-5.0, -5.0, -5.0};
    params.catch_box.max = {5.0, 5.0, 5.0};
    params.wait_pose_n = nv;
    std::copy(home_device.begin(), home_device.begin() + nv, params.wait_pose.begin());
    // A budget that cuts nothing — nine shares of 0.2 s: the timings below are
    // of the work. The wake's start instant t_0 = now + budget is then 2 s
    // ahead, which the prediction below reaches.
    params.budget_s = kBudgetS;
    params.solve_budget_s = 0.2;
    params.start_lead_s = 0.004;
    params.max_solves = 8;
    params.core = arm.rig.params;
    params.limits = arm.rig.limits;
    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
  }

  [[nodiscard]] bool Configure(std::string* error) {
    search = std::make_unique<NlpCatchSearch>();
    return search->Configure(model, constants, params, ik, &SteadyNs, error);
  }

  [[nodiscard]] PlannerRtState RestingRt(std::int64_t now) const {
    PlannerRtState rt = rtc::testing::TrackingRtState(
        3, std::span<const double>(home_device.data(), static_cast<std::size_t>(model.nv)));
    rt.rt_iteration = 500;
    rt.rt_state_ns = now - kMs;
    return rt;
  }
};

struct Throw {
  TrajectorySnapshot traj;
  CovarianceSnapshot cov;
};

// A ball down the home pose's capture axis at 0.8 m/s, on its entrance plane
// `t_star_s` after kNow. 20 samples 200 ms apart: 3.8 s of prediction.
Throw AxisThrow(const Rig& rig, double t_star_s) {
  const int nv = rig.model.nv;
  const dk::HandState h =
      dk::HandAt(rig.arm.rig, rig.arm.rig.arm.q_nominal, Eigen::VectorXd::Zero(nv));
  const Eigen::Vector3d v_b = -0.8 * h.R.col(2);
  const Eigen::Vector3d p_e = h.p + h.R * Eigen::Vector3d(0.0, 0.0, rig.params.core.s_ent);
  Throw t;
  t.traj = rtc::testing::LineTrajectory(p_e - v_b * t_star_s, v_b, kNow, 200 * kMs, 20, 0, 1, 7, 3,
                                        kNow - 5 * kMs);
  t.cov = rtc::testing::IsotropicCovariance(t.traj, 0.002);
  return t;
}

// Median and largest of a set of durations, as two integer properties [µs].
void RecordUs(const std::string& name, std::vector<std::int64_t> ns) {
  if (ns.empty()) {
    ::testing::Test::RecordProperty(name + "_n", 0);
    return;
  }
  std::sort(ns.begin(), ns.end());
  ::testing::Test::RecordProperty(name + "_n", static_cast<int>(ns.size()));
  ::testing::Test::RecordProperty(name + "_median_us", static_cast<int>(ns[ns.size() / 2] / 1000));
  ::testing::Test::RecordProperty(name + "_max_us", static_cast<int>(ns.back() / 1000));
}

const ReportedSegments& NoSegments() {
  static const ReportedSegments none{};
  return none;
}

TEST(NlpSearchShipped, RecordsConfigureAndSolveTimesByNodeCount) {
  for (ShippedArm& shipped : ShippedArms()) {
    ASSERT_TRUE(shipped.rig.arm.model);
    const std::string tag = shipped.rig.arm.name;
    SCOPED_TRACE(tag);
    Rig rig(std::move(shipped));
    // ── Configure: every core, the IK, the memory, one warm-up solve each ──
    const long rss_before = ResidentKb();
    const std::int64_t t_configure = SteadyNs();
    std::string err;
    ASSERT_TRUE(rig.Configure(&err)) << err;
    const std::int64_t configure_ns = SteadyNs() - t_configure;
    RecordProperty(tag + "_configure_ms", static_cast<int>(configure_ns / kMs));
    RecordProperty(tag + "_configure_rss_added_kb", static_cast<int>(ResidentKb() - rss_before));
    RecordProperty(tag + "_cores", rig.params.n_pre_max - rig.params.n_pre_min + 1);

    // ── Cold and warm solves, by the number of pre-catch intervals ─────────
    // Wake 1 of a trial has no memory: every solve is cold. The wakes after it
    // start each candidate from its own previous solution.
    std::map<int, std::vector<std::int64_t>> cold;
    std::map<int, std::vector<std::int64_t>> warm;
    std::map<int, std::vector<std::int64_t>> cold_iterations;
    std::map<int, std::vector<std::int64_t>> warm_iterations;
    // Where a solve's time goes: in the QP solver, and before the first
    // iterate [ns], and how many QPs it ran.
    std::vector<std::int64_t> cold_qp, warm_qp, cold_start, warm_start, cold_qps, warm_qps;
    std::vector<std::int64_t> cold_all, warm_all;
    std::vector<std::int64_t> screen_per_candidate;
    std::vector<std::int64_t> ik_max;
    std::map<std::string, int> reasons;
    constexpr int kTrials = 6;
    constexpr int kWakes = 6;
    // Three throws — the ball on the capture axis early, in the middle and
    // late in the candidate window — so that the solves the rank picks (the
    // candidates nearest the home pose) fall on short, medium and long grids.
    const std::array<Throw, 3> balls{AxisThrow(rig, kBudgetS + 0.30),
                                     AxisThrow(rig, kBudgetS + kStarAfterT0S),
                                     AxisThrow(rig, kBudgetS + 0.85)};
    for (int trial = 0; trial < kTrials; ++trial) {
      const Throw& ball = balls[static_cast<std::size_t>(trial) % balls.size()];
      rig.search->ResetTrial();
      for (int wake = 0; wake < kWakes; ++wake) {
        // Each trial starts a few ms later, so that the candidates fall on
        // other phases of the grid and every core is used.
        const std::int64_t now = kNow + (trial * 7 + wake * 33) * kMs;
        SearchStats stats;
        const PlanSnapshot plan = rig.search->Plan(ball.traj, ball.cov, true, rig.RestingRt(now),
                                                   NoSegments(), NowReal{now}, stats);
        static_cast<void>(plan);
        if (stats.n_ik > 0) {
          screen_per_candidate.push_back(stats.nlp.screen_ns / stats.n_ik);
          ik_max.push_back(stats.ik_ns_max);
        }
        for (const Candidate& c : rig.search->Candidates()) {
          ++reasons[NlpRejectName(c.reject)];
          if (c.rank < 0 || c.solve_ns == 0) {
            continue;
          }
          const bool is_cold = c.start == NlpCatchSearch::Start::kIkTarget;
          const bool is_warm = c.start == NlpCatchSearch::Start::kSameCandidate;
          const auto us = [](double v) { return static_cast<std::int64_t>(v * 1e3); };
          if (is_cold) {
            cold[c.n_pre].push_back(c.solve_ns);
            cold_iterations[c.n_pre].push_back(c.iterations);
            cold_all.push_back(c.solve_ns);
            cold_qp.push_back(us(c.qp_us));
            cold_start.push_back(us(c.start_us));
            cold_qps.push_back(c.qp_solves);
          } else if (is_warm) {
            warm[c.n_pre].push_back(c.solve_ns);
            warm_iterations[c.n_pre].push_back(c.iterations);
            warm_all.push_back(c.solve_ns);
            warm_qp.push_back(us(c.qp_us));
            warm_start.push_back(us(c.start_us));
            warm_qps.push_back(c.qp_solves);
          }
        }
        if (wake == 0) {
          // The first wake solved, and from nothing.
          EXPECT_GT(stats.nlp.n_solved, 0) << tag;
        }
      }
    }
    RecordUs(tag + "_cold_solve", cold_all);
    RecordUs(tag + "_cold_solve_in_qp", cold_qp);
    RecordUs(tag + "_cold_solve_before_first_iterate", cold_start);
    RecordUs(tag + "_warm_solve", warm_all);
    RecordUs(tag + "_warm_solve_in_qp", warm_qp);
    RecordUs(tag + "_warm_solve_before_first_iterate", warm_start);
    std::sort(cold_qps.begin(), cold_qps.end());
    std::sort(warm_qps.begin(), warm_qps.end());
    if (!cold_qps.empty() && !warm_qps.empty()) {
      RecordProperty(tag + "_cold_solve_median_qps",
                     static_cast<int>(cold_qps[cold_qps.size() / 2]));
      RecordProperty(tag + "_warm_solve_median_qps",
                     static_cast<int>(warm_qps[warm_qps.size() / 2]));
    }
    RecordUs(tag + "_screen_per_candidate", screen_per_candidate);
    RecordUs(tag + "_ik_slowest_of_a_wake", ik_max);
    for (const auto& [name, count] : reasons) {
      RecordProperty(tag + "_candidates_" + name, count);
    }
    EXPECT_FALSE(cold.empty()) << tag << ": no cold solve was timed";
    EXPECT_FALSE(warm.empty()) << tag << ": no warm solve was timed";
    for (const auto& [n_pre, ns] : cold) {
      const std::string name = tag + "_cold_n_pre_" + std::to_string(n_pre);
      RecordUs(name, ns);
      std::vector<std::int64_t> it = cold_iterations[n_pre];
      std::sort(it.begin(), it.end());
      RecordProperty(name + "_median_iterations", static_cast<int>(it[it.size() / 2]));
    }
    for (const auto& [n_pre, ns] : warm) {
      const std::string name = tag + "_warm_n_pre_" + std::to_string(n_pre);
      RecordUs(name, ns);
      std::vector<std::int64_t> it = warm_iterations[n_pre];
      std::sort(it.begin(), it.end());
      RecordProperty(name + "_median_iterations", static_cast<int>(it[it.size() / 2]));
    }
  }
}

TEST(NlpSearchShipped, RecordsAWholeWakeByTheNumberOfSolves) {
  for (ShippedArm& shipped : ShippedArms()) {
    ASSERT_TRUE(shipped.rig.arm.model);
    const std::string tag = shipped.rig.arm.name;
    SCOPED_TRACE(tag);
    Rig rig(std::move(shipped));
    const Throw ball = AxisThrow(rig, kBudgetS + kStarAfterT0S);
    for (const int solves : {1, 2, 4, 8}) {
      rig.params.max_solves = solves;
      std::string err;
      ASSERT_TRUE(rig.Configure(&err)) << err;
      std::vector<std::int64_t> cold_wake;
      std::vector<std::int64_t> warm_wake;
      std::vector<std::int64_t> screen;
      int solved_min = solves;
      constexpr int kTrials = 5;
      for (int trial = 0; trial < kTrials; ++trial) {
        rig.search->ResetTrial();
        for (int wake = 0; wake < 4; ++wake) {
          const std::int64_t now = kNow + (trial * 7 + wake * 33) * kMs;
          SearchStats stats;
          const PlanSnapshot plan = rig.search->Plan(ball.traj, ball.cov, true, rig.RestingRt(now),
                                                     NoSegments(), NowReal{now}, stats);
          static_cast<void>(plan);
          (wake == 0 ? cold_wake : warm_wake).push_back(stats.search_ns);
          screen.push_back(stats.nlp.screen_ns);
          solved_min = std::min<int>(solved_min, stats.nlp.n_solved);
        }
      }
      // The cap was what limited the wake: it ran that many solves.
      EXPECT_EQ(solved_min, solves) << tag;
      const std::string name = tag + "_wake_" + std::to_string(solves) + "_solves";
      RecordUs(name + "_cold", cold_wake);
      RecordUs(name + "_warm", warm_wake);
      RecordUs(name + "_screening", screen);
    }
  }
}

TEST(NlpSearchShipped, RecordsHowFarASolveRunsPastAShortShare) {
  // A share far below what a cold solve takes. The core reads its deadline
  // between iterations, so a solve ends one iteration after its share ran out
  // at the latest — by how much is that iteration's cost.
  constexpr std::int64_t kShareNs = 2 * kMs;
  for (ShippedArm& shipped : ShippedArms()) {
    ASSERT_TRUE(shipped.rig.arm.model);
    const std::string tag = shipped.rig.arm.name;
    SCOPED_TRACE(tag);
    Rig rig(std::move(shipped));
    rig.params.solve_budget_s = static_cast<double>(kShareNs) / 1e9;
    std::string err;
    ASSERT_TRUE(rig.Configure(&err)) << err;
    const Throw ball = AxisThrow(rig, kBudgetS + kStarAfterT0S);
    std::vector<std::int64_t> overrun;
    int solved = 0;
    for (int trial = 0; trial < 5; ++trial) {
      rig.search->ResetTrial();
      const std::int64_t now = kNow + trial * 7 * kMs;
      SearchStats stats;
      static_cast<void>(rig.search->Plan(ball.traj, ball.cov, true, rig.RestingRt(now),
                                         NoSegments(), NowReal{now}, stats));
      solved += stats.nlp.n_solved;
      for (const Candidate& c : rig.search->Candidates()) {
        if (c.reject == NlpReject::kDeadline) {
          overrun.push_back(c.solve_ns - kShareNs);
        }
      }
    }
    EXPECT_GT(solved, 0) << tag;
    RecordProperty(tag + "_short_share_us", static_cast<int>(kShareNs / 1000));
    RecordProperty(tag + "_short_share_solves", solved);
    RecordUs(tag + "_past_the_share", overrun);
  }
}

}  // namespace
