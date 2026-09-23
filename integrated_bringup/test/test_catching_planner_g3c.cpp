// ── G3-C: 1000 synthetic throws through the planner (dynamic_catching S6-C) ──
//
// The search as it ships: the real UR5e+P1b catch sub-model (`ur5e_catch`,
// R-3), the shipped ur5e_p1b planner profile (parsed from the file that
// ships, not restated), the shipped IK options and D-16 box. Each throw is a
// ballistic prediction that passes through a catch point the arm CAN reach
// (the catch frame of a configuration drawn around the wait pose and redrawn
// until the point lies inside the shipped catch_box, the ball coming into the
// palm), with the time to the catch point and the ball speed drawn at random.
// The arm starts at rest at the shipped wait pose. Many of these throws are too
// fast or too far for the reach and γ rank gates: that is on purpose, since the
// timing must cover the candidates that get as far as IK and the rollout.
//
// G3-C asks two things and this suite answers both:
//   * the planner's cycle time: p99 < `planner.budget_s` — ASSERTED;
//   * the rejection-reason distribution — RECORDED (RecordProperty, integer
//     counts), together with the plan rate and the rank-gate failure counts of
//     the chosen candidates. Those are measurements of the design under D-27,
//     not pass/fail criteria (S8 reports attempts against successes).
//
// Timing is on THIS host (the development PC); the control PC's numbers are
// S6-D's (D-7a). The planner runs on the test thread, not a FIFO thread.

#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/planner_search.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::catching::CovarianceSnapshot;
using rtc::catching::JudgeReject;
using rtc::catching::NowReal;
using rtc::catching::PlannerConstants;
using rtc::catching::PlannerModel;
using rtc::catching::PlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlannerSearch;
using rtc::catching::SearchStats;
using rtc::catching::TrajectorySnapshot;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 50'000 * kMs;
/// Half-width [rad] of the catch configurations drawn around the wait pose.
constexpr double kQSpread = 1.2;

std::int64_t SteadyClock() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct Shipped {
  YAML::Node controller;
  PlannerParams params;
  rtc::catching::CatchPoseIkConfig ik;
  std::vector<double> qdd_max;  // D-16 box, device order
};

Shipped LoadShippedP1b() {
  Shipped s;
  s.controller =
      integrated_bringup::testfx::ShippedControllerNode("ur5e_p1b", "demo_catching_controller");
  s.params = rtc::catching::ParsePlannerParams(s.controller["catching"]);
  s.ik = rtc::catching::ParseCatchPoseIkParams(s.controller["catching"]);
  const std::string path =
      ament_index_cpp::get_package_share_directory("integrated_bringup") + "/" +
      s.controller["catching"]["robot"]["arm"]["accel_limits_path"].as<std::string>();
  const YAML::Node doc = YAML::LoadFile(path);
  s.qdd_max = doc["derived_accel_limits"]["ur5e"]["qdd_max"].as<std::vector<double>>();
  return s;
}

TEST(PlannerG3C, OneThousandSyntheticThrowsStayInsideTheBudget) {
  const Shipped shipped = LoadShippedP1b();
  ASSERT_TRUE(shipped.params.enabled);
  ASSERT_EQ(shipped.params.sub_model, "ur5e_catch");

  // The catch sub-model, exactly as the binding builds it.
  auto builder = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>([] {
    auto cfg = integrated_bringup::testfx::MakeUr5eP1bModelConfig();
    rtc_urdf_bridge::ExtraFrameConfig frame;
    frame.name = "catch_frame";
    frame.parent = "l_palm_link";
    frame.xyz = Eigen::Vector3d(0.015, 0.145, 0.052);
    frame.provisional = false;
    cfg.extra_frames.push_back(frame);
    return cfg;
  }());
  const auto model = builder->GetReducedModel("ur5e_catch");
  ASSERT_TRUE(model);
  auto handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model);
  const auto frame = rtc::catching::ResolveCatchFrame(*model, "catch_frame");
  const int nv = model->nv;
  ASSERT_EQ(nv, 6);
  const auto arm_names =
      integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names;

  PlannerModel pm;
  pm.handle = handle.get();
  pm.catch_frame = frame;
  pm.nv = nv;
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(model->njoints);
       ++jid) {
    const auto it = std::find(arm_names.begin(), arm_names.end(), model->names[jid]);
    ASSERT_NE(it, arm_names.end()) << model->names[jid];
    const auto q = static_cast<std::size_t>(model->joints[jid].idx_q());
    const auto d = static_cast<std::size_t>(std::distance(arm_names.begin(), it));
    pm.device_of_model[q] = static_cast<int>(d);
    pm.qdot_max[q] = 3.1416;  // the ratings the S3.5b map used
    pm.qddot_max[q] = shipped.qdd_max.at(d);
  }
  pm.accel_box = true;

  const YAML::Node c = shipped.controller["catching"];
  PlannerConstants pc;
  pc.eta_v = c["planner"]["gamma"]["eta_v"].as<double>();
  pc.v_max = c["reference"]["v_max"].as<double>();
  pc.a_dec = c["supervisor"]["decel"]["a_dec"].as<double>();
  pc.t_arm_s = c["joint_cmd"]["lag"]["T_arm"].as<double>();
  pc.t_close_e2e = c["robot"]["hand"]["T_close_e2e"].as<double>();
  pc.t_close_total = pc.t_close_e2e + 0.001;
  pc.ball_mass = c["core"]["ball"]["mass"].as<double>();
  pc.ref_omega = c["reference"]["omega"].as<double>();
  pc.ref_zeta = c["reference"]["zeta"].as<double>();
  pc.ref_a_max = c["reference"]["a_max"].as<double>();
  pc.control_dt = 0.002;

  PlannerSearch search;
  ASSERT_TRUE(search.Configure(pm, pc, shipped.params, shipped.ik.options, &SteadyClock));

  std::mt19937 rng(537);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  constexpr int kThrows = 1000;
  std::vector<std::int64_t> cycle_us;
  cycle_us.reserve(kThrows);
  std::array<long, rtc::catching::kJudgeRejectCount> rejects{};
  std::array<long, 6> rank_fail{};
  long plans = 0;
  long budget_hits = 0;
  long window_only = 0;
  auto traj = std::make_unique<TrajectorySnapshot>();
  auto cov = std::make_unique<CovarianceSnapshot>();
  for (int trial = 0; trial < kThrows; ++trial) {
    // A reachable catch configuration whose catch frame lies inside the
    // shipped catch_box: a throw the box refuses outright never reaches the
    // search and would time nothing (the first draft of this test measured a
    // 1 us median that way). Drawn around the wait pose, clamped to the limits.
    Eigen::VectorXd q(nv);
    Eigen::Vector3d p_star = Eigen::Vector3d::Zero();
    bool in_box = false;
    for (int attempt = 0; attempt < 500 && !in_box; ++attempt) {
      for (int j = 0; j < nv; ++j) {
        const double seed = shipped.params.wait_pose[static_cast<std::size_t>(
            pm.device_of_model[static_cast<std::size_t>(j)])];
        const double lo = model->lowerPositionLimit[j];
        const double hi = model->upperPositionLimit[j];
        q[j] = std::clamp(seed + kQSpread * (2.0 * uni(rng) - 1.0), lo, hi);
      }
      handle->ComputeForwardKinematics(
          std::span<const double>(q.data(), static_cast<std::size_t>(nv)));
      p_star = handle->GetFramePosition(frame);
      in_box = shipped.params.catch_box.Contains(p_star.x(), p_star.y(), p_star.z());
    }
    ASSERT_TRUE(in_box) << "no catch configuration inside catch_box after 500 draws";
    const Eigen::Vector3d z = handle->GetFrameRotation(frame).col(2);
    const double speed = 2.0 + 4.0 * uni(rng);
    const Eigen::Vector3d v_c = -speed * z;   // into the palm
    const double t_c = 0.4 + 0.5 * uni(rng);  // seconds from now

    // 20 samples at 50 ms: the shipped vision profile's shape.
    traj->valid = true;
    traj->n = 20;
    traj->token.activation_generation = 1;
    traj->token.generation = static_cast<std::uint64_t>(trial + 1);
    traj->token.snapshot_sequence = 10;
    traj->token.traj_recv_ns = kNow - 5 * kMs;
    for (int k = 0; k < traj->n; ++k) {
      const double tk = 0.05 * k;
      const double dt = tk - t_c;
      const Eigen::Vector3d p = p_star + v_c * dt + 0.5 * g * dt * dt;
      const Eigen::Vector3d v = v_c + g * dt;
      auto& s = traj->s[static_cast<std::size_t>(k)];
      s.t_ns = kNow + static_cast<std::int64_t>(std::llround(tk * 1e9));
      s.p = {p.x(), p.y(), p.z()};
      s.v = {v.x(), v.y(), v.z()};
      s.a = {g.x(), g.y(), g.z()};
    }
    cov->valid = true;
    cov->n = traj->n;
    cov->token = traj->token;
    for (int k = 0; k < traj->n; ++k) {
      auto& e = cov->c[static_cast<std::size_t>(k)];
      e.fill(0.0);
      const double sp = 0.003 + 0.004 * (k / 20.0);  // growing with the horizon
      for (int d = 0; d < 6; ++d) {
        e[static_cast<std::size_t>(d * 6 + d)] = sp * sp;
      }
    }

    PlannerRtState rt{};
    rt.valid = true;
    rt.activation_generation = 1;
    rt.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kTracking);
    rt.nv = nv;
    // The arm starts at the shipped wait pose, at rest.
    for (int j = 0; j < nv; ++j) {
      rt.q_cmd[static_cast<std::size_t>(j)] = shipped.params.wait_pose[static_cast<std::size_t>(j)];
    }
    search.ResetTrial();
    // n_settle snapshots first (each a new sequence), then the measured one.
    SearchStats stats;
    for (int s = 0; s <= shipped.params.n_settle; ++s) {
      traj->token.snapshot_sequence = static_cast<std::uint64_t>(10 + s);
      cov->token = traj->token;
      static_cast<void>(search.Plan(*traj, *cov, true, rt, NowReal{kNow}, stats));
    }
    ASSERT_FALSE(stats.settling);
    const auto plan = search.Plan(*traj, *cov, true, rt, NowReal{kNow}, stats);
    // The cycle as timed by the search itself (its own clock reads).
    cycle_us.push_back(stats.search_ns / 1000);
    plans += plan.valid ? 1 : 0;
    budget_hits += stats.budget_hit ? 1 : 0;
    for (std::size_t r = 0; r < rejects.size(); ++r) {
      rejects[r] += stats.judge_rejects[r];
    }
    if (plan.valid) {
      for (int b = 0; b < 6; ++b) {
        rank_fail[static_cast<std::size_t>(b)] += (stats.chosen_rank_mask >> b) & 1U;
      }
      window_only += stats.chosen_rollout_window_only ? 1 : 0;
    }
  }

  std::vector<std::int64_t> sorted = cycle_us;
  std::sort(sorted.begin(), sorted.end());
  const auto pct = [&sorted](double p) {
    return sorted[static_cast<std::size_t>(p * static_cast<double>(sorted.size() - 1))];
  };
  const std::int64_t budget_us =
      static_cast<std::int64_t>(std::llround(shipped.params.budget_s * 1e6));
  RecordProperty("budget_us", static_cast<int>(budget_us));
  RecordProperty("cycle_us_p50", static_cast<int>(pct(0.5)));
  RecordProperty("cycle_us_p99", static_cast<int>(pct(0.99)));
  RecordProperty("cycle_us_max", static_cast<int>(sorted.back()));
  RecordProperty("plans", static_cast<int>(plans));
  RecordProperty("budget_hits", static_cast<int>(budget_hits));
  RecordProperty("rej_input", static_cast<int>(rejects[1]));
  RecordProperty("rej_ik", static_cast<int>(rejects[2]));
  RecordProperty("rej_manipulability", static_cast<int>(rejects[3]));
  RecordProperty("rej_workspace", static_cast<int>(rejects[4]));
  RecordProperty("rej_not_evaluated", static_cast<int>(rejects[5]));
  RecordProperty("rank_uncertainty", static_cast<int>(rank_fail[0]));
  RecordProperty("rank_reach", static_cast<int>(rank_fail[1]));
  RecordProperty("rank_gamma", static_cast<int>(rank_fail[2]));
  RecordProperty("rank_commit_lead", static_cast<int>(rank_fail[3]));
  RecordProperty("rank_error_budget", static_cast<int>(rank_fail[4]));
  RecordProperty("rank_rollout", static_cast<int>(rank_fail[5]));
  RecordProperty("rollout_window_only", static_cast<int>(window_only));

  EXPECT_LT(pct(0.99), budget_us) << "G3-C: the planner's p99 cycle exceeds planner.budget_s";
  EXPECT_GT(plans, 0) << "not one of 1000 reachable throws produced a plan";
}

}  // namespace
