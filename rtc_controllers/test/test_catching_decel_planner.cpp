// E1-F03 (#629): the decel MPC's `planner.decel_mpc.*` keys and the RT-side
// admission and switch rules (planner_io.hpp). The planner itself — the
// first segment, the replans and the cycle that publishes them — is tested in
// test_catching_approach_planner.cpp and test_catching_approach_cycle.cpp
// (E1-F08); the stop-only planner E1-F03 shipped was removed with its tests
// (MD-70).
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace {

using rtc::catching::AdmittedDecel;
using rtc::catching::ChooseDecelSegment;
using rtc::catching::DecelAdmissionContext;
using rtc::catching::DecelBlocksFor;
using rtc::catching::DecelPlannerParams;
using rtc::catching::DecelPlanSnapshot;
using rtc::catching::DecelRefusal;
using rtc::catching::DecelSegmentChoice;
using rtc::catching::DecelSwitchVerdict;
using rtc::catching::JudgeDecelPlan;
using rtc::catching::JudgeDecelSwitch;
using rtc::catching::kMaxDecelNodes;
using rtc::catching::kMaxDecelNv;
using rtc::catching::Mode;
using rtc::catching::NowReal;
using rtc::catching::ParsePlannerParams;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kDt = 25 * kMs;                     // Δ_s (MD-24)
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;  // realistic steady ns
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

std::size_t Idx(int k, int j) {
  return static_cast<std::size_t>(k * kMaxDecelNv + j);
}

// ── 1. planner.decel_mpc.* ───────────────────────────────────────────────────

TEST(DecelParams, DefaultsAreTheShippedHorizon) {
  const DecelPlannerParams d = rtc::catching::PlannerParams{}.decel;
  EXPECT_FALSE(d.enabled);
  EXPECT_EQ(d.n_nodes, 14);  // MD-24: 14 × 0.025 s = 0.35 s
  EXPECT_DOUBLE_EQ(d.dt_s, 0.025);
  EXPECT_EQ(d.DtNs(), 25'000'000);
  EXPECT_EQ(d.n_blocks, 6);
  EXPECT_EQ((std::array<int, 6>{d.blocks[0], d.blocks[1], d.blocks[2], d.blocks[3], d.blocks[4],
                                d.blocks[5]}),
            (std::array<int, 6>{1, 1, 2, 2, 4, 4}));
  EXPECT_EQ(d.k_max, 4);               // MD-31
  EXPECT_DOUBLE_EQ(d.slack_max, 0.1);  // MD-33
  EXPECT_DOUBLE_EQ(d.slack_terminal_max, 0.1);
  EXPECT_DOUBLE_EQ(d.eta_tau, 0.7);
  EXPECT_DOUBLE_EQ(d.m_q, 0.05);
}

TEST(DecelParams, ParsesTheSectionAndKeepsDefaultsWhenAbsent) {
  const auto absent = ParsePlannerParams(YAML::Load("planner: {enabled: true}"));
  EXPECT_FALSE(absent.decel.enabled);
  EXPECT_EQ(absent.decel.n_nodes, 14);
  const auto p = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 2, 2, "
                 "2]}, replan: {k_max: 2}, eta_tau: 0.6, m_q: 0.04, publish: {slack_max: "
                 "0.2, slack_terminal_max: 0.05}}}"));
  const DecelPlannerParams& d = p.decel;
  EXPECT_TRUE(d.enabled);
  EXPECT_EQ(d.n_nodes, 7);
  EXPECT_EQ(d.DtNs(), 50'000'000);
  EXPECT_EQ(d.n_blocks, 4);
  EXPECT_EQ(d.blocks[3], 2);
  EXPECT_EQ(d.blocks[4], 0) << "a shorter list must not keep the default's tail";
  EXPECT_EQ(d.k_max, 2);
  EXPECT_DOUBLE_EQ(d.eta_tau, 0.6);
  EXPECT_DOUBLE_EQ(d.m_q, 0.04);
  EXPECT_DOUBLE_EQ(d.slack_max, 0.2);
  EXPECT_DOUBLE_EQ(d.slack_terminal_max, 0.05);
}

TEST(DecelParams, RejectsAMalformedSection) {
  for (const char* bad : {
           "planner: {decel_mpc: 3}",
           "planner: {decel_mpc: {enabled: maybe}}",
           "planner: {decel_mpc: {horizon: {n_nodes: 2, blocks: [1, 1]}}}",
           "planner: {decel_mpc: {horizon: {n_nodes: 25}}}",
           // Σ blocks ≠ n_nodes
           "planner: {decel_mpc: {horizon: {n_nodes: 14, blocks: [1, 1, 2, 2, 4]}}}",
           "planner: {decel_mpc: {horizon: {blocks: [7, 7]}}}",
           "planner: {decel_mpc: {horizon: {n_nodes: 14, blocks: [1, 1, 2, 2, 4, 4, 0]}}}",
           // not a whole number of nanoseconds
           "planner: {decel_mpc: {horizon: {dt_s: 0.0250000004}}}",
           "planner: {decel_mpc: {horizon: {dt_s: 0.001}}}",
           // k_max whose patterns drop below three blocks: {1,1,1} → k = 1 has 2
           "planner: {decel_mpc: {horizon: {n_nodes: 3, blocks: [1, 1, 1]}, replan: {k_max: 1}}}",
           "planner: {decel_mpc: {replan: {k_max: 9}}}",
           "planner: {decel_mpc: {eta_tau: 0.0}}",
           "planner: {decel_mpc: {publish: {slack_max: .nan}}}",
           "planner: {decel_mpc: {publish: {slack_terminal_max: -0.01}}}",
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
}

TEST(DecelParams, BlocksForShrinksTheLargestTrailingBlock) {
  const DecelPlannerParams d{};
  const std::array<std::array<int, 6>, 5> expected{{
      {1, 1, 2, 2, 4, 4},
      {1, 1, 2, 2, 4, 3},
      {1, 1, 2, 2, 3, 3},
      {1, 1, 2, 2, 3, 2},
      {1, 1, 2, 2, 2, 2},
  }};
  for (int k = 0; k <= 4; ++k) {
    std::array<int, kMaxDecelNodes> b{};
    int n = 0;
    ASSERT_TRUE(DecelBlocksFor(d, k, b, n)) << k;
    ASSERT_EQ(n, 6) << k;
    int sum = 0;
    for (int i = 0; i < n; ++i) {
      EXPECT_EQ(b[static_cast<std::size_t>(i)],
                expected[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])
          << "k " << k << " block " << i;
      sum += b[static_cast<std::size_t>(i)];
    }
    EXPECT_EQ(sum, 14 - k);
  }
  // Blocks that reach zero are dropped; below three the pattern is refused.
  DecelPlannerParams small{};
  small.n_nodes = 4;
  small.blocks = {1, 1, 1, 1};
  small.n_blocks = 4;
  std::array<int, kMaxDecelNodes> b{};
  int n = 0;
  ASSERT_TRUE(DecelBlocksFor(small, 1, b, n));
  EXPECT_EQ(n, 3);
  EXPECT_FALSE(DecelBlocksFor(small, 2, b, n));
  EXPECT_FALSE(DecelBlocksFor(d, -1, b, n));
  EXPECT_FALSE(DecelBlocksFor(d, 14, b, n));
}

// E1-F08 (#661): the pre-catch keys. n_pre_max defaults to 0 — no pre-catch
// grid, with which the decel planner does not configure (MD-70).
TEST(DecelParams, ApproachKeysDefaultOff) {
  const DecelPlannerParams d = rtc::catching::PlannerParams{}.decel;
  EXPECT_EQ(d.n_pre_max, 0);
  EXPECT_DOUBLE_EQ(d.dt_pre_s, 0.1);
  EXPECT_EQ(d.DtPreNs(), 100'000'000);
  EXPECT_DOUBLE_EQ(d.rest_tol, 0.05);
  EXPECT_DOUBLE_EQ(d.budget_first_s, 0.035);
  EXPECT_DOUBLE_EQ(d.budget_replan_s, 0.025);
  EXPECT_TRUE(d.replan_same_point);
  EXPECT_DOUBLE_EQ(d.catch_pos_err_max, 0.02);
  EXPECT_DOUBLE_EQ(d.w_axis, 100.0);
  EXPECT_DOUBLE_EQ(d.w_v_par, 1.0);
  EXPECT_DOUBLE_EQ(d.w_v_perp, 20.0);
  EXPECT_DOUBLE_EQ(d.gamma_ref, 1.0);
  EXPECT_DOUBLE_EQ(d.kappa, 1.0);
  EXPECT_DOUBLE_EQ(d.sigma_floor, 0.01);
  EXPECT_DOUBLE_EQ(d.w_max, 1e4);
  EXPECT_DOUBLE_EQ(d.w_const, 2500.0);
  EXPECT_DOUBLE_EQ(d.sigma_ref, 0.03);
  EXPECT_FALSE(d.horizon_explicit);
  EXPECT_FALSE(
      ParsePlannerParams(YAML::Load("planner: {decel_mpc: {m_q: 0.04}}")).decel.horizon_explicit);
}

TEST(DecelParams, ARemovedEnabledKeyIsIgnoredWhateverItsValue) {
  // MPC MD-91 removed `planner.decel_mpc.enabled`; the parser rejects no
  // unknown key (D18), so a config that still writes it reads as without it —
  // a non-bool value included, which the key's own parser used to refuse.
  for (const char* stale : {"true", "false", "maybe", "3"}) {
    const std::string yaml = std::string("planner: {decel_mpc: {enabled: ") + stale +
                             ", horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 2, 2, 2]}}}";
    rtc::catching::PlannerParams p;
    ASSERT_NO_THROW(p = ParsePlannerParams(YAML::Load(yaml))) << stale;
    EXPECT_EQ(p.decel.n_nodes, 7) << stale;
    EXPECT_TRUE(p.decel.horizon_explicit) << stale;
  }
}

TEST(DecelParams, ParsesTheApproachKeys) {
  const auto p = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, 2, 3]}, "
                 "replan: {k_max: 2, same_point: false}, "
                 "approach: {n_pre_max: 6, dt_pre_s: 0.08, rest_tol: 0.02}, "
                 "budget: {first_s: 0.04, replan_s: 0.03}, publish: {catch_pos_err_max: 0.015}, "
                 "catch: {w_axis: 50, w_v_par: 2, w_v_perp: 10, gamma_ref: 0.8, kappa: 2, "
                 "sigma_floor: 0.02, w_max: 5000, w_const: 1000, sigma_ref: 0.05}}}"));
  const DecelPlannerParams& d = p.decel;
  EXPECT_TRUE(d.horizon_explicit);
  EXPECT_EQ(d.n_pre_max, 6);
  EXPECT_EQ(d.DtPreNs(), 80'000'000);
  EXPECT_DOUBLE_EQ(d.rest_tol, 0.02);
  EXPECT_DOUBLE_EQ(d.budget_first_s, 0.04);
  EXPECT_DOUBLE_EQ(d.budget_replan_s, 0.03);
  EXPECT_FALSE(d.replan_same_point);
  EXPECT_DOUBLE_EQ(d.catch_pos_err_max, 0.015);
  EXPECT_DOUBLE_EQ(d.w_axis, 50.0);
  EXPECT_DOUBLE_EQ(d.w_v_par, 2.0);
  EXPECT_DOUBLE_EQ(d.w_v_perp, 10.0);
  EXPECT_DOUBLE_EQ(d.gamma_ref, 0.8);
  EXPECT_DOUBLE_EQ(d.kappa, 2.0);
  EXPECT_DOUBLE_EQ(d.sigma_floor, 0.02);
  EXPECT_DOUBLE_EQ(d.w_max, 5000.0);
  EXPECT_DOUBLE_EQ(d.w_const, 1000.0);
  EXPECT_DOUBLE_EQ(d.sigma_ref, 0.05);
  // The pre-catch nodes fit next to the stop's: 24 − 7 nodes, 24 − 4 blocks.
  EXPECT_EQ(ParsePlannerParams(
                YAML::Load("planner: {decel_mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, "
                           "1, 2, 3]}, approach: {n_pre_max: 17}}}"))
                .decel.n_pre_max,
            17);
}

TEST(DecelParams, RejectsMalformedApproachKeys) {
  for (const char* bad : {
           "planner: {decel_mpc: {approach: 3}}",
           "planner: {decel_mpc: {approach: {n_pre_max: -1}}}",
           // 14 stop nodes (the code default) leave 10 for the pre-catch part
           "planner: {decel_mpc: {approach: {n_pre_max: 11}}}",
           "planner: {decel_mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, 2, 3]}, "
           "approach: {n_pre_max: 18}}}",
           "planner: {decel_mpc: {approach: {dt_pre_s: 0.0}}}",
           "planner: {decel_mpc: {approach: {dt_pre_s: 0.3}}}",
           "planner: {decel_mpc: {approach: {dt_pre_s: 0.1000000004}}}",
           "planner: {decel_mpc: {approach: {rest_tol: -0.01}}}",
           "planner: {decel_mpc: {budget: {first_s: 0.0}}}",
           "planner: {decel_mpc: {budget: {replan_s: 0.06}}}",
           "planner: {decel_mpc: {replan: {same_point: maybe}}}",
           "planner: {decel_mpc: {publish: {catch_pos_err_max: 0.0}}}",
           "planner: {decel_mpc: {publish: {catch_pos_err_max: .nan}}}",
           "planner: {decel_mpc: {catch: {w_axis: -1}}}",
           "planner: {decel_mpc: {catch: {gamma_ref: 0.0}}}",
           "planner: {decel_mpc: {catch: {gamma_ref: 1.1}}}",
           "planner: {decel_mpc: {catch: {kappa: 0.0}}}",
           "planner: {decel_mpc: {catch: {sigma_floor: 0.0}}}",
           "planner: {decel_mpc: {catch: {w_max: 0.0}}}",
           "planner: {decel_mpc: {catch: {w_const: .inf}}}",
           "planner: {decel_mpc: {catch: {sigma_ref: 0.0}}}",
       }) {
    EXPECT_THROW(static_cast<void>(ParsePlannerParams(YAML::Load(bad))), std::invalid_argument)
        << bad;
  }
}

// ── 2. RT-side admission and the switch rule ────────────────────────────────

DecelPlanSnapshot AdmissibleSegment() {
  DecelPlanSnapshot p{};
  p.valid = true;
  p.token.activation_generation = 3;
  p.plan_id = 7;
  p.decel_seq = 4;
  p.t_c_ns = kT0 + 500 * kMs;
  p.t0_ns = p.t_c_ns;
  p.dt_ns = kDt;
  p.n_nodes = 14;
  p.nv = 6;
  p.publish_ns = kT0 + 400 * kMs;
  return p;
}

DecelAdmissionContext AdmissionContext() {
  DecelAdmissionContext c;
  c.activation_generation = 3;
  c.plan_active = true;
  c.plan_id = 7;
  c.plan_t_c_ns = kT0 + 500 * kMs;
  c.now = NowReal{kT0 + 450 * kMs};
  c.max_age_ns = 500 * kMs;
  c.reset_floor_ns = kT0;
  return c;
}

TEST(DecelAdmission, JudgesInTheDocumentedOrder) {
  const DecelPlanSnapshot good = AdmissibleSegment();
  const DecelAdmissionContext ctx = AdmissionContext();
  EXPECT_EQ(JudgeDecelPlan(good, ctx, AdmittedDecel{}), DecelRefusal::kNone);
  EXPECT_EQ(JudgeDecelPlan(good, ctx, AdmittedDecel{true, 3}), DecelRefusal::kNone);
  auto judged = [&](auto mutate_plan, auto mutate_ctx, AdmittedDecel admitted = {}) {
    DecelPlanSnapshot p = good;
    DecelAdmissionContext c = ctx;
    mutate_plan(p);
    mutate_ctx(c);
    return JudgeDecelPlan(p, c, admitted);
  };
  const auto none_p = [](DecelPlanSnapshot&) {};
  const auto none_c = [](DecelAdmissionContext&) {};
  EXPECT_EQ(judged([](DecelPlanSnapshot& p) { p.valid = false; }, none_c), DecelRefusal::kInvalid);
  EXPECT_EQ(judged([](DecelPlanSnapshot& p) { p.token.activation_generation = 2; }, none_c),
            DecelRefusal::kActivation);
  EXPECT_EQ(judged(none_p, [](DecelAdmissionContext& c) { c.plan_active = false; }),
            DecelRefusal::kPlan);
  EXPECT_EQ(judged([](DecelPlanSnapshot& p) { p.plan_id = 8; }, none_c), DecelRefusal::kPlan);
  EXPECT_EQ(judged([](DecelPlanSnapshot& p) { p.t_c_ns += 1; }, none_c), DecelRefusal::kPlan);
  EXPECT_EQ(judged(none_p, none_c, AdmittedDecel{true, 4}), DecelRefusal::kRepeat);
  EXPECT_EQ(judged(none_p, none_c, AdmittedDecel{true, 5}), DecelRefusal::kRepeat);
  EXPECT_EQ(judged([](DecelPlanSnapshot& p) { p.publish_ns = 0; }, none_c), DecelRefusal::kAged);
  EXPECT_EQ(judged(none_p, [](DecelAdmissionContext& c) { c.now = NowReal{kT0 + 399 * kMs}; }),
            DecelRefusal::kAged)
      << "a publish instant in the future";
  EXPECT_EQ(judged(none_p, [](DecelAdmissionContext& c) { c.max_age_ns = 10 * kMs; }),
            DecelRefusal::kAged);
  EXPECT_EQ(judged(none_p, [](DecelAdmissionContext& c) { c.max_age_ns = 0; }), DecelRefusal::kNone)
      << "max_age 0 disables the age bound";
  EXPECT_EQ(judged(none_p, [](DecelAdmissionContext& c) { c.reset_floor_ns = kT0 + 401 * kMs; }),
            DecelRefusal::kBeforeReset);
  EXPECT_EQ(judged([](DecelPlanSnapshot& p) { p.qd[Idx(p.n_nodes, 0)] = kNan; }, none_c),
            DecelRefusal::kMalformed);
  // First reason wins: invalid AND wrong plan → invalid.
  EXPECT_EQ(judged(
                [](DecelPlanSnapshot& p) {
                  p.valid = false;
                  p.plan_id = 9;
                },
                none_c),
            DecelRefusal::kInvalid);
}

TEST(DecelAdmission, SegmentSwitchesAtItsEffectiveInstant) {
  const std::int64_t t0 = kT0 + 100 * kMs;
  EXPECT_EQ(ChooseDecelSegment(false, false, 0, t0), DecelSegmentChoice::kNone);
  EXPECT_EQ(ChooseDecelSegment(false, true, t0, t0 - 1), DecelSegmentChoice::kNone)
      << "a pending segment is not sampled before its node 0";
  EXPECT_EQ(ChooseDecelSegment(true, true, t0, t0 - 1), DecelSegmentChoice::kCurrent);
  EXPECT_EQ(ChooseDecelSegment(true, true, t0, t0), DecelSegmentChoice::kPending);
  EXPECT_EQ(ChooseDecelSegment(false, true, t0, t0 + 1), DecelSegmentChoice::kPending);
  EXPECT_EQ(ChooseDecelSegment(true, false, t0, t0 + 1), DecelSegmentChoice::kCurrent);
}

TEST(DecelAdmission, AStatePredictedBeforeTheFloorIsBeforeReset) {
  // MD-37: the planner stamps publish_ns after it reads the RT state, so a
  // segment can pass the publish floor and still come from the tick before
  // the reset. The state floor catches it; 0 leaves the check off.
  DecelPlanSnapshot p = AdmissibleSegment();
  DecelAdmissionContext c = AdmissionContext();
  c.reset_floor_ns = kT0 + 300 * kMs;
  p.rt_state_ns = kT0 + 299 * kMs;
  EXPECT_EQ(JudgeDecelPlan(p, c, AdmittedDecel{}), DecelRefusal::kNone) << "floor off by default";
  c.state_floor_ns = kT0 + 300 * kMs;
  EXPECT_EQ(JudgeDecelPlan(p, c, AdmittedDecel{}), DecelRefusal::kBeforeReset);
  p.rt_state_ns = kT0 + 300 * kMs;
  EXPECT_EQ(JudgeDecelPlan(p, c, AdmittedDecel{}), DecelRefusal::kNone) << "at the floor is after";
}

TEST(DecelSwitch, PassesInsideTheHeadroomAndNamesTheFirstJointPastIt) {
  // MD-39: |Δq̇_i| + K_p|Δq_i| ≤ ρ_max (1 − η_v) q̇_max,i per joint.
  const std::array<double, 3> qmax{2.0, 2.0, 4.0};  // headroom d = 0.2, 0.2, 0.4 at η_v 0.9
  std::array<double, 3> q_c{0.0, 0.0, 0.0};
  std::array<double, 3> qd_c{0.0, 0.0, 0.0};
  const std::array<double, 3> q_ref{0.0, 0.0, 0.0};
  const std::array<double, 3> qd_ref{0.0, 0.0, 0.0};
  auto judge = [&](double rho_max) {
    return JudgeDecelSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 3, 20.0, 0.9, rho_max);
  };
  DecelSwitchVerdict v = judge(1.0);
  EXPECT_TRUE(v.pass);
  EXPECT_EQ(v.rho, 0.0);
  EXPECT_EQ(v.joint, -1);

  q_c[2] = 0.005;  // K_p·Δq = 0.1 → ρ_2 = 0.25
  qd_c[0] = 0.1;   // ρ_0 = 0.5
  v = judge(1.0);
  EXPECT_TRUE(v.pass);
  EXPECT_NEAR(v.rho, 0.5, 1e-12);
  EXPECT_NEAR(v.dq_max, 0.005, 1e-15);
  EXPECT_NEAR(v.dqd_max, 0.1, 1e-15);
  v = judge(0.4);
  EXPECT_FALSE(v.pass);
  EXPECT_EQ(v.joint, 0);
  EXPECT_NEAR(v.rho, 0.5, 1e-12) << "ρ is recorded over every joint, not up to the refusal";

  qd_c[0] = std::numeric_limits<double>::quiet_NaN();
  v = judge(1.0);
  EXPECT_FALSE(v.pass) << "a NaN difference refuses";
  EXPECT_EQ(v.joint, 0);
  EXPECT_TRUE(std::isinf(v.rho));
  qd_c[0] = 0.0;

  // No headroom (η_v = 1, or a missing q̇_max) refuses rather than divides.
  EXPECT_FALSE(JudgeDecelSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 3, 20.0, 1.0, 1.0).pass);
  const std::array<double, 3> no_limit{2.0, 0.0, 4.0};
  v = JudgeDecelSwitch(q_c, qd_c, q_ref, qd_ref, no_limit, 3, 20.0, 0.9, 1.0);
  EXPECT_FALSE(v.pass);
  EXPECT_EQ(v.joint, 1);
  // Bad arguments refuse outright.
  EXPECT_FALSE(JudgeDecelSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 4, 20.0, 0.9, 1.0).pass);
  EXPECT_FALSE(JudgeDecelSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 3, 20.0, 0.9, 0.0).pass);
  EXPECT_FALSE(JudgeDecelSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 0, 20.0, 0.9, 1.0).pass);
}

}  // namespace
