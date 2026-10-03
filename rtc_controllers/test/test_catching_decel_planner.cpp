// E1-F03 (#629): the decel MPC's `planner.decel_mpc.*` keys and the RT-side
// admission and switch rules (planner_io.hpp). The planner itself — the
// first segment, the replans and the cycle that publishes them — is tested in
// test_catching_approach_planner.cpp and test_catching_approach_cycle.cpp
// (E1-F08); the stop-only planner E1-F03 shipped was removed with its tests
// (MD-70).
#include "rtc_controllers/catching/decel_mpc.hpp"
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
using rtc::catching::DecelMpcParams;
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
  EXPECT_EQ(absent.decel.n_nodes, 14);
  const auto p = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 2, 2, "
                 "2]}, replan: {k_max: 2}, eta_tau: 0.6, m_q: 0.04, publish: {slack_max: "
                 "0.2, slack_terminal_max: 0.05}}}"));
  const DecelPlannerParams& d = p.decel;
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

// MPC MD-91: the relative-velocity slack row's two keys, `catch.rho_v` and
// `catch.v_rel_allow`. Off by default — the core then builds no slack variable
// and no rows, so a profile without the keys solves what it solved before.
TEST(DecelParams, VelocitySlackKeysDefaultOffAndParse) {
  const DecelPlannerParams d = rtc::catching::PlannerParams{}.decel;
  EXPECT_EQ(d.rho_v, 0.0);
  EXPECT_EQ(d.v_rel_allow, 0.0);
  const auto absent = ParsePlannerParams(YAML::Load("planner: {decel_mpc: {catch: {w_axis: 50}}}"));
  EXPECT_EQ(absent.decel.rho_v, 0.0);
  EXPECT_EQ(absent.decel.v_rel_allow, 0.0);

  // Distinct values, so a swapped pair of reads would show.
  const auto on = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {catch: {rho_v: 5.0, v_rel_allow: 0.25}}}"));
  EXPECT_DOUBLE_EQ(on.decel.rho_v, 5.0);
  EXPECT_DOUBLE_EQ(on.decel.v_rel_allow, 0.25);
  // The written zeros (what the shipped profiles carry) are the off state.
  const auto zeros = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {catch: {rho_v: 0.0, v_rel_allow: 0.0}}}"));
  EXPECT_EQ(zeros.decel.rho_v, 0.0);
  EXPECT_EQ(zeros.decel.v_rel_allow, 0.0);
  // A bound without the slack is no contradiction: the bound is not read.
  const auto bound_only =
      ParsePlannerParams(YAML::Load("planner: {decel_mpc: {catch: {v_rel_allow: 0.25}}}"));
  EXPECT_EQ(bound_only.decel.rho_v, 0.0);
  EXPECT_DOUBLE_EQ(bound_only.decel.v_rel_allow, 0.25);
  // Bounded from below only (as the core's own check).
  const auto large = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {catch: {rho_v: 1.0e+9, v_rel_allow: 50.0}}}"));
  EXPECT_DOUBLE_EQ(large.decel.rho_v, 1e9);
  EXPECT_DOUBLE_EQ(large.decel.v_rel_allow, 50.0);
}

// Each rejection names the key the profile has to fix, and for the reason
// stated — a range row must not pass because the cross check threw (or the
// reverse), so the other key and the other reason are asserted ABSENT.
TEST(DecelParams, RejectsAVelocitySlackKeyByName) {
  const std::string rho = "'planner.decel_mpc.catch.rho_v'";
  const std::string allow = "'planner.decel_mpc.catch.v_rel_allow'";
  const std::string range = "must be a finite number >= 0";
  const std::string cross = "turns the slack on";
  const auto message = [](const std::string& yaml) -> std::string {
    try {
      static_cast<void>(ParsePlannerParams(YAML::Load(yaml)));
    } catch (const std::invalid_argument& e) {
      return e.what();
    }
    return {};
  };
  const auto has = [](const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
  };

  // rho_v out of range — with a valid bound beside it, so only the range can
  // be what refuses it.
  for (const char* bad : {"-0.1", ".nan", ".inf", "-.inf"}) {
    const std::string why = message(std::string("planner: {decel_mpc: {catch: {rho_v: ") + bad +
                                    ", v_rel_allow: 0.2}}}");
    ASSERT_FALSE(why.empty()) << "rho_v: " << bad << " was accepted";
    EXPECT_TRUE(has(why, rho)) << why;
    EXPECT_TRUE(has(why, range)) << why;
    EXPECT_FALSE(has(why, allow)) << why;
  }
  // v_rel_allow out of range — with the slack OFF, so the cross check cannot
  // be what refuses it.
  for (const char* bad : {"-0.01", ".nan", ".inf", "-.inf"}) {
    const std::string why =
        message(std::string("planner: {decel_mpc: {catch: {v_rel_allow: ") + bad + "}}}");
    ASSERT_FALSE(why.empty()) << "v_rel_allow: " << bad << " was accepted";
    EXPECT_TRUE(has(why, allow)) << why;
    EXPECT_TRUE(has(why, range)) << why;
    EXPECT_FALSE(has(why, rho)) << why;
  }
  // Not a number at all.
  EXPECT_TRUE(
      has(message("planner: {decel_mpc: {catch: {rho_v: soft}}}"), rho + " must be a number"));
  EXPECT_TRUE(has(message("planner: {decel_mpc: {catch: {v_rel_allow: [0.2]}}}"),
                  allow + " must be a number"));
  // The cross constraint: the slack on with no bound to be slack against —
  // the bound absent, and written as 0. Both keys are named.
  for (const char* bad : {"planner: {decel_mpc: {catch: {rho_v: 1.0}}}",
                          "planner: {decel_mpc: {catch: {rho_v: 1.0, v_rel_allow: 0.0}}}"}) {
    const std::string why = message(bad);
    ASSERT_FALSE(why.empty()) << bad << " was accepted";
    EXPECT_TRUE(has(why, rho)) << why;
    EXPECT_TRUE(has(why, allow)) << why;
    EXPECT_TRUE(has(why, cross)) << why;
    EXPECT_FALSE(has(why, range)) << why;
  }
}

// MPC MD-92: the core's own design values as keys. The planner's defaults ARE
// the core's (DecelMpcParams) — a profile without the keys, and the shipped
// profiles that write them, solve what the code solved before.
TEST(DecelParams, TheDesignKeysDefaultToTheCoresOwnValues) {
  const DecelMpcParams core{};
  const DecelPlannerParams d = rtc::catching::PlannerParams{}.decel;
  EXPECT_TRUE(d.jerk_weight.empty());  // empty = the core's all-ones
  EXPECT_EQ(core.jerk_weight.size(), 0);
  EXPECT_EQ(d.u_scale, core.u_scale);
  EXPECT_EQ(d.w_delta, core.w_delta);
  EXPECT_EQ(d.rho_tau, core.rho_tau);
  EXPECT_EQ(d.axis_theta_max, core.axis_theta_max);
  EXPECT_EQ(d.delta_tr, core.delta_tr);
  EXPECT_EQ(d.reference_rest_tol, core.reference_rest_tol);
  EXPECT_EQ(d.solver_max_iter, core.solver.max_iter);
  EXPECT_EQ(d.solver_max_iter_in, core.solver.max_iter_in);
  EXPECT_EQ(d.solver_eps_abs, core.solver.eps_abs);
  EXPECT_EQ(d.solver_eps_rel, core.solver.eps_rel);
  EXPECT_EQ(d.ref_speed_fraction, 0.9);  // the planner's own former constant (MD-62)
  // An absent section changes nothing either.
  const auto absent = ParsePlannerParams(YAML::Load("planner: {decel_mpc: {m_q: 0.05}}")).decel;
  EXPECT_TRUE(absent.jerk_weight.empty());
  EXPECT_EQ(absent.delta_tr, core.delta_tr);
  EXPECT_EQ(absent.solver_max_iter, core.solver.max_iter);
}

// Twelve keys, twelve distinct values: a read wired to the wrong field, or to
// the default, shows as a mismatch on its own line.
TEST(DecelParams, TheDesignKeysParse) {
  const auto d = ParsePlannerParams(YAML::Load(R"(
planner:
  decel_mpc:
    cost: {jerk_weight: [1.0, 2.0, 3.0], u_scale: 500.0, w_delta: 2.5, rho_tau: 7.0}
    catch: {axis_theta_max: 1.2}
    linearization: {delta_tr: 0.2, reference_rest_tol: 2.0e-5, ref_speed_fraction: 0.8}
    solver: {max_iter: 300, max_iter_in: 150, eps_abs: 2.0e-7, eps_rel: 1.0e-5}
)"))
                     .decel;
  ASSERT_EQ(d.jerk_weight.size(), 3U);
  EXPECT_EQ(d.jerk_weight[0], 1.0);
  EXPECT_EQ(d.jerk_weight[1], 2.0);
  EXPECT_EQ(d.jerk_weight[2], 3.0);
  EXPECT_EQ(d.u_scale, 500.0);
  EXPECT_EQ(d.w_delta, 2.5);
  EXPECT_EQ(d.rho_tau, 7.0);
  EXPECT_EQ(d.axis_theta_max, 1.2);
  EXPECT_EQ(d.delta_tr, 0.2);
  EXPECT_EQ(d.reference_rest_tol, 2.0e-5);
  EXPECT_EQ(d.ref_speed_fraction, 0.8);
  EXPECT_EQ(d.solver_max_iter, 300);
  EXPECT_EQ(d.solver_max_iter_in, 150);
  EXPECT_EQ(d.solver_eps_abs, 2.0e-7);
  EXPECT_EQ(d.solver_eps_rel, 1.0e-5);
  // Valid edges: rho_tau 0 (torque rows off), w_delta 0, ref_speed_fraction 1.
  const auto edge =
      ParsePlannerParams(YAML::Load("planner: {decel_mpc: {cost: {rho_tau: 0.0, w_delta: "
                                    "0.0}, linearization: {ref_speed_fraction: 1.0}}}"))
          .decel;
  EXPECT_EQ(edge.rho_tau, 0.0);
  EXPECT_EQ(edge.w_delta, 0.0);
  EXPECT_EQ(edge.ref_speed_fraction, 1.0);
}

// Each rejection names the key the profile has to fix. The message is looked
// up for that key's full path, so a refusal by ANOTHER key's check (or the
// cross constraint) cannot stand in for the range under test.
TEST(DecelParams, RejectsADesignKeyByName) {
  const auto message = [](const std::string& body) -> std::string {
    try {
      static_cast<void>(ParsePlannerParams(YAML::Load("planner: {decel_mpc: {" + body + "}}")));
    } catch (const std::invalid_argument& e) {
      return e.what();
    }
    return {};
  };

  struct Case {
    const char* body;
    const char* key;
  };

  const Case cases[] = {
      {"cost: {jerk_weight: []}", "planner.decel_mpc.cost.jerk_weight'"},
      {"cost: {jerk_weight: 1.0}", "planner.decel_mpc.cost.jerk_weight'"},
      {"cost: {jerk_weight: [1.0, 0.0]}", "planner.decel_mpc.cost.jerk_weight[1]'"},
      {"cost: {jerk_weight: [-1.0, 1.0]}", "planner.decel_mpc.cost.jerk_weight[0]'"},
      {"cost: {jerk_weight: [1.0, .nan]}", "planner.decel_mpc.cost.jerk_weight[1]'"},
      {"cost: {jerk_weight: [1.0, x]}", "planner.decel_mpc.cost.jerk_weight[1]'"},
      {"cost: {u_scale: 0.0}", "planner.decel_mpc.cost.u_scale'"},
      {"cost: {u_scale: -1.0}", "planner.decel_mpc.cost.u_scale'"},
      {"cost: {u_scale: .inf}", "planner.decel_mpc.cost.u_scale'"},
      {"cost: {w_delta: -0.1}", "planner.decel_mpc.cost.w_delta'"},
      {"cost: {w_delta: .nan}", "planner.decel_mpc.cost.w_delta'"},
      {"cost: {rho_tau: -1.0}", "planner.decel_mpc.cost.rho_tau'"},
      {"cost: {rho_tau: .inf}", "planner.decel_mpc.cost.rho_tau'"},
      {"catch: {axis_theta_max: 0.0}", "planner.decel_mpc.catch.axis_theta_max'"},
      {"catch: {axis_theta_max: 3.1415926535897932}", "planner.decel_mpc.catch.axis_theta_max'"},
      {"catch: {axis_theta_max: 4.0}", "planner.decel_mpc.catch.axis_theta_max'"},
      {"catch: {axis_theta_max: .nan}", "planner.decel_mpc.catch.axis_theta_max'"},
      {"linearization: {delta_tr: 0.0}", "planner.decel_mpc.linearization.delta_tr'"},
      {"linearization: {delta_tr: .inf}", "planner.decel_mpc.linearization.delta_tr'"},
      {"linearization: {delta_tr: -0.1}", "planner.decel_mpc.linearization.delta_tr'"},
      {"linearization: {reference_rest_tol: 0.0}",
       "planner.decel_mpc.linearization.reference_rest_tol'"},
      {"linearization: {reference_rest_tol: .nan}",
       "planner.decel_mpc.linearization.reference_rest_tol'"},
      {"linearization: {ref_speed_fraction: 0.0}",
       "planner.decel_mpc.linearization.ref_speed_fraction'"},
      {"linearization: {ref_speed_fraction: 1.01}",
       "planner.decel_mpc.linearization.ref_speed_fraction'"},
      {"linearization: {ref_speed_fraction: .nan}",
       "planner.decel_mpc.linearization.ref_speed_fraction'"},
      {"solver: {max_iter: 0}", "planner.decel_mpc.solver.max_iter'"},
      {"solver: {max_iter: 1.5}", "planner.decel_mpc.solver.max_iter'"},
      {"solver: {max_iter_in: 0}", "planner.decel_mpc.solver.max_iter_in'"},
      {"solver: {max_iter_in: -3}", "planner.decel_mpc.solver.max_iter_in'"},
      {"solver: {eps_abs: 0.0}", "planner.decel_mpc.solver.eps_abs'"},
      {"solver: {eps_abs: -1.0e-6}", "planner.decel_mpc.solver.eps_abs'"},
      {"solver: {eps_rel: -1.0e-6}", "planner.decel_mpc.solver.eps_rel'"},
      {"solver: {eps_rel: .inf}", "planner.decel_mpc.solver.eps_rel'"},
  };
  for (const Case& c : cases) {
    const std::string why = message(c.body);
    ASSERT_FALSE(why.empty()) << c.body << " was accepted";
    EXPECT_NE(why.find(c.key), std::string::npos) << c.body << " -> " << why;
  }
  // The cross constraint: the rest tolerance at or below eps_abs would refuse
  // every warm solve. Both keys are named; neither range text is.
  for (const char* body :
       {"solver: {eps_abs: 1.0e-4}",  // against the default 1e-4 tolerance
        "linearization: {reference_rest_tol: 1.0e-6}",
        "linearization: {reference_rest_tol: 5.0e-7}, solver: {eps_abs: 5.0e-7}"}) {
    const std::string why = message(body);
    ASSERT_FALSE(why.empty()) << body << " was accepted";
    EXPECT_NE(why.find("planner.decel_mpc.linearization.reference_rest_tol'"), std::string::npos)
        << why;
    EXPECT_NE(why.find("planner.decel_mpc.solver.eps_abs'"), std::string::npos) << why;
    EXPECT_NE(why.find("must exceed"), std::string::npos) << why;
  }
  // Above kDecelRestTol Judge would take a node N the RT's validator refuses
  // (ValidateDecelNodes), so the tolerance is bounded above, by key name.
  for (const char* body : {"linearization: {reference_rest_tol: 2.0e-3}",
                           "linearization: {reference_rest_tol: 1.0}"}) {
    const std::string why = message(body);
    ASSERT_FALSE(why.empty()) << body << " was accepted";
    EXPECT_NE(why.find("planner.decel_mpc.linearization.reference_rest_tol'"), std::string::npos)
        << why;
    EXPECT_NE(why.find("kDecelRestTol"), std::string::npos) << why;
  }
  EXPECT_TRUE(message("linearization: {reference_rest_tol: 1.0e-3}").empty());
  // The cross-check message echoes tiny tolerances as written, not as 0.000000.
  {
    const std::string why =
        message("linearization: {reference_rest_tol: 5.0e-7}, solver: {eps_abs: 1.0e-6}");
    EXPECT_NE(why.find("5.0e-7"), std::string::npos) << why;
    EXPECT_NE(why.find("1.0e-6"), std::string::npos) << why;
    EXPECT_EQ(why.find("0.000000"), std::string::npos) << why;
  }
  // A default (absent key) is printed in scientific notation, not rounded to 0.
  {
    const std::string why = message("solver: {eps_abs: 1.0e-4}");
    EXPECT_NE(why.find("e-04"), std::string::npos) << why;
    EXPECT_EQ(why.find("0.000100"), std::string::npos) << why;
  }
  // A tolerance just above eps_abs is fine.
  EXPECT_TRUE(
      message("linearization: {reference_rest_tol: 1.0e-6}, solver: {eps_abs: 5.0e-7}").empty());
}

// #698: `cost.w_perp`, the stop-path weight. Off by default — the core's own
// default — so a profile without the key, and the shipped ones that write
// 0.0, solve what they solved before the key existed.
TEST(DecelParams, TheStopPathWeightDefaultsOffAndParses) {
  const DecelMpcParams core{};
  const DecelPlannerParams d = rtc::catching::PlannerParams{}.decel;
  EXPECT_EQ(d.w_perp, 0.0);
  EXPECT_EQ(d.w_perp, core.w_perp);
  // Absent: the section, and the key inside a present section.
  EXPECT_EQ(ParsePlannerParams(YAML::Load("planner: {decel_mpc: {m_q: 0.05}}")).decel.w_perp, 0.0);
  EXPECT_EQ(
      ParsePlannerParams(YAML::Load("planner: {decel_mpc: {cost: {w_delta: 2.5}}}")).decel.w_perp,
      0.0);
  // The written zero (what the shipped profiles carry) is the off state.
  EXPECT_EQ(
      ParsePlannerParams(YAML::Load("planner: {decel_mpc: {cost: {w_perp: 0.0}}}")).decel.w_perp,
      0.0);
  // Beside its neighbours, each with a value of its own: a read wired to
  // another field shows.
  const auto on = ParsePlannerParams(YAML::Load("planner: {decel_mpc: {cost: {u_scale: 500.0, "
                                                "w_delta: 2.5, rho_tau: 7.0, w_perp: 40.0}}}"))
                      .decel;
  EXPECT_EQ(on.w_perp, 40.0);
  EXPECT_EQ(on.u_scale, 500.0);
  EXPECT_EQ(on.w_delta, 2.5);
  EXPECT_EQ(on.rho_tau, 7.0);
  // Bounded from below only (as the core's own check).
  EXPECT_EQ(
      ParsePlannerParams(YAML::Load("planner: {decel_mpc: {cost: {w_perp: 1.0e+9}}}")).decel.w_perp,
      1e9);
}

TEST(DecelParams, RejectsTheStopPathWeightByName) {
  const std::string key = "'planner.decel_mpc.cost.w_perp'";
  const auto message = [](const std::string& body) -> std::string {
    try {
      static_cast<void>(ParsePlannerParams(YAML::Load("planner: {decel_mpc: {" + body + "}}")));
    } catch (const std::invalid_argument& e) {
      return e.what();
    }
    return {};
  };
  const auto has = [](const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
  };
  // Valid neighbours beside it, so only this key's range can be what refuses.
  for (const char* bad : {"-0.1", "-1.0e-12", ".nan", ".inf", "-.inf"}) {
    const std::string why =
        message(std::string("cost: {w_delta: 1.0, rho_tau: 10.0, w_perp: ") + bad + "}");
    ASSERT_FALSE(why.empty()) << "w_perp: " << bad << " was accepted";
    EXPECT_TRUE(has(why, key)) << why;
    EXPECT_TRUE(has(why, "must be a finite number >= 0")) << why;
    EXPECT_FALSE(has(why, "w_delta")) << why;
    EXPECT_FALSE(has(why, "rho_tau")) << why;
  }
  // Not a number at all.
  EXPECT_TRUE(has(message("cost: {w_perp: strong}"), key + " must be a number"));
  EXPECT_TRUE(has(message("cost: {w_perp: [1.0]}"), key + " must be a number"));
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
