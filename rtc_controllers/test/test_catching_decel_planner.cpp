// E1-F03 (#629): the decel planner on the planner thread (decel_planner.hpp),
// its `planner.decel_mpc.*` keys, the RT-side admission and switch rules
// (planner_io.hpp) and the cycle's decel step (planner_cycle.hpp). Each #629
// "Done when" item maps to named tests (spec on #629, MD-23 – MD-33):
//   1 pre-computed before t_c        WaitsForTPreThenSolvesCold,
//                                    PublishedSegmentIsDeviceOrderAndEndsAtTheStop
//   2 continuity / fixed stop end    PostCatchReplanContinuesTheSegment (MD-31)
//   3 fail-closed publish gate       OverBudgetWithholds, EachSlackThresholdWithholdsOnItsOwn,
//                                    PastTheReplanWindowPublishesNothing, ...
//   4 within budget at N_s = 14      ShippedHorizonTiming6R / 7R (MD-24, MD-26)
//   5 no allocation outside ProxQP   AllocatesNothingOutsideProxQp (MD-23)
//   6 cycle integration              DecelCycle.* (MD-29: CycleOutcome untouched)
// plus the informational records D-3 / D-11 asked for (armature re-evaluation,
// static wrist torque at the stop posture).
//
// Fixtures break representation symmetries: the device order is a
// non-identity permutation of the model order, T_arm ≠ 0 (real ≠ lead axis),
// and instants are realistic absolute steady ns.
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/decel_planner.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/rnea.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rtc::catching::AdmittedDecel;
using rtc::catching::ChooseDecelSegment;
using rtc::catching::DecelAdmissionContext;
using rtc::catching::DecelBlocksFor;
using rtc::catching::DecelOutcome;
using rtc::catching::DecelOutcomeName;
using rtc::catching::DecelPlanner;
using rtc::catching::DecelPlannerConstants;
using rtc::catching::DecelPlannerModel;
using rtc::catching::DecelPlannerParams;
using rtc::catching::DecelPlanSnapshot;
using rtc::catching::DecelRecord;
using rtc::catching::DecelRefusal;
using rtc::catching::DecelSegmentChoice;
using rtc::catching::DecelSwitchVerdict;
using rtc::catching::JudgeDecelPlan;
using rtc::catching::JudgeDecelSwitch;
using rtc::catching::kMaxDecelNodes;
using rtc::catching::kMaxDecelNv;
using rtc::catching::Mode;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::NowReal;
using rtc::catching::ParsePlannerParams;
using rtc::catching::PlannerRtState;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kDt = 25 * kMs;                     // Δ_s (MD-24)
constexpr std::int64_t kTArm = 30 * kMs;                   // T_arm ≠ 0
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;  // realistic steady ns
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// ── Clock ────────────────────────────────────────────────────────────────────
// Returns the pinned instant, then advances it by the step (0 = frozen): a
// deterministic stand-in for the solve taking time.
std::atomic<std::int64_t> g_now{kT0};
std::atomic<std::int64_t> g_step{0};

std::int64_t FakeClock() noexcept {
  return g_now.fetch_add(g_step.load());
}

void SetClock(std::int64_t now, std::int64_t step = 0) {
  g_now.store(now);
  g_step.store(step);
}

// ── Arms ─────────────────────────────────────────────────────────────────────

struct Arm {
  std::shared_ptr<const pinocchio::Model> model;
  pinocchio::FrameIndex frame{0};
  Eigen::VectorXd q_nominal;             // model order
  std::array<int, 7> device_of_model{};  // non-identity permutation
  // Ratings of each PHYSICAL joint, listed in MODEL order (the shipped device
  // configs' values). The permutation relabels joints; it must not move a
  // rating to another joint — an elbow given a wrist's 28 N·m saturates on
  // gravity alone.
  std::vector<double> qd_max;
  std::vector<double> tau_max;
  std::vector<double> armature;  // MJCF values — analysis only (MD-25)
};

std::shared_ptr<const pinocchio::Model> LoadModel(const std::string& rel) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(RTC_TEST_ROBOT_DESCRIPTIONS_DIR) + "/" + rel;
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  return builder.GetFullModel();
}

Arm Arm6() {
  Arm a;
  a.model = LoadModel("ur5e/urdf/ur5e.urdf");
  a.frame = a.model->getFrameId("tool0");
  a.q_nominal.resize(6);
  a.q_nominal << 0.2, -1.4, 1.1, -1.9, -1.6, 0.1;
  a.device_of_model = {2, 0, 5, 1, 4, 3, 0};
  a.qd_max = {2.0, 2.0, 3.0, 3.0, 3.0, 3.0};
  a.tau_max = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};
  a.armature = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  return a;
}

Arm Arm7() {
  Arm a;
  a.model = LoadModel("iiwa7/urdf/iiwa7.urdf");
  a.frame = a.model->getFrameId("ee_link");
  a.q_nominal.resize(7);
  a.q_nominal << 0.0, 0.6, 0.0, -1.2, 0.0, 0.9, 0.0;
  a.device_of_model = {3, 0, 6, 1, 5, 2, 4};
  a.qd_max = {1.71, 1.71, 1.74, 2.27, 2.44, 3.14, 3.14};
  a.tau_max = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
  a.armature = {0.25, 0.25, 0.25, 0.25, 0.15, 0.15, 0.15};
  return a;
}

int Dev(const Arm& a, int m) {
  return a.device_of_model[static_cast<std::size_t>(m)];
}

DecelPlannerModel PlannerModelOf(const Arm& a) {
  DecelPlannerModel pm;
  pm.arm = a.model;
  pm.catch_frame = a.frame;
  pm.nv = a.model->nv;
  for (int m = 0; m < pm.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    pm.device_of_model[u] = Dev(a, m);
    pm.q_min[u] = a.model->lowerPositionLimit[m];
    pm.q_max[u] = a.model->upperPositionLimit[m];
    pm.qdot_max[u] = a.qd_max[u];
    pm.tau_max[u] = a.tau_max[u];
    pm.qddot_cap[u] = 20.0;
  }
  pm.qddot_cap_valid = true;
  return pm;
}

DecelPlannerConstants Consts() {
  DecelPlannerConstants c;
  c.eta_v = 0.9;
  c.t_arm_s = static_cast<double>(kTArm) * 1e-9;
  c.control_dt = 0.002;
  c.budget_s = 0.020;
  return c;
}

// The RT's report: command state in DEVICE order at real instant `rt_ns`.
PlannerRtState Rt(const Arm& a, std::int64_t t_c, std::int64_t rt_ns, const Eigen::VectorXd& q,
                  const Eigen::VectorXd& qd, std::uint32_t plan_id = 7) {
  PlannerRtState s{};
  s.valid = true;
  s.activation_generation = 3;
  s.rt_iteration = 1000;
  s.rt_state_ns = rt_ns;
  s.reset_epoch = 1;
  s.mode = static_cast<std::uint8_t>(Mode::kCommitted);
  s.nv = a.model->nv;
  s.cmd_seeded = true;
  for (int m = 0; m < s.nv; ++m) {
    const auto d = static_cast<std::size_t>(Dev(a, m));
    s.q_cmd[d] = q[m];
    s.qd_cmd[d] = qd[m];
  }
  s.plan_active = true;
  s.plan_id = plan_id;
  s.plan_t_c_ns = t_c;
  s.track_seen = true;
  s.track_generation = 5;
  return s;
}

// A catching-like entry velocity: the TCP moving at ~1 m/s.
Eigen::VectorXd EntryVelocity(int n) {
  Eigen::VectorXd qd(n);
  for (int j = 0; j < n; ++j) {
    qd[j] = (j % 2 == 0 ? 0.6 : -0.5) * (1.0 - 0.08 * j);
  }
  return qd;
}

std::size_t Idx(int k, int j) {
  return static_cast<std::size_t>(k * kMaxDecelNv + j);
}

struct Planner {
  Arm arm;
  DecelPlanner planner;
  DecelPlanSnapshot out{};
  DecelRecord rec{};

  explicit Planner(Arm a, DecelPlannerParams params = DecelPlannerParams{},
                   DecelPlannerConstants consts = Consts())
      : arm(std::move(a)) {
    std::string err;
    EXPECT_TRUE(planner.Configure(PlannerModelOf(arm), consts, params, &FakeClock, &err)) << err;
  }

  bool Step(const PlannerRtState& rt) { return planner.Plan(rt, out, rec); }

  // Publish what Step produced the way the cycle does.
  void Publish(std::uint32_t seq, std::int64_t publish_ns) {
    out.decel_seq = seq;
    out.publish_ns = publish_ns;
    planner.NotePublished(out);
  }
};

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
  EXPECT_DOUBLE_EQ(d.t_pre_s, 0.1);    // MD-26
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
  const auto p = ParsePlannerParams(YAML::Load(
      "planner: {decel_mpc: {enabled: true, horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 2, 2, "
      "2]}, replan: {t_pre_s: 0.15, k_max: 2}, eta_tau: 0.6, m_q: 0.04, publish: {slack_max: "
      "0.2, slack_terminal_max: 0.05}}}"));
  const DecelPlannerParams& d = p.decel;
  EXPECT_TRUE(d.enabled);
  EXPECT_EQ(d.n_nodes, 7);
  EXPECT_EQ(d.DtNs(), 50'000'000);
  EXPECT_EQ(d.n_blocks, 4);
  EXPECT_EQ(d.blocks[3], 2);
  EXPECT_EQ(d.blocks[4], 0) << "a shorter list must not keep the default's tail";
  EXPECT_DOUBLE_EQ(d.t_pre_s, 0.15);
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
           "planner: {decel_mpc: {replan: {t_pre_s: -0.1}}}",
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

// E1-F08 (#661): the APPROACH–stop keys. Off by default (MD-55), so every
// test above runs the stop-segment planner unchanged.
TEST(DecelParams, ApproachKeysDefaultOff) {
  const DecelPlannerParams d = rtc::catching::PlannerParams{}.decel;
  EXPECT_EQ(d.n_pre_max, 0);
  EXPECT_DOUBLE_EQ(d.dt_pre_s, 0.1);
  EXPECT_EQ(d.DtPreNs(), 100'000'000);
  EXPECT_DOUBLE_EQ(d.rest_tol, 0.05);
  EXPECT_DOUBLE_EQ(d.budget_first_s, 0.035);
  EXPECT_DOUBLE_EQ(d.budget_replan_s, 0.025);
  EXPECT_TRUE(d.replan_same_point);
  EXPECT_FALSE(d.shadow);
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
  EXPECT_FALSE(ParsePlannerParams(YAML::Load("planner: {decel_mpc: {enabled: true}}"))
                   .decel.horizon_explicit);
}

TEST(DecelParams, ParsesTheApproachKeys) {
  const auto p = ParsePlannerParams(
      YAML::Load("planner: {decel_mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, 2, 3]}, "
                 "replan: {k_max: 2, same_point: false}, shadow: true, "
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
  EXPECT_TRUE(d.shadow);
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
           "planner: {decel_mpc: {shadow: 1.5}}",
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

// ── 3. The planner ───────────────────────────────────────────────────────────

TEST(DecelPlanner, ConfigureBuildsOneCorePerReplanInstance) {
  Planner p(Arm6());
  ASSERT_TRUE(p.planner.Configured());
  for (int k = 0; k <= 4; ++k) {
    EXPECT_EQ(p.planner.Core(k).NumNodes(), 14 - k) << k;
    EXPECT_EQ(p.planner.Core(k).NumBlocks(), 6) << k;
  }
  DecelPlanner bad;
  DecelPlannerModel pm = PlannerModelOf(Arm6());
  std::string err;
  EXPECT_FALSE(bad.Configure(pm, Consts(), DecelPlannerParams{}, nullptr, &err));
  pm.device_of_model[1] = pm.device_of_model[0];
  EXPECT_FALSE(bad.Configure(pm, Consts(), DecelPlannerParams{}, &FakeClock, &err));
  EXPECT_NE(err.find("permutation"), std::string::npos) << err;
  pm = PlannerModelOf(Arm6());
  pm.tau_max[3] = 0.0;
  EXPECT_FALSE(bad.Configure(pm, Consts(), DecelPlannerParams{}, &FakeClock, &err));
  EXPECT_NE(err.find("Init"), std::string::npos) << err;
  EXPECT_FALSE(bad.Configured());
}

TEST(DecelPlanner, WaitsForTPreThenSolvesCold) {
  Planner p(Arm6());
  const Eigen::VectorXd qd = EntryVelocity(6);
  const std::int64_t now = kT0;
  SetClock(now);
  // t_c − now_lead = 0.2 s > t_pre 0.1 s: not yet.
  const std::int64_t t_c_far = now + kTArm + 200 * kMs;
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c_far, now - 2 * kMs, p.arm.q_nominal, qd)));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kNotDue) << DecelOutcomeName(p.rec.outcome);
  // 0.09 s: due — the first solve is cold (pre-solve + solve) at k = 0.
  const std::int64_t t_c = now + kTArm + 90 * kMs;
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, qd)))
      << DecelOutcomeName(p.rec.outcome) << " " << DecelMpcReasonName(p.rec.core_reason);
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kReady);
  EXPECT_EQ(p.rec.k, 0);
  EXPECT_EQ(p.rec.n_nodes, 14);
  EXPECT_TRUE(p.rec.presolved);
  EXPECT_FALSE(p.rec.from_segment);
  // h = t_eff − (rt_state_ns + T_arm) = 0.09 + 0.002 s.
  EXPECT_NEAR(p.rec.h_s, 0.092, 1e-12);
}

TEST(DecelPlanner, TheReportLeadMovesTheReportedInstant) {
  // MD-40: with report_lead the report is the state at rt_state + T_arm +
  // report_lead, so the extrapolation span h shrinks by exactly that much.
  DecelPlannerConstants c = Consts();
  c.report_lead_s = 0.004;
  Planner p(Arm6(), DecelPlannerParams{}, c);
  const std::int64_t now = kT0;
  SetClock(now);
  const std::int64_t t_c = now + kTArm + 90 * kMs;
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))))
      << DecelOutcomeName(p.rec.outcome);
  EXPECT_NEAR(p.rec.h_s, 0.092 - 0.004, 1e-12);

  DecelPlanner bad;
  std::string err;
  c.report_lead_s = -0.001;
  EXPECT_FALSE(bad.Configure(PlannerModelOf(Arm6()), c, DecelPlannerParams{}, &FakeClock, &err));
  EXPECT_NE(err.find("report_lead_s"), std::string::npos) << err;
}

TEST(DecelPlanner, PublishedSegmentIsDeviceOrderAndEndsAtTheStop) {
  Planner p(Arm6());
  const Eigen::VectorXd qd = EntryVelocity(6);
  SetClock(kT0);
  const std::int64_t rt_ns = kT0 - 2 * kMs;
  const std::int64_t t_c = kT0 + kTArm + 80 * kMs;
  const PlannerRtState rt = Rt(p.arm, t_c, rt_ns, p.arm.q_nominal, qd);
  ASSERT_TRUE(p.Step(rt)) << DecelOutcomeName(p.rec.outcome);
  const DecelPlanSnapshot& s = p.out;
  EXPECT_TRUE(rtc::catching::ValidateDecelNodes(s));
  EXPECT_EQ(s.t_c_ns, t_c);
  EXPECT_EQ(s.t0_ns, t_c) << "pre-catch node 0 is at t_c (k = 0)";
  EXPECT_EQ(s.k0, 0);
  EXPECT_EQ(s.dt_ns, kDt);
  EXPECT_EQ(s.n_nodes, 14);
  EXPECT_EQ(s.plan_id, 7U);
  EXPECT_EQ(s.token.activation_generation, 3U);
  EXPECT_EQ(s.rt_state_ns, rt_ns);
  // Node 0 = the prediction: x̂ = q + q̇h (no trusted q̈ on a first wake),
  // in DEVICE order — the permutation would scramble a model-order store.
  const double h = static_cast<double>(t_c - (rt_ns + kTArm)) * 1e-9;
  for (int m = 0; m < 6; ++m) {
    const int d = Dev(p.arm, m);
    EXPECT_NEAR(s.q[Idx(0, d)], p.arm.q_nominal[m] + qd[m] * h, 1e-12) << m;
    EXPECT_NEAR(s.qd[Idx(0, d)], qd[m], 1e-12) << m;
    EXPECT_NEAR(s.qd[Idx(s.n_nodes, d)], 0.0, 1e-5) << "node N at rest";
    EXPECT_NEAR(s.qdd[Idx(s.n_nodes, d)], 0.0, 1e-5);
  }
  EXPECT_FALSE(s.x0_clamped);
  EXPECT_TRUE(std::isfinite(s.slack_max));
  EXPECT_LE(s.slack_max, 0.1);
}

TEST(DecelPlanner, OneSolvePerGridPoint) {
  Planner p(Arm6());
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 60 * kMs;
  const PlannerRtState rt = Rt(p.arm, t_c, kT0 - 2 * kMs, p.arm.q_nominal, EntryVelocity(6));
  ASSERT_TRUE(p.Step(rt));
  p.Publish(1, kT0);
  SetClock(kT0 + 20 * kMs);  // still before t_c: k is 0 again
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0 + 18 * kMs, p.arm.q_nominal, EntryVelocity(6))));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kUpToDate) << DecelOutcomeName(p.rec.outcome);
}

TEST(DecelPlanner, PostCatchReplanContinuesTheSegment) {
  // MD-31: after t_c the replan starts at the next reachable grid point k and
  // solves N_s − k nodes to the SAME end. With the RT reported as following
  // the published segment (path (i), what E1-F04 will report), node 0 is that
  // segment at t_eff, the shifted reference is accepted (no cold retry), and
  // the new segment stays on the old one.
  Planner p(Arm6());
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 60 * kMs;
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, kT0 - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))));
  p.Publish(1, kT0);
  const DecelPlanSnapshot first = p.out;

  const std::int64_t now = t_c - kTArm + 10 * kMs;  // now_lead = t_c + 10 ms
  SetClock(now);
  PlannerRtState rt = Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(6));
  rt.mode = static_cast<std::uint8_t>(Mode::kDecel);
  rt.decel_active = true;
  rt.decel_seq = 1;
  ASSERT_TRUE(p.Step(rt)) << DecelOutcomeName(p.rec.outcome) << " "
                          << DecelMpcReasonName(p.rec.core_reason);
  // earliest = now_lead + budget + 2 dt = t_c + 34 ms → k = 2 (t_c + 50 ms).
  EXPECT_EQ(p.rec.k, 2);
  EXPECT_TRUE(p.rec.from_segment);
  EXPECT_FALSE(p.rec.presolved);
  EXPECT_FALSE(p.rec.cold_retry);
  const DecelPlanSnapshot& s = p.out;
  EXPECT_EQ(s.k0, 2);
  EXPECT_EQ(s.n_nodes, 12);
  EXPECT_EQ(s.t0_ns, t_c + 2 * kDt);
  EXPECT_EQ(s.t0_ns + s.n_nodes * s.dt_ns, first.t0_ns + first.n_nodes * first.dt_ns)
      << "the stop end moved";
  double node0 = 0.0;
  double drift = 0.0;
  for (int i = 0; i <= s.n_nodes; ++i) {
    for (int j = 0; j < 6; ++j) {
      const double old_q = first.q[Idx(i + 2, j)];
      const double d = std::fabs(s.q[Idx(i, j)] - old_q);
      if (i == 0) {
        node0 = std::max({node0, d, std::fabs(s.qd[Idx(0, j)] - first.qd[Idx(2, j)]),
                          std::fabs(s.qdd[Idx(0, j)] - first.qdd[Idx(2, j)])});
      }
      drift = std::max(drift, d);
    }
  }
  EXPECT_LT(node0, 1e-9) << "node 0 must be the followed segment at t_eff";
  EXPECT_LT(drift, 5e-3) << "a replan of the same stop moved the path";
  char drift_s[32];
  std::snprintf(drift_s, sizeof(drift_s), "%.3e", drift);
  RecordProperty("replan_path_drift_rad", drift_s);
}

TEST(DecelPlanner, PostCatchWithoutAPublishSolvesColdToTheSameEnd) {
  Planner p(Arm6());
  const std::int64_t t_c = kT0 + kTArm + 60 * kMs;
  const std::int64_t now = t_c - kTArm + 30 * kMs;  // first wake already past t_c
  SetClock(now);
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))))
      << DecelOutcomeName(p.rec.outcome);
  EXPECT_TRUE(p.rec.presolved);
  EXPECT_EQ(p.out.t0_ns + p.out.n_nodes * p.out.dt_ns, t_c + 14 * kDt);
}

TEST(DecelPlanner, TheLastReplanGridPointStillPublishes) {
  // now_lead = t_c + 70 ms → earliest t_c + 94 ms → k = 4 = k_max.
  Planner p(Arm6());
  const std::int64_t t_c = kT0 + kTArm;
  const std::int64_t now = t_c - kTArm + 70 * kMs;
  SetClock(now);
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))))
      << DecelOutcomeName(p.rec.outcome);
  EXPECT_EQ(p.rec.k, 4);
  EXPECT_EQ(p.out.n_nodes, 10);
  EXPECT_EQ(p.out.t0_ns + p.out.n_nodes * p.out.dt_ns, t_c + 14 * kDt);
}

TEST(DecelPlanner, PastTheReplanWindowPublishesNothing) {
  Planner p(Arm6());
  const std::int64_t t_c = kT0 + kTArm;
  // now_lead + 24 ms beyond t_c + 4Δ → k = 5 > k_max.
  const std::int64_t now = t_c - kTArm + 4 * kDt;
  SetClock(now);
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kPastReplanWindow) << DecelOutcomeName(p.rec.outcome);
  EXPECT_GT(p.rec.k, 4);
}

TEST(DecelPlanner, OverBudgetWithholds) {
  {
    // Every clock read advances the clock by the step, so start → end is one
    // step: 25 ms > budget 20 ms.
    Planner p(Arm6());
    const std::int64_t t_c = kT0 + kTArm + 90 * kMs;
    SetClock(kT0, 25 * kMs);
    EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0 - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))));
    EXPECT_EQ(p.rec.outcome, DecelOutcome::kBudget) << DecelOutcomeName(p.rec.outcome);
    EXPECT_EQ(p.rec.solve_ns, 25 * kMs);
  }
  // kLate has no test: t_eff ≥ start_lead + budget + 2·control_dt by
  // construction, so a solve within budget cannot end past it (the check is
  // the guard against that arithmetic changing).
  SetClock(kT0);
}

TEST(DecelPlanner, EachSlackThresholdWithholdsOnItsOwn) {
  // η'_τ 0.05: the static torque at the stop posture alone exceeds it, so both
  // slacks are positive. The solve is deterministic, so each threshold can be
  // put just below (withheld) or exactly at (published, `<=`) its own value
  // while the other is wide open — each comparison has to fire alone.
  const std::int64_t t_c = kT0 + kTArm + 80 * kMs;
  auto run = [&](double slack_max, double slack_terminal_max) {
    DecelPlannerParams params{};
    params.eta_tau = 0.05;
    params.slack_max = slack_max;
    params.slack_terminal_max = slack_terminal_max;
    Planner p(Arm6(), params);
    SetClock(kT0);
    static_cast<void>(p.Step(Rt(p.arm, t_c, kT0 - 2 * kMs, p.arm.q_nominal, EntryVelocity(6))));
    return p.rec;
  };
  const DecelRecord open = run(1.0, 1.0);
  ASSERT_EQ(open.outcome, DecelOutcome::kReady)
      << DecelOutcomeName(open.outcome) << " " << DecelMpcReasonName(open.core_reason);
  ASSERT_GT(open.slack_terminal_max, 1e-3) << "fixture premise: a positive terminal slack";
  ASSERT_GT(open.slack_max, 1e-3);
  const double s = open.slack_max;
  const double st = open.slack_terminal_max;
  EXPECT_EQ(run(s - 1e-6, 1.0).outcome, DecelOutcome::kSlack) << "slack_max threshold";
  EXPECT_EQ(run(1.0, st - 1e-6).outcome, DecelOutcome::kSlack) << "slack_terminal_max threshold";
  EXPECT_EQ(run(s, st).outcome, DecelOutcome::kReady) << "at the thresholds is within them";
}

TEST(DecelPlanner, StaleRtReportIsNotExtrapolated) {
  Planner p(Arm6());
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 80 * kMs;
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0 - rtc::catching::kDecelMaxRtStateAgeNs - 1,
                         p.arm.q_nominal, EntryVelocity(6))));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kStaleState) << "an RT stall";
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0 + 1, p.arm.q_nominal, EntryVelocity(6))));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kStaleState) << "a report from the future";
  EXPECT_TRUE(p.Step(Rt(p.arm, t_c, kT0 - rtc::catching::kDecelMaxRtStateAgeNs, p.arm.q_nominal,
                        EntryVelocity(6))))
      << DecelOutcomeName(p.rec.outcome);
}

TEST(DecelPlanner, UnusableStateIsNoState) {
  Planner p(Arm6());
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 80 * kMs;
  const PlannerRtState good = Rt(p.arm, t_c, kT0, p.arm.q_nominal, EntryVelocity(6));
  for (auto mutate : std::vector<void (*)(PlannerRtState&)>{
           [](PlannerRtState& s) { s.valid = false; },
           [](PlannerRtState& s) { s.plan_active = false; },
           [](PlannerRtState& s) { s.plan_t_c_ns = 0; },
           [](PlannerRtState& s) { s.cmd_seeded = false; },
           [](PlannerRtState& s) { s.nv = 7; },
       }) {
    PlannerRtState s = good;
    mutate(s);
    EXPECT_FALSE(p.Step(s));
    EXPECT_EQ(p.rec.outcome, DecelOutcome::kNoState);
  }
  DecelPlanner unconfigured;
  DecelPlanSnapshot out{};
  DecelRecord rec{};
  EXPECT_FALSE(unconfigured.Plan(good, out, rec));
  EXPECT_EQ(rec.outcome, DecelOutcome::kOff);
}

TEST(DecelPlanner, AccelerationComesFromTheWakeToWakeDifference) {
  Planner p(Arm6());
  // First wake 120 ms before t_c (not due: only the estimate runs), second
  // 25 ms later and due (95 ms).
  const std::int64_t t_c = kT0 + kTArm + 120 * kMs;
  const Eigen::VectorXd qd0 = EntryVelocity(6);
  Eigen::VectorXd qd1 = qd0;
  qd1[1] += 0.1;  // +0.1 rad/s over 25 ms → q̈ = 4 rad/s² on model joint 1
  SetClock(kT0);
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0, p.arm.q_nominal, qd0)));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kNotDue);
  SetClock(kT0 + 25 * kMs);
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, kT0 + 25 * kMs, p.arm.q_nominal, qd1)))
      << DecelOutcomeName(p.rec.outcome) << " " << DecelMpcReasonName(p.rec.core_reason);
  EXPECT_TRUE(p.rec.qdd_trusted);
  const int d = Dev(p.arm, 1);
  EXPECT_NEAR(p.out.qdd[Idx(0, d)], 4.0, 1e-9);
  const double h = p.rec.h_s;
  EXPECT_NEAR(p.out.qd[Idx(0, d)], qd1[1] + 4.0 * h, 1e-9);
  // A difference over more than 0.2 s is not trusted.
  Planner q(Arm6());
  SetClock(kT0);
  const std::int64_t t_c2 = kT0 + kTArm + 600 * kMs;
  EXPECT_FALSE(q.Step(Rt(q.arm, t_c2, kT0, q.arm.q_nominal, qd0)));
  SetClock(kT0 + 510 * kMs);
  ASSERT_TRUE(q.Step(Rt(q.arm, t_c2, kT0 + 510 * kMs, q.arm.q_nominal, qd1)))
      << DecelOutcomeName(q.rec.outcome);
  EXPECT_FALSE(q.rec.qdd_trusted);
  EXPECT_EQ(q.out.qdd[Idx(0, d)], 0.0);
}

TEST(DecelPlanner, AccelerationEstimateIsCappedAndSurvivesFastWakes) {
  // Wakes 3 ms apart (a burst of trajectory signals), each under the 5 ms
  // floor: the reference sample must stay the first one, so the third wake
  // (6 ms after it) still gets an estimate — replacing it on every wake would
  // leave every span at 3 ms and the estimate dead. The 1 rad/s step over
  // 6 ms (≈167 rad/s²) is clamped to the fixture's 20 rad/s² cap.
  Planner p(Arm6());
  const std::int64_t t_c = kT0 + kTArm + 105 * kMs;  // due only at the third wake
  const Eigen::VectorXd qd0 = EntryVelocity(6);
  Eigen::VectorXd qd1 = qd0;
  qd1[1] += 1.0;
  SetClock(kT0);
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0, p.arm.q_nominal, qd0)));
  SetClock(kT0 + 3 * kMs);
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0 + 3 * kMs, p.arm.q_nominal, qd0)));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kNotDue);
  SetClock(kT0 + 6 * kMs);
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, kT0 + 6 * kMs, p.arm.q_nominal, qd1)))
      << DecelOutcomeName(p.rec.outcome) << " " << DecelMpcReasonName(p.rec.core_reason);
  EXPECT_TRUE(p.rec.qdd_trusted);
  EXPECT_DOUBLE_EQ(p.out.qdd[Idx(0, Dev(p.arm, 1))], 20.0);
}

TEST(DecelPlanner, VelocityIsProjectedIntoTheBox) {
  Planner p(Arm6());
  SetClock(kT0);
  // Model joint 0 (rating 2 rad/s, box 1.8) sits in device slot 2, whose own
  // model joint is rated 3 rad/s (box 2.7): a planner that skipped the
  // device↔model map would read 2.0 as within the box and publish it.
  ASSERT_NE(Dev(p.arm, 0), 0);
  Eigen::VectorXd qd = EntryVelocity(6);
  qd[0] = 2.0;
  const std::int64_t t_c = kT0 + kTArm + 50 * kMs;
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, kT0, p.arm.q_nominal, qd))) << DecelOutcomeName(p.rec.outcome);
  EXPECT_TRUE(p.rec.x0_clamped);
  EXPECT_TRUE(p.out.x0_clamped);
  EXPECT_DOUBLE_EQ(p.out.qd[Idx(0, Dev(p.arm, 0))], 0.9 * 2.0);
  for (int m = 1; m < 6; ++m) {
    EXPECT_DOUBLE_EQ(p.out.qd[Idx(0, Dev(p.arm, m))], qd[m]) << "unclamped joint " << m;
  }
}

TEST(DecelPlanner, NonFiniteStateIsRefusedBeforeTheProjection) {
  Planner p(Arm6());
  SetClock(kT0);
  Eigen::VectorXd qd = EntryVelocity(6);
  qd[2] = kNan;  // std::clamp passes a NaN through — only the check stops it
  const std::int64_t t_c = kT0 + kTArm + 50 * kMs;
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c, kT0, p.arm.q_nominal, qd)));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kInputNonFinite);
}

TEST(DecelPlanner, ANewFollowedPlanStartsANewStop) {
  Planner p(Arm6());
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 50 * kMs;
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, kT0, p.arm.q_nominal, EntryVelocity(6))));
  p.Publish(1, kT0);
  // Plan 8 with a later t_c: gated by t_pre again, not "up to date".
  const std::int64_t t_c2 = kT0 + kTArm + 400 * kMs;
  EXPECT_FALSE(p.Step(Rt(p.arm, t_c2, kT0, p.arm.q_nominal, EntryVelocity(6), 8)));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kNotDue) << DecelOutcomeName(p.rec.outcome);
}

TEST(DecelPlanner, ATrialResetDropsThePublishedStop) {
  Planner p(Arm6());
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 50 * kMs;
  const PlannerRtState rt = Rt(p.arm, t_c, kT0, p.arm.q_nominal, EntryVelocity(6));
  ASSERT_TRUE(p.Step(rt));
  p.Publish(1, kT0);
  EXPECT_FALSE(p.Step(rt));
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kUpToDate) << "same plan, same grid point";
  p.planner.ResetTrial();
  EXPECT_TRUE(p.Step(rt)) << "after a reset the same plan is a new stop: "
                          << DecelOutcomeName(p.rec.outcome);
}

// ── 3b. The cycle's decel step (MD-29) ───────────────────────────────────────

struct CycleBoxes {
  rtc::SeqLock<rtc::catching::TrajectorySnapshot> traj{};
  rtc::SeqLock<rtc::catching::CovarianceSnapshot> cov{};
  rtc::SeqLock<PlannerRtState> rt{};
  rtc::SeqLock<rtc::catching::PlanSnapshot> plan{};
  rtc::SeqLock<DecelPlanSnapshot> decel{};
};

// Heap-allocated: the boxes and the cycle's scratch are tens of KB.
struct CycleRig {
  Arm arm = Arm6();
  CycleBoxes boxes;
  rtc::catching::PlannerCycle cycle;

  explicit CycleRig(bool bind_decel = true) {
    rtc::catching::PlannerParams params;
    params.enabled = true;
    params.decel.enabled = true;
    cycle.Configure(params);
    cycle.SetClock(&FakeClock);
    rtc::catching::PlannerCycleIo io{&boxes.traj, &boxes.cov, &boxes.rt, &boxes.plan,
                                     bind_decel ? &boxes.decel : nullptr};
    EXPECT_TRUE(cycle.Bind(io));
    std::string err;
    EXPECT_TRUE(cycle.ConfigureDecel(PlannerModelOf(arm), Consts(), &err)) << err;
  }
};

TEST(DecelCycle, StoresTheStopInCommittedAndKeepsTheOutcomeIdle) {
  auto rig = std::make_unique<CycleRig>();
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 70 * kMs;
  rig->boxes.rt.Store(Rt(rig->arm, t_c, kT0 - 2 * kMs, rig->arm.q_nominal, EntryVelocity(6)));
  const auto rec = rig->cycle.Run(NowReal{kT0});
  EXPECT_EQ(rec.outcome, rtc::catching::CycleOutcome::kIdle)
      << "MD-29: the outcome is the search's";
  ASSERT_EQ(rec.decel.outcome, DecelOutcome::kPublished) << DecelOutcomeName(rec.decel.outcome);
  EXPECT_EQ(rec.decel.decel_seq, 1U);
  EXPECT_EQ(rec.decel.publish_ns, kT0);
  const DecelPlanSnapshot s = rig->boxes.decel.Load();
  EXPECT_TRUE(s.valid);
  EXPECT_EQ(s.decel_seq, 1U);
  EXPECT_EQ(s.publish_ns, kT0);
  EXPECT_EQ(rig->cycle.LastDecelSeq(), 1U);
  EXPECT_EQ(rig->cycle.LastPlanId(), 0U) << "no PlanSnapshot published";
  EXPECT_EQ(rig->boxes.plan.sequence(), 0U);
  // The next wake at the same grid point stores nothing new.
  const auto again = rig->cycle.Run(NowReal{kT0});
  EXPECT_EQ(again.decel.outcome, DecelOutcome::kUpToDate);
  EXPECT_EQ(rig->boxes.decel.Load().decel_seq, 1U);
}

TEST(DecelCycle, WithdrawsTheSegmentOnATrialReset) {
  auto rig = std::make_unique<CycleRig>();
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 70 * kMs;
  PlannerRtState rt = Rt(rig->arm, t_c, kT0 - 2 * kMs, rig->arm.q_nominal, EntryVelocity(6));
  rig->boxes.rt.Store(rt);
  ASSERT_EQ(rig->cycle.Run(NowReal{kT0}).decel.outcome, DecelOutcome::kPublished);
  rt.reset_epoch = 2;
  rt.mode = static_cast<std::uint8_t>(Mode::kIdle);
  rig->boxes.rt.Store(rt);
  const auto rec = rig->cycle.Run(NowReal{kT0 + 10 * kMs});
  EXPECT_TRUE(rec.reset_seen);
  EXPECT_FALSE(rig->boxes.decel.Load().valid) << "the ended trial's segment stays in the box";
}

struct PlanSwap {
  rtc::SeqLock<PlannerRtState>* rt;
  PlannerRtState next;
};

void SwapPlan(void* ctx) noexcept {
  auto* s = static_cast<PlanSwap*>(ctx);
  s->rt->Store(s->next);
}

TEST(DecelCycle, DropsASolveTheFollowedPlanLeft) {
  auto rig = std::make_unique<CycleRig>();
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 70 * kMs;
  const PlannerRtState rt = Rt(rig->arm, t_c, kT0 - 2 * kMs, rig->arm.q_nominal, EntryVelocity(6));
  // The first wake sees reset epoch 1 as a change from 0 and withdraws the
  // (empty) box — consume it in a mode with nothing to plan.
  PlannerRtState idle = rt;
  idle.mode = static_cast<std::uint8_t>(Mode::kIdle);
  rig->boxes.rt.Store(idle);
  static_cast<void>(rig->cycle.Run(NowReal{kT0}));
  rig->boxes.rt.Store(rt);
  for (auto mutate : std::vector<void (*)(PlannerRtState&)>{
           [](PlannerRtState& s) { s.plan_id = 8; },
           [](PlannerRtState& s) { s.reset_epoch = 2; },
           [](PlannerRtState& s) { s.activation_generation = 4; },
           [](PlannerRtState& s) { s.mode = static_cast<std::uint8_t>(Mode::kHold); },
           [](PlannerRtState& s) { s.plan_t_c_ns += 1; },
           [](PlannerRtState& s) { s.plan_active = false; },
           [](PlannerRtState& s) { s.valid = false; },
           [](PlannerRtState& s) {
             s.decel_active = true;
             s.decel_seq = 9;
           },
       }) {
    PlanSwap swap{&rig->boxes.rt, rt};
    mutate(swap.next);
    rig->cycle.SetPostDecelHookForTesting(&SwapPlan, &swap);
    rig->boxes.rt.Store(rt);
    const std::uint32_t before = rig->boxes.decel.sequence();
    const auto rec = rig->cycle.Run(NowReal{kT0});
    EXPECT_EQ(rec.decel.outcome, DecelOutcome::kSuperseded) << DecelOutcomeName(rec.decel.outcome);
    // A reset seen by the NEXT wake stores valid = false; this wake stored
    // nothing.
    EXPECT_EQ(rig->boxes.decel.sequence(), before);
    EXPECT_EQ(rig->cycle.LastDecelSeq(), 0U);
  }
  rig->cycle.SetPostDecelHookForTesting(nullptr, nullptr);
}

TEST(DecelCycle, ReplansInDecelAfterTheCatch) {
  auto rig = std::make_unique<CycleRig>();
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 70 * kMs;
  PlannerRtState rt = Rt(rig->arm, t_c, kT0 - 2 * kMs, rig->arm.q_nominal, EntryVelocity(6));
  rig->boxes.rt.Store(rt);
  ASSERT_EQ(rig->cycle.Run(NowReal{kT0}).decel.outcome, DecelOutcome::kPublished);
  const std::int64_t now = t_c - kTArm + 10 * kMs;
  SetClock(now);
  rt.mode = static_cast<std::uint8_t>(Mode::kDecel);
  rt.rt_state_ns = now - 2 * kMs;
  rig->boxes.rt.Store(rt);
  const auto rec = rig->cycle.Run(NowReal{now});
  EXPECT_EQ(rec.outcome, rtc::catching::CycleOutcome::kIdle);
  ASSERT_EQ(rec.decel.outcome, DecelOutcome::kPublished) << DecelOutcomeName(rec.decel.outcome);
  const DecelPlanSnapshot s = rig->boxes.decel.Load();
  EXPECT_EQ(s.decel_seq, 2U);
  EXPECT_EQ(s.k0, 2);
  EXPECT_EQ(s.t0_ns + s.n_nodes * s.dt_ns, t_c + 14 * kDt);
}

TEST(DecelCycle, WithoutADecelBoxThereIsNoDecelStep) {
  auto rig = std::make_unique<CycleRig>(/*bind_decel=*/false);
  SetClock(kT0);
  const std::int64_t t_c = kT0 + kTArm + 70 * kMs;
  rig->boxes.rt.Store(Rt(rig->arm, t_c, kT0 - 2 * kMs, rig->arm.q_nominal, EntryVelocity(6)));
  EXPECT_EQ(rig->cycle.Run(NowReal{kT0}).decel.outcome, DecelOutcome::kOff);
  rig->cycle.ClearDecel();
  EXPECT_FALSE(rig->cycle.DecelConfigured());
}

// ── 4. Allocation (MD-23) ────────────────────────────────────────────────────

struct Counts {
  std::size_t op_new{0};
  std::size_t c_malloc{0};
};

Counts GatedStep(Planner& p, const PlannerRtState& rt, bool& ok) {
  Counts c;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    ok = p.Step(rt);
    c.op_new = new_gate.count();
    c.c_malloc = malloc_gate.count();
  }
  return c;
}

TEST(DecelPlanner, AllocatesNothingOutsideProxQp) {
  {
    // Positive control: an allocation inside pinocchio's shared object.
    const Arm arm = Arm6();
    rtc::testing::ScopedMallocGate gate;
    pinocchio::Data probe(*arm.model);
    ASSERT_GT(gate.count(), 0U);
  }
  Planner p(Arm7());
  const int n = 7;
  const std::int64_t t_c = kT0 + kTArm + 60 * kMs;
  SetClock(kT0);
  // Before t_pre: every allocation here would be ours.
  {
    const std::int64_t far = kT0 + kTArm + 300 * kMs;
    SetClock(kT0);
    bool ok_far = true;
    const Counts c_far =
        GatedStep(p, Rt(p.arm, far, kT0 - 2 * kMs, p.arm.q_nominal, EntryVelocity(n)), ok_far);
    EXPECT_EQ(p.rec.outcome, DecelOutcome::kNotDue);
    EXPECT_EQ(c_far.op_new + c_far.c_malloc, 0U) << "not-due path";
  }
  // Warm-up: the cold path once, published.
  ASSERT_TRUE(p.Step(Rt(p.arm, t_c, kT0 - 2 * kMs, p.arm.q_nominal, EntryVelocity(n))));
  p.Publish(1, kT0);
  bool ok = false;
  // Paths that end before any QP: every allocation here would be ours.
  SetClock(kT0 + 5 * kMs);
  Counts c = GatedStep(p, Rt(p.arm, t_c, kT0 + 3 * kMs, p.arm.q_nominal, EntryVelocity(n)), ok);
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kUpToDate);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "up-to-date path";
  PlannerRtState stateless = Rt(p.arm, t_c, kT0, p.arm.q_nominal, EntryVelocity(n));
  stateless.cmd_seeded = false;
  c = GatedStep(p, stateless, ok);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "no-state path";
  // Post-catch, prediction + shift, then the core refuses x0 before its QP
  // (q within m_q of a limit): the whole prepare path, no ProxQP.
  const std::int64_t now = t_c - kTArm + 10 * kMs;
  SetClock(now);
  Eigen::VectorXd q_edge = p.arm.q_nominal;
  q_edge[3] = p.arm.model->upperPositionLimit[3] - 0.01;
  c = GatedStep(p, Rt(p.arm, t_c, now - 2 * kMs, q_edge, EntryVelocity(n)), ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kSolveFailed);
  EXPECT_EQ(p.rec.core_reason, rtc::catching::DecelMpcReason::kInitialStateOutsideBox);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "prediction / shift / pre-QP rejection path";
  PlannerRtState nan_state = Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(n));
  nan_state.q_cmd[0] = kNan;
  c = GatedStep(p, nan_state, ok);
  EXPECT_EQ(p.rec.outcome, DecelOutcome::kInputNonFinite);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "non-finite path";
  // The full replan (prediction, shift, warm QP, pack): operator new stays at
  // zero; the C mallocs are ProxQP's (#654), recorded.
  PlannerRtState follow = Rt(p.arm, t_c, now - 2 * kMs, p.arm.q_nominal, EntryVelocity(n));
  follow.decel_active = true;
  follow.decel_seq = 1;
  c = GatedStep(p, follow, ok);
  EXPECT_TRUE(ok) << DecelOutcomeName(p.rec.outcome) << " "
                  << DecelMpcReasonName(p.rec.core_reason);
  EXPECT_EQ(c.op_new, 0U) << "operator new in the replan path";
  RecordProperty("replan_qp_solver_mallocs", std::to_string(c.c_malloc));
  std::printf("[alloc] decel replan: %zu C mallocs (ProxQP, #654)\n", c.c_malloc);
}

// ── 5. Timing at the shipped horizon (MD-24, MD-26) + informational records ──

[[nodiscard]] bool OptimisedBuild() {
  const std::string bt = RTC_TEST_BUILD_TYPE;
  return bt == "Release" || bt == "RelWithDebInfo" || bt == "MinSizeRel";
}

std::int64_t RealClock() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double P99(std::vector<double> v) {
  if (v.empty()) {
    return kNan;
  }
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, v.size() * 99 / 100)];
}

// The decision flow runs on the fake clock (so the budget gate never trips on
// a slow host and every sample reaches the solve); the solve time is measured
// around Plan() on the real clock — the same span rec.solve_ns covers.
void RunShippedTiming(const Arm& arm, const std::string& key) {
  const int n = arm.model->nv;
  const int samples = OptimisedBuild() ? 200 : 10;
  DecelPlanner planner;
  std::string err;
  ASSERT_TRUE(
      planner.Configure(PlannerModelOf(arm), Consts(), DecelPlannerParams{}, &FakeClock, &err))
      << err;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  std::vector<double> cold_ms;
  std::vector<double> follow_ms;
  std::vector<double> report_ms;
  int cold_fail = 0;
  int follow_fail = 0;
  int report_fail = 0;
  int report_cold_retries = 0;
  double armature_ratio_max = 0.0;
  int wrist_heavy = 0;
  DecelPlanSnapshot out{};
  DecelRecord rec{};
  pinocchio::Data data(*arm.model);
  std::map<std::string, int> reasons;
  const auto timed = [&](const PlannerRtState& rt, std::vector<double>& ms, int& fail) {
    const std::int64_t a = RealClock();
    const bool ok = planner.Plan(rt, out, rec);
    const std::int64_t b = RealClock();
    if (ok) {
      ms.push_back(static_cast<double>(b - a) * 1e-6);
    } else {
      ++fail;
      const std::string why = std::string(DecelOutcomeName(rec.outcome)) + "/" +
                              DecelMpcReasonName(rec.core_reason) + "/qp" +
                              std::to_string(rec.qp_status);
      ++reasons[why];
    }
    return ok;
  };
  for (int i = 0; i < samples; ++i) {
    Eigen::VectorXd q = arm.q_nominal;
    Eigen::VectorXd qd(n);
    for (int m = 0; m < n; ++m) {
      q[m] += 0.2 * uni(rng);
      qd[m] = 0.6 * 0.9 * arm.qd_max[static_cast<std::size_t>(m)] * uni(rng);
    }
    const auto id = static_cast<std::uint32_t>(i + 1);
    // Cold: the first solve, t_pre before t_c (MD-26).
    SetClock(kT0);
    const std::int64_t t_c = kT0 + kTArm + 90 * kMs;
    if (!timed(Rt(arm, t_c, kT0 - 2 * kMs, q, qd, id), cold_ms, cold_fail)) {
      continue;
    }
    out.decel_seq = 1;
    out.publish_ns = kT0;
    planner.NotePublished(out);
    const DecelPlanSnapshot first = out;
    // Informational (D-3 / MD-25): what the MJCF armature would add to the
    // torque the plan asks for; (D-11) the static wrist torque at the stop
    // posture as a fraction of τ_max.
    for (int k = 0; k <= first.n_nodes; ++k) {
      for (int m = 0; m < n; ++m) {
        const auto u = static_cast<std::size_t>(m);
        armature_ratio_max =
            std::max(armature_ratio_max,
                     std::fabs(arm.armature[u] * first.qdd[Idx(k, Dev(arm, m))]) / arm.tau_max[u]);
      }
    }
    Eigen::VectorXd q_stop(n);
    for (int m = 0; m < n; ++m) {
      q_stop[m] = first.q[Idx(first.n_nodes, Dev(arm, m))];
    }
    const Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(*arm.model, data, q_stop);
    double wrist = 0.0;
    for (int m = n - 3; m < n; ++m) {
      wrist = std::max(wrist, std::fabs(g[m]) / arm.tau_max[static_cast<std::size_t>(m)]);
    }
    wrist_heavy += wrist >= 0.7 ? 1 : 0;

    // Post-catch replan, now_lead = t_c + 10 ms → k = 2.
    const std::int64_t now = t_c - kTArm + 10 * kMs;
    SetClock(now);
    // (a) The RT follows the published segment (E1-F04's report, path (i)).
    PlannerRtState follow = Rt(arm, t_c, now - 2 * kMs, q, qd, id);
    follow.mode = static_cast<std::uint8_t>(Mode::kDecel);
    follow.decel_active = true;
    follow.decel_seq = 1;
    timed(follow, follow_ms, follow_fail);
    // (b) The RT runs something else (F03: the closed-form DECEL) — its
    // report drifted from the segment: often a trust-region refusal and a cold
    // retry, inside the same budget.
    planner.NotePublished(first);  // back to the pre-catch segment
    PlannerRtState report = follow;
    report.decel_active = false;
    for (int m = 0; m < n; ++m) {
      const auto d = static_cast<std::size_t>(Dev(arm, m));
      report.q_cmd[d] = q[m] + qd[m] * 0.1;
      report.qd_cmd[d] = 0.7 * qd[m];
    }
    if (timed(report, report_ms, report_fail)) {
      report_cold_retries += rec.cold_retry ? 1 : 0;
    }
  }
  SetClock(kT0);
  const double cold_p99 = P99(cold_ms);
  const double follow_p99 = P99(follow_ms);
  const double report_p99 = P99(report_ms);
  ::testing::Test::RecordProperty(key + "_cold_p99_ms", std::to_string(cold_p99));
  ::testing::Test::RecordProperty(key + "_replan_follow_p99_ms", std::to_string(follow_p99));
  ::testing::Test::RecordProperty(key + "_replan_report_p99_ms", std::to_string(report_p99));
  ::testing::Test::RecordProperty(key + "_cold_fail", std::to_string(cold_fail));
  ::testing::Test::RecordProperty(key + "_replan_follow_fail", std::to_string(follow_fail));
  ::testing::Test::RecordProperty(key + "_replan_report_fail", std::to_string(report_fail));
  ::testing::Test::RecordProperty(key + "_replan_report_cold_retries",
                                  std::to_string(report_cold_retries));
  ::testing::Test::RecordProperty(key + "_armature_tau_ratio_max",
                                  std::to_string(armature_ratio_max));
  ::testing::Test::RecordProperty(key + "_stop_wrist_gravity_ge_0p7", std::to_string(wrist_heavy));
  ::testing::Test::RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
  std::printf(
      "[ record ] %s (N_s 14 x 25 ms): cold p99 %.2f ms (%zu ok, %d fail) | replan following "
      "p99 %.2f ms (%zu ok, %d fail) | replan from report p99 %.2f ms (%zu ok, %d fail, %d cold "
      "retries) | armature |a qdd|/tau_max max %.3f | stop wrist gravity >= 0.7 tau_max in %d "
      "/ %zu\n",
      key.c_str(), cold_p99, cold_ms.size(), cold_fail, follow_p99, follow_ms.size(), follow_fail,
      report_p99, report_ms.size(), report_fail, report_cold_retries, armature_ratio_max,
      wrist_heavy, cold_ms.size());
  for (const auto& [why, count] : reasons) {
    std::printf("[ record ] %s failure %s x%d\n", key.c_str(), why.c_str(), count);
  }
  if (OptimisedBuild()) {
    EXPECT_LT(cold_p99, 20.0) << "cold p99 over budget_s — MD-24's fallback is 7 x 0.05 s";
    EXPECT_LT(follow_p99, 20.0);
    EXPECT_LT(report_p99, 20.0);
    EXPECT_EQ(cold_fail, 0);
    EXPECT_EQ(follow_fail, 0);
  }
}

TEST(DecelPlannerTiming, ShippedHorizonTiming6R) {
  RunShippedTiming(Arm6(), "real_6dof");
}

TEST(DecelPlannerTiming, ShippedHorizonTiming7R) {
  RunShippedTiming(Arm7(), "real_7dof");
}

}  // namespace
