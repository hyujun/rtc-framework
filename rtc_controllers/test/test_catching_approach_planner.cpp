// E1-F08 (#661): the decel planner's APPROACH–stop solves (decel_planner.hpp
// §APPROACH–stop) on a fake clock — PlanFirst, Replan, the ball target, the
// between-node speed check and the allocation boundary. NOT the E1-F07 core
// suite (test_catching_decel_mpc_approach.cpp drives DecelMpc alone); the
// cycle that wires these into a wake is test_catching_approach_cycle.cpp.
// Spec on #661 (MD-55 – MD-64):
//   configure      ConfigureBuildsCatchCoresInTheStopCoresBox, ...
//   first solve    FirstSolvePicksTheLargestPreCatchCountThatFits, ...
//   publish gates  EachGateWithholdsOnItsOwn, BetweenNodeSpeed*
//   replans        ASamePointResolveIsWarmAndKeepsNodeZero, AdvanceIsColdAndStartsOnTheSource,
//                  PreCatchHandsOverToTheStopCores, TheSourceIsWhatTheRtReports, ...
//   allocation     PathsBeforeTheSolveAllocateNothing, SolvesAllocateNothingOutsideProxQp
//
// Fixtures break representation symmetries: the device order is a
// non-identity permutation of the model order, T_arm ≠ 0 (real ≠ lead axis),
// instants are realistic absolute steady ns, and both a 6- and a 7-joint arm
// run.
#include "rtc_controllers/catching/decel_planner.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using rtc::catching::DecelBallTarget;
using rtc::catching::DecelKind;
using rtc::catching::DecelMpcReason;
using rtc::catching::DecelNodeTimeNs;
using rtc::catching::DecelOutcome;
using rtc::catching::DecelOutcomeName;
using rtc::catching::DecelPlanner;
using rtc::catching::DecelPlannerConstants;
using rtc::catching::DecelPlannerModel;
using rtc::catching::DecelPlannerParams;
using rtc::catching::DecelPlanSnapshot;
using rtc::catching::DecelRecord;
using rtc::catching::kMaxDecelNv;
using rtc::catching::Mode;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kTArm = 30 * kMs;                   // T_arm ≠ 0
constexpr std::int64_t kH = 2 * kMs;                       // control_dt
constexpr std::int64_t kDtPre = 100 * kMs;                 // Δ_pre (MD-54)
constexpr std::int64_t kDt = 50 * kMs;                     // Δ_s (MD-54)
constexpr std::int64_t kFirst = 35 * kMs;                  // budget.first_s
constexpr std::int64_t kReplan = 25 * kMs;                 // budget.replan_s
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;  // realistic steady ns
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// ── Clock ────────────────────────────────────────────────────────────────────
std::atomic<std::int64_t> g_now{kT0};
std::atomic<std::int64_t> g_step{0};

std::int64_t FakeClock() noexcept {
  return g_now.fetch_add(g_step.load());
}

void SetClock(std::int64_t now, std::int64_t step = 0) {
  g_now.store(now);
  g_step.store(step);
}

// ── Arms (the shipped device ratings, listed in MODEL order) ─────────────────

struct Arm {
  std::shared_ptr<const pinocchio::Model> model;
  pinocchio::FrameIndex frame{0};
  Eigen::VectorXd q_nominal;             // model order
  std::array<int, 7> device_of_model{};  // non-identity permutation
  std::vector<double> qd_max;
  std::vector<double> tau_max;
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
  return a;
}

std::size_t Dev(const Arm& a, int m) {
  return static_cast<std::size_t>(a.device_of_model[static_cast<std::size_t>(m)]);
}

DecelPlannerModel PlannerModelOf(const Arm& a) {
  DecelPlannerModel pm;
  pm.arm = a.model;
  pm.catch_frame = a.frame;
  pm.nv = a.model->nv;
  for (int m = 0; m < pm.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    pm.device_of_model[u] = static_cast<int>(Dev(a, m));
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
  c.control_dt = static_cast<double>(kH) * 1e-9;
  c.budget_s = 0.020;
  c.report_lead_s = 2.0 * c.control_dt;
  c.v_eps = 1e-6;
  return c;
}

// The shipped approach profile (MD-54): 7 × 0.05 s [1, 1, 2, 3] after the
// catch, up to 6 × 0.1 s before it, k_max 2.
DecelPlannerParams ApproachParams() {
  DecelPlannerParams p;
  p.enabled = true;
  p.n_nodes = 7;
  p.dt_s = 0.05;
  p.blocks = {1, 1, 2, 3};
  p.n_blocks = 4;
  p.k_max = 2;
  p.n_pre_max = 6;
  p.dt_pre_s = 0.1;
  p.horizon_explicit = true;
  return p;
}

// A catch the arm reaches at `q_catch` (model order): the ball passes the
// catch frame's position along the frame's −z at `speed`, so the catch
// frame's +z IS the approach axis −v̂ (the search's a_d, and the IK's goal).
struct Catch {
  Eigen::VectorXd q_catch;  // model order
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a_d;
};

Catch CatchAt(const Arm& a, const Eigen::VectorXd& q_catch, double speed = 5.0) {
  pinocchio::Data data(*a.model);
  pinocchio::forwardKinematics(*a.model, data, q_catch);
  pinocchio::updateFramePlacement(*a.model, data, a.frame);
  Catch c;
  c.q_catch = q_catch;
  c.p = data.oMf[a.frame].translation();
  c.a_d = data.oMf[a.frame].rotation().col(2);
  c.v = -speed * c.a_d;
  return c;
}

Eigen::Vector3d FkPos(const Arm& a, const Eigen::VectorXd& q_model) {
  pinocchio::Data data(*a.model);
  pinocchio::forwardKinematics(*a.model, data, q_model);
  pinocchio::updateFramePlacement(*a.model, data, a.frame);
  return data.oMf[a.frame].translation();
}

PlanSnapshot PlanFor(const Arm& a, const Catch& c, std::int64_t t_c, std::uint32_t plan_id = 7) {
  PlanSnapshot p{};
  p.valid = true;
  p.plan_id = plan_id;
  p.t_c_ns = t_c;
  p.token.activation_generation = 3;
  p.token.generation = 5;
  p.nv = a.model->nv;
  for (int m = 0; m < p.nv; ++m) {
    p.q_star[Dev(a, m)] = c.q_catch[m];
  }
  for (int i = 0; i < 3; ++i) {
    const auto u = static_cast<std::size_t>(i);
    p.p_c[u] = c.p[i];
    p.v_c[u] = c.v[i];
    p.a_d[u] = c.a_d[i];
  }
  return p;
}

DecelBallTarget BallFor(const Catch& c, double sigma = 0.01) {
  DecelBallTarget b;
  b.valid = true;
  b.p_b = c.p;
  b.v_b = c.v;
  b.a_d = c.a_d;
  b.sigma_p = sigma * sigma * Eigen::Matrix3d::Identity();
  b.sigma_valid = true;
  return b;
}

// The RT at rest at `q` (model order), reported at real instant `rt_ns`.
PlannerRtState RestingRt(const Arm& a, const Eigen::VectorXd& q, std::int64_t rt_ns) {
  PlannerRtState s{};
  s.valid = true;
  s.activation_generation = 3;
  s.rt_iteration = 1000;
  s.rt_state_ns = rt_ns;
  s.reset_epoch = 1;
  s.mode = static_cast<std::uint8_t>(Mode::kTracking);
  s.nv = a.model->nv;
  s.cmd_seeded = true;
  for (int m = 0; m < s.nv; ++m) {
    s.q_cmd[Dev(a, m)] = q[m];
  }
  s.track_seen = true;
  s.track_generation = 5;
  return s;
}

// The RT following plan `plan_id` (t_c), reporting `pending` / `active`.
PlannerRtState FollowingRt(const Arm& a, const Eigen::VectorXd& q, std::int64_t rt_ns,
                           std::int64_t t_c, std::uint32_t pending, std::uint32_t active,
                           std::uint32_t plan_id = 7) {
  PlannerRtState s = RestingRt(a, q, rt_ns);
  s.mode = static_cast<std::uint8_t>(Mode::kApproach);
  s.plan_active = true;
  s.plan_id = plan_id;
  s.plan_t_c_ns = t_c;
  s.decel_pending = pending != 0;
  s.decel_pending_seq = pending;
  s.decel_active = active != 0;
  s.decel_seq = active;
  return s;
}

Eigen::VectorXd Offset(const Arm& a, double d) {
  Eigen::VectorXd q = a.q_nominal;
  for (int m = 0; m < q.size(); ++m) {
    q[m] += (m % 2 == 0 ? d : -d);
  }
  return q;
}

struct Rig {
  Arm arm;
  DecelPlanner planner;
  DecelPlanSnapshot out{};
  DecelRecord rec{};
  std::uint32_t seq{0};

  explicit Rig(Arm a, DecelPlannerParams params = ApproachParams(),
               DecelPlannerConstants consts = Consts())
      : arm(std::move(a)) {
    std::string err;
    EXPECT_TRUE(planner.Configure(PlannerModelOf(arm), consts, params, &FakeClock, &err)) << err;
  }

  // Publish what the last step produced the way the cycle does.
  std::uint32_t Publish(std::int64_t publish_ns) {
    out.decel_seq = ++seq;
    out.publish_ns = publish_ns;
    planner.NoteApproachPublished(out);
    return out.decel_seq;
  }
};

std::string Why(const DecelRecord& r) {
  return std::string(DecelOutcomeName(r.outcome)) + " / " +
         rtc::catching::DecelMpcReasonName(r.core_reason);
}

// The catch node's frame position of a published segment (model order).
Eigen::Vector3d CatchNodePos(const Arm& a, const DecelPlanSnapshot& p) {
  Eigen::VectorXd q(a.model->nv);
  for (int m = 0; m < q.size(); ++m) {
    q[m] = p.q[static_cast<std::size_t>(p.n_pre * kMaxDecelNv) + Dev(a, m)];
  }
  return FkPos(a, q);
}

// A first segment for a catch at t_c = now + lead, published as seq 1.
struct Started {
  Catch c;
  std::int64_t t_c{0};
  std::uint32_t seq{0};
};

Started StartPlan(Rig& r, std::int64_t now, std::int64_t lead, double reach = 0.03) {
  Started s;
  s.c = CatchAt(r.arm, Offset(r.arm, reach));
  s.t_c = now + kTArm + lead;
  SetClock(now);
  const bool ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, now - kH),
                                      PlanFor(r.arm, s.c, s.t_c), BallFor(s.c), r.out, r.rec);
  EXPECT_TRUE(ok) << Why(r.rec);
  s.seq = ok ? r.Publish(now) : 0;
  return s;
}

// ── 1. Configure ─────────────────────────────────────────────────────────────

TEST(ApproachPlanner, ConfigureBuildsCatchCoresInTheStopCoresBox) {
  for (Arm arm : {Arm6(), Arm7()}) {
    Rig r(std::move(arm));
    ASSERT_TRUE(r.planner.ApproachConfigured());
    // A successful Configure means every warm-up solve succeeded (MD-64): a
    // failed one fails the configure, so a measurement after it measures
    // warmed cores.
    for (int j = 1; j <= 6; ++j) {
      const auto& core = r.planner.ApproachCore(j);
      EXPECT_EQ(core.CatchNode(), j);
      EXPECT_EQ(core.NumNodes(), j + 7);
      const auto& cp = r.planner.ApproachCoreParams(j);
      EXPECT_TRUE(cp.catch_terms);
      EXPECT_DOUBLE_EQ(cp.dt_pre, 0.1);
      for (int k = 0; k <= 2; ++k) {
        // The stop core that takes over at t_c refuses a state outside its
        // own box: both boxes must be the same one.
        const auto& sp = r.planner.StopCoreParams(k);
        EXPECT_EQ(cp.eta_v, sp.eta_v) << j << "/" << k;
        EXPECT_EQ(cp.eta_tau, sp.eta_tau) << j << "/" << k;
        EXPECT_EQ(cp.m_q, sp.m_q) << j << "/" << k;
        EXPECT_EQ(sp.eta_v, 0.9);  // planner.gamma.eta_v, not the core default
      }
    }
  }
}

TEST(ApproachPlanner, WithoutAPreCatchGridItIsTheStopSegmentPlanner) {
  DecelPlannerParams p = ApproachParams();
  p.n_pre_max = 0;
  Rig r(Arm6(), p);
  EXPECT_FALSE(r.planner.ApproachConfigured());
  const Catch c = CatchAt(r.arm, Offset(r.arm, 0.03));
  SetClock(kT0);
  EXPECT_FALSE(r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                   PlanFor(r.arm, c, kT0 + 800 * kMs), BallFor(c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kOff);
  EXPECT_FALSE(
      r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, kT0 - kH, kT0 + 800 * kMs, 1, 0),
                       BallFor(c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kOff);
}

TEST(ApproachPlanner, ConfigureRefusesAPositionMarginTheTrustRegionCannotHold) {
  DecelPlannerParams p = ApproachParams();
  p.m_q = 0.1;  // the core's δ_tr
  DecelPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(PlannerModelOf(Arm6()), Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("trust region"), std::string::npos) << err;
}

// ── 2. The first solve ───────────────────────────────────────────────────────

TEST(ApproachPlanner, FirstSolvePicksTheLargestPreCatchCountThatFits) {
  Rig r(Arm6());
  const Catch c = CatchAt(r.arm, Offset(r.arm, 0.02));

  // n_pre = min(6, ⌊(t_c − now_lead − first − 2h) / Δ_pre⌋), ≥ 1.
  struct Row {
    std::int64_t numer;
    int n_pre;  // 0 = kTooLate
  };

  for (const Row row : {Row{kDtPre - 1, 0}, Row{kDtPre, 1}, Row{2 * kDtPre - 1, 1},
                        Row{3 * kDtPre + 7 * kMs, 3}, Row{6 * kDtPre, 6}, Row{11 * kDtPre, 6}}) {
    const std::int64_t now = kT0;
    const std::int64_t t_c = now + kTArm + kFirst + 2 * kH + row.numer;
    SetClock(now);
    const bool ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, now - kH),
                                        PlanFor(r.arm, c, t_c), BallFor(c), r.out, r.rec);
    EXPECT_EQ(r.rec.kind, DecelKind::kFirst);
    if (row.n_pre == 0) {
      EXPECT_FALSE(ok);
      EXPECT_EQ(r.rec.outcome, DecelOutcome::kTooLate) << row.numer;
      continue;
    }
    // The grid point is recorded before the solve, published or not.
    EXPECT_EQ(r.rec.k, -row.n_pre);
    EXPECT_EQ(r.rec.n_nodes, row.n_pre + 7);
    if (row.n_pre == 1) {
      // One pre-catch interval reaches little (the known limit of n_pre 1 –
      // 2, #661): the catch gate may withhold it, and then says so.
      if (!ok) {
        EXPECT_EQ(r.rec.outcome, DecelOutcome::kCatchError) << Why(r.rec);
        std::printf("[ record ] n_pre 1 withheld: catch error %.1f mm\n",
                    1e3 * r.rec.catch_pos_err);
        continue;
      }
    }
    ASSERT_TRUE(ok) << row.numer << ": " << Why(r.rec);
    EXPECT_EQ(r.out.n_pre, row.n_pre);
    EXPECT_EQ(r.out.k0, 0);
    EXPECT_EQ(r.out.t_c_ns, t_c);
    EXPECT_EQ(r.out.t0_ns, t_c - row.n_pre * kDtPre);
    EXPECT_EQ(r.out.dt_pre_ns, kDtPre);
    EXPECT_EQ(r.out.dt_ns, kDt);
    // The stop end is t_c + N_s·Δ_s whatever n_pre is.
    EXPECT_EQ(DecelNodeTimeNs(r.out, r.out.n_nodes), t_c + 7 * kDt);
    EXPECT_TRUE(rtc::catching::ValidateDecelNodes(r.out));
    // Its effective instant is reachable after the solve.
    EXPECT_GT(r.out.t0_ns, now + kTArm + kFirst + 2 * kH - 1);
  }
}

TEST(ApproachPlanner, FirstSegmentStartsAtTheCommandAndReachesTheBall) {
  for (Arm arm : {Arm6(), Arm7()}) {
    Rig r(std::move(arm));
    const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
    ASSERT_NE(s.seq, 0U);
    EXPECT_TRUE(r.rec.cold_start);
    EXPECT_EQ(r.rec.w_delta_scale, 0.0);
    EXPECT_FALSE(r.rec.w_p_fallback);
    EXPECT_FALSE(r.rec.ref_clamped);
    EXPECT_FALSE(r.rec.ref_scaled);
    EXPECT_EQ(r.rec.x0_speed, 0.0);
    // Node 0 IS the command, in DEVICE order.
    for (int m = 0; m < r.arm.model->nv; ++m) {
      EXPECT_NEAR(r.out.q[Dev(r.arm, m)], r.arm.q_nominal[m], 1e-12) << m;
      EXPECT_NEAR(r.out.qd[Dev(r.arm, m)], 0.0, 1e-9) << m;
    }
    // The catch node reaches the ball (the gate) and the record says by how much.
    EXPECT_LE((CatchNodePos(r.arm, r.out) - s.c.p).norm(), 0.02);
    EXPECT_NEAR((CatchNodePos(r.arm, r.out) - s.c.p).norm(), r.rec.catch_pos_err, 1e-9);
    // Node N rests to the core's reference tolerance.
    for (int m = 0; m < r.out.nv; ++m) {
      const auto e = static_cast<std::size_t>(r.out.n_nodes * kMaxDecelNv + m);
      EXPECT_LE(std::fabs(r.out.qd[e]), 1e-4);
      EXPECT_LE(std::fabs(r.out.qdd[e]), 1e-4);
    }
    EXPECT_GE(r.rec.speed_ratio_max, 0.0);
    EXPECT_LE(r.rec.speed_ratio_max, 1.0);
  }
}

TEST(ApproachPlanner, TheFirstSolveNeedsTheArmAtRest) {
  Rig r(Arm6());
  const Catch c = CatchAt(r.arm, Offset(r.arm, 0.02));
  const std::int64_t t_c = kT0 + 800 * kMs;
  SetClock(kT0);
  PlannerRtState rt = RestingRt(r.arm, r.arm.q_nominal, kT0 - kH);
  rt.qd_cmd[3] = 0.051;  // rest_tol 0.05
  EXPECT_FALSE(r.planner.PlanFirst(rt, PlanFor(r.arm, c, t_c), BallFor(c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotAtRest);
  EXPECT_DOUBLE_EQ(r.rec.x0_speed, 0.051);
  rt.qd_cmd[3] = kNan;
  EXPECT_FALSE(r.planner.PlanFirst(rt, PlanFor(r.arm, c, t_c), BallFor(c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotAtRest);
  EXPECT_TRUE(std::isnan(r.rec.x0_speed));
  rt.qd_cmd[3] = 0.049;
  EXPECT_TRUE(r.planner.PlanFirst(rt, PlanFor(r.arm, c, t_c), BallFor(c), r.out, r.rec))
      << Why(r.rec);
}

TEST(ApproachPlanner, TheFirstSolveTakesTheMeasuredPoseOfAnUnseededCommand) {
  // What the controller reports in TRACKING before any plan: cmd_seeded
  // false, q_cmd the measured pose, q̇_cmd zero. The first solve must run from
  // it — requiring a seeded command would withhold every pair. A replan does
  // need the command (the RT follows a plan by then).
  Rig r(Arm6());
  const Catch c = CatchAt(r.arm, Offset(r.arm, 0.02));
  const std::int64_t t_c = kT0 + 800 * kMs;
  SetClock(kT0);
  PlannerRtState rt = RestingRt(r.arm, r.arm.q_nominal, kT0 - kH);
  rt.cmd_seeded = false;
  ASSERT_TRUE(r.planner.PlanFirst(rt, PlanFor(r.arm, c, t_c), BallFor(c), r.out, r.rec))
      << Why(r.rec);
  for (int m = 0; m < r.arm.model->nv; ++m) {
    EXPECT_NEAR(r.out.q[Dev(r.arm, m)], r.arm.q_nominal[m], 1e-12) << m;
  }
  const std::uint32_t seq = r.Publish(kT0);
  PlannerRtState following = FollowingRt(r.arm, r.arm.q_nominal, kT0 - kH, t_c, seq, 0);
  following.cmd_seeded = false;
  EXPECT_FALSE(r.planner.Replan(following, BallFor(c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNoState);
}

TEST(ApproachPlanner, TheReferenceIsClampedToTheBoxAndSlowedToTheVelocityBox) {
  Rig r(Arm6());
  const std::int64_t t_c = kT0 + 800 * kMs;
  // A q_star past a joint's limit: the IK clamps to the limit itself, the
  // reference must not aim outside the core's box [q_min + m, q_max − m].
  Catch c = CatchAt(r.arm, Offset(r.arm, 0.02));
  Eigen::VectorXd q_over = c.q_catch;
  q_over[1] = r.arm.model->upperPositionLimit[1] + 0.2;
  Catch over = c;
  over.q_catch = q_over;
  SetClock(kT0);
  static_cast<void>(r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                        PlanFor(r.arm, over, t_c), BallFor(c), r.out, r.rec));
  EXPECT_TRUE(r.rec.ref_clamped);
  // A reach the velocity box cannot make in the pre-catch time: slowed, and
  // the record says by how much.
  Catch far = c;
  far.q_catch = r.arm.q_nominal;
  far.q_catch[0] += 1.5;  // 1.875·1.5/0.6 s ≈ 4.7 rad/s ≫ 0.9·0.9·2 rad/s
  SetClock(kT0);
  static_cast<void>(r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                        PlanFor(r.arm, far, t_c), BallFor(c), r.out, r.rec));
  EXPECT_TRUE(r.rec.ref_scaled);
  const double allowed = 0.9 * 0.9 * 2.0 * 0.6 / 1.875;
  EXPECT_NEAR(r.rec.ref_scale, allowed / 1.5, 1e-12);
  EXPECT_NEAR(r.rec.ref_shortfall, 1.5 - allowed, 1e-12);
}

TEST(ApproachPlanner, WithoutACovarianceThePositionWeightIsConstant) {
  Rig r(Arm6());
  const Catch c = CatchAt(r.arm, Offset(r.arm, 0.02));
  DecelBallTarget ball = BallFor(c);
  ball.sigma_valid = false;
  SetClock(kT0);
  ASSERT_TRUE(r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                  PlanFor(r.arm, c, kT0 + 800 * kMs), ball, r.out, r.rec))
      << Why(r.rec);
  EXPECT_TRUE(r.rec.w_p_fallback);
  EXPECT_EQ(r.rec.w_delta_scale, 0.0);  // the first solve's, whatever the weight
}

// ── 3. Publish gates ─────────────────────────────────────────────────────────

TEST(ApproachPlanner, EachGateWithholdsOnItsOwn) {
  const Catch c0 = CatchAt(Arm6(), Offset(Arm6(), 0.02));
  const auto first = [&](DecelPlannerParams p, std::int64_t clock_step) {
    Rig r(Arm6(), p);
    SetClock(kT0, clock_step);
    const bool ok =
        r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                            PlanFor(r.arm, c0, kT0 + 800 * kMs), BallFor(c0), r.out, r.rec);
    SetClock(kT0);
    EXPECT_FALSE(ok);
    return r.rec;
  };
  // Budget: the fake clock advances the first budget plus one per read.
  EXPECT_EQ(first(ApproachParams(), kFirst + 1).outcome, DecelOutcome::kBudget);
  DecelPlannerParams p = ApproachParams();
  p.slack_max = -1.0;
  EXPECT_EQ(first(p, 0).outcome, DecelOutcome::kSlack);
  p = ApproachParams();
  p.slack_terminal_max = kNan;  // a NaN threshold fails, not passes
  EXPECT_EQ(first(p, 0).outcome, DecelOutcome::kSlack);
  p = ApproachParams();
  p.catch_pos_err_max = 1e-9;
  DecelRecord rec = first(p, 0);
  EXPECT_EQ(rec.outcome, DecelOutcome::kCatchError);
  EXPECT_TRUE(std::isfinite(rec.catch_pos_err));
  p.catch_pos_err_max = kNan;
  EXPECT_EQ(first(p, 0).outcome, DecelOutcome::kCatchError);
}

TEST(ApproachPlanner, BetweenNodeSpeedFindsTheInteriorExtremum) {
  // One joint, two 0.1 s intervals. q̈ goes +4 → −4 on the first: q̇ peaks
  // mid-interval at q̇_0 + ½·4·0.05 = 1.0 + 0.1.
  Eigen::MatrixXd qd(1, 3);
  Eigen::MatrixXd qdd(1, 3);
  qd << 1.0, 1.0, 0.0;
  qdd << 4.0, -4.0, 0.0;
  const std::array<double, 1> lim_hi{1.2};
  const std::array<double, 1> lim_lo{1.05};
  double ratio = 0.0;
  EXPECT_TRUE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_hi, ratio));
  EXPECT_NEAR(ratio, 1.1 / 1.2, 1e-12);
  // The nodes alone are inside 1.05; the extremum is not.
  EXPECT_FALSE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_lo, ratio));
  EXPECT_NEAR(ratio, 1.1 / 1.05, 1e-12);
  // The spacing matters: read as Δ_s (n_pre 0) the peak is 1.0 + ½·4·0.025.
  EXPECT_TRUE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 0, 0.1, 0.05, lim_lo, ratio));
  EXPECT_NEAR(ratio, 1.05 / 1.05, 1e-12);
  // Same-sign q̈: monotone in between, the nodes decide.
  qdd << 4.0, 4.0, 0.0;
  EXPECT_TRUE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_lo, ratio));
  // A NaN anywhere fails and records +inf.
  qdd(0, 1) = kNan;
  EXPECT_FALSE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_hi, ratio));
  qdd(0, 1) = -4.0;
  qd(0, 2) = kNan;
  EXPECT_FALSE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_hi, ratio));
  EXPECT_TRUE(std::isinf(ratio));
  qd(0, 2) = 0.0;
  const std::array<double, 1> lim_nan{kNan};
  EXPECT_FALSE(rtc::catching::DecelBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_nan, ratio));
}

// ── 4. Replans ───────────────────────────────────────────────────────────────

TEST(ApproachPlanner, ASamePointResolveIsWarmAndKeepsNodeZero) {
  Rig r(Arm7());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const std::int64_t t0 = r.out.t0_ns;
  ASSERT_EQ(r.out.n_pre, 6);
  // The RT holds seq 1 pending; 20 ms later the replan budget still reaches
  // the same grid point (n_pre stays at its cap).
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                               BallFor(s.c), r.out, r.rec))
      << Why(r.rec);
  EXPECT_EQ(r.rec.kind, DecelKind::kSame);
  EXPECT_FALSE(r.rec.cold_start);
  EXPECT_EQ(r.rec.source_seq, s.seq);
  EXPECT_TRUE(r.rec.from_segment);
  EXPECT_EQ(r.out.t0_ns, t0);
  EXPECT_EQ(r.out.n_pre, 6);
  EXPECT_GT(r.rec.w_delta_scale, 0.0);  // σ 0.01 m: tr Σ / σ_ref² = 3e-4/9e-4
  EXPECT_NEAR(r.rec.w_delta_scale, 3.0 * 1e-4 / 9e-4, 1e-12);
  // `replan.same_point: false` makes it up to date instead.
  DecelPlannerParams p = ApproachParams();
  p.replan_same_point = false;
  Rig q(Arm7(), p);
  const Started s2 = StartPlan(q, kT0, 800 * kMs);
  SetClock(now);
  EXPECT_FALSE(q.planner.Replan(FollowingRt(q.arm, q.arm.q_nominal, now - kH, s2.t_c, s2.seq, 0),
                                BallFor(s2.c), q.out, q.rec));
  EXPECT_EQ(q.rec.outcome, DecelOutcome::kUpToDate);
}

TEST(ApproachPlanner, AdvanceIsColdAndStartsOnTheSource) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const DecelPlanSnapshot source = r.out;
  // Late enough that only five pre-catch intervals fit.
  const std::int64_t now = s.t_c - kTArm - kReplan - 2 * kH - 5 * kDtPre - 10 * kMs;
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                               BallFor(s.c), r.out, r.rec))
      << Why(r.rec);
  EXPECT_EQ(r.rec.kind, DecelKind::kAdvance);
  EXPECT_TRUE(r.rec.cold_start);
  EXPECT_EQ(r.rec.k, -5);
  EXPECT_EQ(r.out.n_pre, 5);
  EXPECT_EQ(r.out.t0_ns, s.t_c - 5 * kDtPre);
  // Node 0 is the source at t_eff — continuous where the RT switches.
  std::array<double, kMaxDecelNv> q{};
  std::array<double, kMaxDecelNv> qd{};
  std::array<double, kMaxDecelNv> qdd{};
  ASSERT_TRUE(rtc::catching::NodeTrajectoryFollower::SampleJoints(source, r.out.t0_ns, q, qd, qdd));
  for (int j = 0; j < r.out.nv; ++j) {
    const auto u = static_cast<std::size_t>(j);
    EXPECT_NEAR(r.out.q[u], q[u], 1e-9) << j;
    EXPECT_NEAR(r.out.qd[u], qd[u], 1e-6) << j;
  }
  EXPECT_EQ(DecelNodeTimeNs(r.out, r.out.n_nodes), DecelNodeTimeNs(source, source.n_nodes));
}

TEST(ApproachPlanner, PreCatchHandsOverToTheStopCores) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  std::uint32_t followed = s.seq;
  // earliest = now + T_arm + replan + 2h; place it just inside each grid
  // point: the catch node (k = 0), then k = 1 and k = 2 after t_c.
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  for (const int k : {0, 1, 2}) {
    const std::int64_t now = s.t_c + k * kDt - lag - 1 * kMs;
    SetClock(now);
    ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, followed),
                                 BallFor(s.c), r.out, r.rec))
        << "k " << k << ": " << Why(r.rec);
    EXPECT_EQ(r.rec.kind, DecelKind::kStop);
    EXPECT_EQ(r.rec.k, k);
    EXPECT_TRUE(r.rec.cold_start) << k;  // another core / grid point
    EXPECT_EQ(r.out.n_pre, 0);
    EXPECT_EQ(r.out.k0, k);
    EXPECT_EQ(r.out.t0_ns, s.t_c + k * kDt);
    EXPECT_EQ(DecelNodeTimeNs(r.out, r.out.n_nodes), s.t_c + 7 * kDt);
    EXPECT_TRUE(std::isnan(r.rec.catch_pos_err));  // no catch terms after t_c
    followed = r.Publish(now);
    // At most one solve per stop grid point.
    ASSERT_FALSE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, followed),
                                  BallFor(s.c), r.out, r.rec));
    EXPECT_EQ(r.rec.outcome, DecelOutcome::kUpToDate);
  }
  const std::int64_t now = s.t_c + 3 * kDt - lag - 1 * kMs;
  SetClock(now);
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, followed),
                                BallFor(s.c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kPastReplanWindow);
  EXPECT_EQ(r.rec.k, 3);
}

TEST(ApproachPlanner, TheSourceIsWhatTheRtReports) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const std::int64_t now = kT0 + 20 * kMs;
  const DecelBallTarget ball = BallFor(s.c);
  const Eigen::VectorXd& q = r.arm.q_nominal;
  SetClock(now);
  // Nothing reported: not followed — never "it is due, so it must be".
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, 0, 0), ball, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotFollowed);
  // A seq the ring never held.
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, 99, 0), ball, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotFollowed);
  // Another plan: not followed, and the ring survives it.
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, s.seq, 0, /*plan_id=*/8),
                                ball, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotFollowed);
  EXPECT_EQ(r.planner.SourceSeq(FollowingRt(r.arm, q, now - kH, s.t_c, s.seq, 0), s.t_c), s.seq);
  // Pending wins over active when it starts no later than t_eff; a pending
  // one that starts after it does not.
  ASSERT_TRUE(
      r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, s.seq, 0), ball, r.out, r.rec))
      << Why(r.rec);
  const std::uint32_t seq2 = r.Publish(now);
  const PlannerRtState both = FollowingRt(r.arm, q, now - kH, s.t_c, seq2, s.seq);
  EXPECT_EQ(r.planner.SourceSeq(both, r.out.t0_ns), seq2);
  EXPECT_EQ(r.planner.SourceSeq(both, r.out.t0_ns - 1), s.seq);
  // Active alone.
  EXPECT_EQ(r.planner.SourceSeq(FollowingRt(r.arm, q, now - kH, s.t_c, 0, seq2), r.out.t0_ns),
            seq2);
}

TEST(ApproachPlanner, TheFollowedSegmentSurvivesABurstOfResolves) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const DecelBallTarget ball = BallFor(s.c);
  // The RT keeps following seq 1 while twelve same-point re-solves are
  // published (more than the ring holds).
  for (int i = 0; i < 12; ++i) {
    const std::int64_t now = kT0 + (10 + i) * kMs;
    SetClock(now);
    ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq),
                                 ball, r.out, r.rec))
        << i << ": " << Why(r.rec);
    EXPECT_EQ(r.rec.source_seq, s.seq);
    r.Publish(now);
  }
  EXPECT_EQ(r.planner.SourceSeq(FollowingRt(r.arm, r.arm.q_nominal, kT0, s.t_c, 0, s.seq), s.t_c),
            s.seq);
  // The evicted middle ones are gone.
  EXPECT_EQ(r.planner.SourceSeq(FollowingRt(r.arm, r.arm.q_nominal, kT0, s.t_c, 0, 3), s.t_c), 0U);
}

TEST(ApproachPlanner, ShadowTakesTheNewestSegmentWithoutAReport) {
  DecelPlannerParams p = ApproachParams();
  p.shadow = true;
  Rig r(Arm6(), p);
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, 0),
                               BallFor(s.c), r.out, r.rec))
      << Why(r.rec);
  EXPECT_EQ(r.rec.source_seq, s.seq);
}

TEST(ApproachPlanner, APreCatchGridPointNeedsABall) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                DecelBallTarget{}, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNoBall);
  EXPECT_EQ(r.rec.k, -6);  // the grid point is recorded even when withheld
}

TEST(ApproachPlanner, TheCatchNodeFollowsAMovedBall) {
  Rig r(Arm7());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const Eigen::Vector3d before = CatchNodePos(r.arm, r.out);
  DecelBallTarget moved = BallFor(s.c);
  moved.p_b += Eigen::Vector3d(0.0, 0.012, -0.008);
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                               moved, r.out, r.rec))
      << Why(r.rec);
  const Eigen::Vector3d after = CatchNodePos(r.arm, r.out);
  EXPECT_LT((after - moved.p_b).norm(), (before - moved.p_b).norm());
  EXPECT_GT((after - before).norm(), 0.005);
  EXPECT_NEAR((after - moved.p_b).norm(), r.rec.catch_pos_err, 1e-9);
}

TEST(ApproachPlanner, AStartOnTheVelocityBoxIsProjectedIntoIt) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  // Put the source's catch node a hair past the velocity box on one joint (a
  // published node keeps the box only to the solver's tolerance).
  DecelPlanSnapshot edge = r.out;
  const auto e = static_cast<std::size_t>(edge.n_pre * kMaxDecelNv) + Dev(r.arm, 2);
  edge.qd[e] = 0.9 * 3.0 * (1.0 + 1e-7);
  edge.decel_seq = 50;
  r.planner.NoteApproachPublished(edge);
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  const std::int64_t now = s.t_c - lag - 1 * kMs;  // the catch node (stop k = 0)
  SetClock(now);
  static_cast<void>(
      r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, edge.decel_seq),
                       BallFor(s.c), r.out, r.rec));
  EXPECT_EQ(r.rec.kind, DecelKind::kStop);
  EXPECT_TRUE(r.rec.x0_clamped) << Why(r.rec);
  EXPECT_NE(r.rec.core_reason, DecelMpcReason::kInitialStateOutsideBox);
}

// ── 5. The ball target ───────────────────────────────────────────────────────

rtc::catching::TrajectorySnapshot Line(int n = 10) {
  rtc::catching::TrajectorySnapshot t{};
  t.valid = true;
  t.n = n;
  t.token.activation_generation = 3;
  t.token.generation = 5;
  t.token.snapshot_sequence = 11;
  for (int k = 0; k < n; ++k) {
    auto& s = t.s[static_cast<std::size_t>(k)];
    s.t_ns = kT0 + k * 50 * kMs;
    s.p = {0.5 + 0.05 * k * -4.0, 0.1, 1.0};
    s.v = {-4.0, 0.0, 0.0};
  }
  return t;
}

rtc::catching::CovarianceSnapshot Cov(const rtc::catching::TrajectorySnapshot& t) {
  rtc::catching::CovarianceSnapshot c{};
  c.valid = true;
  c.n = t.n;
  c.token = t.token;
  for (int k = 0; k < t.n; ++k) {
    auto& b = c.c[static_cast<std::size_t>(k)];
    b.fill(0.0);
    const double s2 = 1e-4 * (k + 1);
    b[0] = s2;
    b[7] = 2.0 * s2;
    b[14] = 3.0 * s2;
    b[1] = b[6] = 0.5 * s2;  // a correlation, so the blend is not diagonal
  }
  return c;
}

TEST(ApproachBallTarget, AtASampleItIsThatSampleEvenBesideANan) {
  const auto t = Line();
  auto c = Cov(t);
  c.c[4][0] = kNan;  // the next sample's block
  const DecelBallTarget b = rtc::catching::MakeDecelBallTarget(t, c, true, t.s[3].t_ns, 1e-6);
  ASSERT_TRUE(b.valid);
  ASSERT_TRUE(b.sigma_valid);
  EXPECT_EQ(b.sigma_p(0, 0), c.c[3][0]);
  EXPECT_EQ(b.sigma_p(0, 1), c.c[3][1]);
  EXPECT_EQ(b.sigma_p(2, 2), c.c[3][14]);
  EXPECT_NEAR((b.a_d - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(b.p_b.x(), t.s[3].p[0], 1e-12);
}

TEST(ApproachBallTarget, BetweenSamplesItBlendsOnIntegerNanoseconds) {
  const auto t = Line();
  const auto c = Cov(t);
  const std::int64_t at = t.s[2].t_ns + 10 * kMs;  // α = 0.2
  const DecelBallTarget b = rtc::catching::MakeDecelBallTarget(t, c, true, at, 1e-6);
  ASSERT_TRUE(b.sigma_valid);
  EXPECT_NEAR(b.sigma_p(1, 1), 0.8 * c.c[2][7] + 0.2 * c.c[3][7], 1e-18);
  EXPECT_NEAR(b.sigma_p(1, 0), 0.8 * c.c[2][6] + 0.2 * c.c[3][6], 1e-18);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(b.sigma_p);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
  // The last sample is itself.
  const DecelBallTarget last = rtc::catching::MakeDecelBallTarget(t, c, true, t.s[9].t_ns, 1e-6);
  ASSERT_TRUE(last.sigma_valid);
  EXPECT_EQ(last.sigma_p(0, 0), c.c[9][0]);
}

TEST(ApproachBallTarget, RefusesWhatItCannotTrust) {
  const auto t = Line();
  const auto c = Cov(t);
  const std::int64_t mid = t.s[2].t_ns + 10 * kMs;
  // Another snapshot's covariance: the ball, but no Σ_p.
  DecelBallTarget b = rtc::catching::MakeDecelBallTarget(t, c, false, mid, 1e-6);
  EXPECT_TRUE(b.valid);
  EXPECT_FALSE(b.sigma_valid);
  auto bad = c;
  bad.c[3][7] = kNan;
  EXPECT_FALSE(rtc::catching::MakeDecelBallTarget(t, bad, true, mid, 1e-6).sigma_valid);
  bad = c;
  bad.n = 9;
  EXPECT_FALSE(rtc::catching::MakeDecelBallTarget(t, bad, true, mid, 1e-6).sigma_valid);
  // Outside the horizon: extrapolated, neither.
  b = rtc::catching::MakeDecelBallTarget(t, c, true, t.s[9].t_ns + 1, 1e-6);
  EXPECT_FALSE(b.valid);
  EXPECT_FALSE(b.sigma_valid);
  b = rtc::catching::MakeDecelBallTarget(t, c, true, t.s[0].t_ns - 1, 1e-6);
  EXPECT_FALSE(b.valid);
  // A ball slower than v_eps has no direction of travel.
  EXPECT_FALSE(rtc::catching::MakeDecelBallTarget(t, c, true, mid, 5.0).valid);
  EXPECT_FALSE(rtc::catching::MakeDecelBallTarget(t, c, true, mid, kNan).valid);
}

// ── 6. Allocation (MD-23) ────────────────────────────────────────────────────

struct Counts {
  std::size_t op_new{0};
  std::size_t c_malloc{0};
};

template <typename F>
Counts Gated(F&& f) {
  Counts c;
  rtc::testing::ScopedAllocGate new_gate;
  rtc::testing::ScopedMallocGate malloc_gate;
  f();
  c.op_new = new_gate.count();
  c.c_malloc = malloc_gate.count();
  return c;
}

TEST(ApproachPlanner, PathsBeforeTheSolveAllocateNothing) {
  {
    // Positive control: an allocation inside pinocchio's shared object.
    const Arm arm = Arm6();
    rtc::testing::ScopedMallocGate gate;
    pinocchio::Data probe(*arm.model);
    ASSERT_GT(gate.count(), 0U);
  }
  Rig r(Arm7());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const DecelBallTarget ball = BallFor(s.c);
  const PlanSnapshot plan = PlanFor(r.arm, s.c, s.t_c, 8);
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  bool ok = true;
  PlannerRtState moving = RestingRt(r.arm, r.arm.q_nominal, now - kH);
  moving.qd_cmd[0] = 0.3;
  Counts c = Gated([&] { ok = r.planner.PlanFirst(moving, plan, ball, r.out, r.rec); });
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotAtRest);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "not-at-rest path";
  PlanSnapshot late = plan;
  late.t_c_ns = now + kTArm + kFirst;
  c = Gated([&] {
    ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, now - kH), late, ball, r.out, r.rec);
  });
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kTooLate);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "too-late path";
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, 0), ball, r.out,
                          r.rec);
  });
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNotFollowed);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "not-followed path";
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                          DecelBallTarget{}, r.out, r.rec);
  });
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kNoBall);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "no-ball path";
  // A post-catch grid point after it was published: up to date.
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  const std::int64_t after = s.t_c + kDt - lag - kMs;
  SetClock(after);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, after - kH, s.t_c, 0, s.seq),
                               ball, r.out, r.rec))
      << Why(r.rec);
  const std::uint32_t stop_seq = r.Publish(after);
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, after - kH, s.t_c, 0, stop_seq), ball,
                          r.out, r.rec);
  });
  EXPECT_EQ(r.rec.outcome, DecelOutcome::kUpToDate);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "up-to-date path";
  EXPECT_FALSE(ok);
}

TEST(ApproachPlanner, SolvesAllocateNothingOutsideProxQp) {
  // Through the solve the C mallocs are ProxQP's (#654) and cannot be told
  // apart from ours, so operator new is the gate (MD-23, the same boundary
  // as the stop-segment planner's AllocatesNothingOutsideProxQp).
  Rig r(Arm7());
  const Started s = StartPlan(r, kT0, 800 * kMs);  // warm-up outside the gates
  ASSERT_NE(s.seq, 0U);
  const DecelBallTarget ball = BallFor(s.c);
  bool ok = false;
  SetClock(kT0);
  Counts c = Gated([&] {
    ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                             PlanFor(r.arm, s.c, s.t_c), ball, r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(c.op_new, 0U) << "first solve";
  r.Publish(kT0);
  std::size_t mallocs = c.c_malloc;
  const std::int64_t same = kT0 + 20 * kMs;
  SetClock(same);
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, same - kH, s.t_c, r.seq, 0), ball,
                          r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, DecelKind::kSame);
  EXPECT_EQ(c.op_new, 0U) << "same-point re-solve";
  r.Publish(same);
  const std::int64_t adv = s.t_c - kTArm - kReplan - 2 * kH - 4 * kDtPre - 10 * kMs;
  SetClock(adv);
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, adv - kH, s.t_c, r.seq, 0), ball,
                          r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, DecelKind::kAdvance);
  EXPECT_EQ(c.op_new, 0U) << "grid advance";
  r.Publish(adv);
  const std::int64_t stop = s.t_c - kTArm - kReplan - 2 * kH - kMs;
  SetClock(stop);
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, stop - kH, s.t_c, 0, r.seq), ball,
                          r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, DecelKind::kStop);
  EXPECT_EQ(c.op_new, 0U) << "catch node → stop core";
  mallocs += c.c_malloc;
  RecordProperty("approach_qp_solver_mallocs", std::to_string(mallocs));
  std::printf("[alloc] approach first + stop: %zu C mallocs (ProxQP, #654)\n", mallocs);
}

// ── 7. Records (not judged): what the cores cost at configure, and what the
// warm-up buys (MD-64). Quote these from a run of this suite alone.

double RssMb() {
  std::ifstream f("/proc/self/statm");
  long total = 0;
  long rss = 0;
  f >> total >> rss;
  return static_cast<double>(rss) * 4096.0 / (1024.0 * 1024.0);
}

TEST(ApproachPlannerRecord, ConfigureCostAndTheFirstSolveAfterTheWarmUp) {
  for (const bool seven : {false, true}) {
    const Arm arm = seven ? Arm7() : Arm6();
    const DecelPlannerModel pm = PlannerModelOf(arm);
    const double rss0 = RssMb();
    const auto t0 = std::chrono::steady_clock::now();
    DecelPlanner planner;
    std::string err;
    ASSERT_TRUE(planner.Configure(pm, Consts(), ApproachParams(), &FakeClock, &err)) << err;
    const double configure_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const double rss_mb = RssMb() - rss0;
    // The first trial solve of each catch core after the warm-up: n_pre 6 … 2
    // by shrinking the lead (n_pre 1 reaches too little to be a fair solve).
    const Catch c = CatchAt(arm, Offset(arm, 0.03));
    DecelPlanSnapshot out{};
    DecelRecord rec{};
    double first_max_ms = 0.0;
    for (int n_pre = 6; n_pre >= 2; --n_pre) {
      const std::int64_t t_c = kT0 + kTArm + kFirst + 2 * kH + n_pre * kDtPre + 5 * kMs;
      SetClock(kT0);
      const auto s0 = std::chrono::steady_clock::now();
      const bool ok = planner.PlanFirst(RestingRt(arm, arm.q_nominal, kT0 - kH),
                                        PlanFor(arm, c, t_c), BallFor(c), out, rec);
      const double ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count();
      EXPECT_TRUE(ok) << n_pre << ": " << Why(rec);
      EXPECT_EQ(rec.k, -n_pre);
      first_max_ms = std::max(first_max_ms, ms);
    }
    const double warm_max_ms = static_cast<double>(planner.WarmUpMaxNs()) * 1e-6;
    const double warm_total_ms = static_cast<double>(planner.WarmUpTotalNs()) * 1e-6;
    EXPECT_GT(planner.WarmUpMaxNs(), 0);
    const std::string tag = seven ? "7dof" : "6dof";
    // Integer microseconds / kilobytes: RecordProperty's to_string squashes
    // small doubles.
    RecordProperty("configure_us_" + tag, std::to_string(std::llround(configure_ms * 1e3)));
    RecordProperty("warmup_total_us_" + tag, std::to_string(std::llround(warm_total_ms * 1e3)));
    RecordProperty("warmup_max_us_" + tag, std::to_string(std::llround(warm_max_ms * 1e3)));
    RecordProperty("first_solve_after_warmup_max_us_" + tag,
                   std::to_string(std::llround(first_max_ms * 1e3)));
    RecordProperty("configure_rss_kb_" + tag, std::to_string(std::llround(rss_mb * 1024.0)));
    std::printf(
        "[ record ] %s: 9 cores, Configure %.1f ms (warm-up %.1f ms of it, slowest %.1f ms), RSS "
        "+%.1f MB; first trial solve after it, max over n_pre 6..2: %.1f ms\n",
        tag.c_str(), configure_ms, warm_total_ms, warm_max_ms, rss_mb, first_max_ms);
  }
}

}  // namespace
