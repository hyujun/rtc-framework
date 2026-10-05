// E1-F08 (#661): the MPC segment planner's APPROACH–stop solves (mpc_segment_planner.hpp
// §APPROACH–stop) on a fake clock — PlanFirst, Replan, the ball target, the
// between-node speed check and the allocation boundary. NOT the E1-F07 core
// suite (test_catching_mpc_segment_core_approach.cpp drives MpcSegmentCore alone); the
// cycle that wires these into a wake is test_catching_approach_cycle.cpp.
// Spec on #661 (MD-55 – MD-64):
//   configure      ConfigureBuildsCatchCoresInTheStopCoresBox, ...
//   first solve    FirstSolvePicksTheLargestPreCatchCountThatFits, ...
//   publish gates  EachGateWithholdsOnItsOwn, BetweenNodeSpeed*
//   replans        ASamePointResolveIsWarmAndKeepsNodeZero, AdvanceIsColdAndStartsOnTheSource,
//                  PreCatchHandsOverToTheStopCores, TheSourceIsWhatTheRtReports, ...
//   allocation     PathsBeforeTheSolveAllocateNothing, SolvesAllocateNothingOutsideProxQp
//   configure      AReconfiguredPlannerIsANewOne — Configure is a full reset
//   interface      TheViewEntriesSolveWhatTheValueEntriesSolve (E1-F12 #738)
//
// Fixtures break representation symmetries: the device order is a
// non-identity permutation of the model order, T_arm ≠ 0 (real ≠ lead axis),
// instants are realistic absolute steady ns, and both a 6- and a 7-joint arm
// run.
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/planner_trace_digest.hpp"
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
#include <utility>
#include <vector>

namespace {

using rtc::catching::kMaxSegmentNv;
using rtc::catching::Mode;
using rtc::catching::MpcSegmentBallTarget;
using rtc::catching::MpcSegmentCoreParams;
using rtc::catching::MpcSegmentCoreReason;
using rtc::catching::MpcSegmentPlanner;
using rtc::catching::MpcSegmentPlannerConstants;
using rtc::catching::MpcSegmentPlannerModel;
using rtc::catching::MpcSegmentPlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::SegmentKind;
using rtc::catching::SegmentNodeTimeNs;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;

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

MpcSegmentPlannerModel MpcSegmentPlannerModelOf(const Arm& a) {
  MpcSegmentPlannerModel pm;
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
  }
  return pm;
}

MpcSegmentPlannerConstants Consts() {
  MpcSegmentPlannerConstants c;
  c.eta_v = 0.9;
  c.t_arm_s = static_cast<double>(kTArm) * 1e-9;
  c.control_dt = static_cast<double>(kH) * 1e-9;
  c.v_eps = 1e-6;
  return c;
}

// The shipped approach profile (MD-54): 7 × 0.05 s [1, 1, 2, 3] after the
// catch, up to 6 × 0.1 s before it, k_max 2.
MpcSegmentPlannerParams ApproachParams() {
  MpcSegmentPlannerParams p;
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

MpcSegmentBallTarget BallFor(const Catch& c, double sigma = 0.01) {
  MpcSegmentBallTarget b;
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
  s.segment_pending = pending != 0;
  s.segment_pending_seq = pending;
  s.segment_active = active != 0;
  s.segment_seq = active;
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
  MpcSegmentPlanner planner;
  SegmentSnapshot out{};
  SegmentRecord rec{};
  std::uint32_t seq{0};

  explicit Rig(Arm a, MpcSegmentPlannerParams params = ApproachParams(),
               MpcSegmentPlannerConstants consts = Consts())
      : arm(std::move(a)) {
    std::string err;
    EXPECT_TRUE(planner.Configure(MpcSegmentPlannerModelOf(arm), consts, params, &FakeClock, &err))
        << err;
  }

  // Publish what the last step produced the way the cycle does.
  std::uint32_t Publish(std::int64_t publish_ns) {
    out.segment_seq = ++seq;
    out.publish_ns = publish_ns;
    planner.NotePublished(out);
    return out.segment_seq;
  }
};

std::string Why(const SegmentRecord& r) {
  return std::string(SegmentOutcomeName(r.outcome)) + " / " +
         rtc::catching::MpcSegmentCoreReasonName(r.core_reason);
}

// The catch node's frame position of a published segment (model order).
// The catch frame's linear velocity at the catch node, world-aligned.
Eigen::Vector3d CatchNodeVel(const Arm& a, const SegmentSnapshot& p) {
  Eigen::VectorXd q(a.model->nv);
  Eigen::VectorXd qd(a.model->nv);
  for (int m = 0; m < q.size(); ++m) {
    const auto e = static_cast<std::size_t>(p.n_pre * kMaxSegmentNv) + Dev(a, m);
    q[m] = p.q[e];
    qd[m] = p.qd[e];
  }
  pinocchio::Data data(*a.model);
  pinocchio::forwardKinematics(*a.model, data, q, qd);
  pinocchio::updateFramePlacement(*a.model, data, a.frame);
  return pinocchio::getFrameVelocity(*a.model, data, a.frame, pinocchio::LOCAL_WORLD_ALIGNED)
      .linear();
}

Eigen::Vector3d CatchNodePos(const Arm& a, const SegmentSnapshot& p) {
  Eigen::VectorXd q(a.model->nv);
  for (int m = 0; m < q.size(); ++m) {
    q[m] = p.q[static_cast<std::size_t>(p.n_pre * kMaxSegmentNv) + Dev(a, m)];
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
    ASSERT_TRUE(r.planner.Configured());
    // One stop core per stop grid point a segment may start at (MD-31): the
    // stop ends at t_c + N_s·Δ_s wherever it starts.
    for (int k = 0; k <= 2; ++k) {
      EXPECT_EQ(r.planner.Core(k).NumNodes(), 7 - k) << k;
      EXPECT_EQ(r.planner.Core(k).CatchNode(), 0) << k;
    }
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
        EXPECT_EQ(sp.eta_v, 0.9);  // planner.search.grid.gamma.eta_v, not the core default
      }
    }
  }
}

TEST(ApproachPlanner, WithoutAPreCatchGridItDoesNotConfigure) {
  // MD-70: a plan is published only with a segment that starts before t_c,
  // so there is no planner to build without a pre-catch grid — and an
  // unconfigured one solves nothing.
  const Arm arm = Arm6();
  MpcSegmentPlannerParams p = ApproachParams();
  p.n_pre_max = 0;
  MpcSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(MpcSegmentPlannerModelOf(arm), Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("n_pre_max"), std::string::npos) << err;
  EXPECT_FALSE(planner.Configured());
  const Catch c = CatchAt(arm, Offset(arm, 0.03));
  SetClock(kT0);
  SegmentSnapshot out{};
  SegmentRecord rec{};
  EXPECT_FALSE(planner.PlanFirst(RestingRt(arm, arm.q_nominal, kT0 - kH),
                                 PlanFor(arm, c, kT0 + 800 * kMs), BallFor(c), out, rec));
  EXPECT_EQ(rec.outcome, SegmentOutcome::kOff);
  EXPECT_FALSE(planner.Replan(FollowingRt(arm, arm.q_nominal, kT0 - kH, kT0 + 800 * kMs, 1, 0),
                              BallFor(c), out, rec));
  EXPECT_EQ(rec.outcome, SegmentOutcome::kOff);
}

TEST(ApproachPlanner, ConfigureRefusesWhatItCannotBuildOn) {
  const MpcSegmentPlannerParams p = ApproachParams();
  MpcSegmentPlanner bad;
  MpcSegmentPlannerModel pm = MpcSegmentPlannerModelOf(Arm6());
  std::string err;
  EXPECT_FALSE(bad.Configure(pm, Consts(), p, nullptr, &err));
  EXPECT_NE(err.find("clock"), std::string::npos) << err;
  pm.device_of_model[1] = pm.device_of_model[0];
  EXPECT_FALSE(bad.Configure(pm, Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("permutation"), std::string::npos) << err;
  pm = MpcSegmentPlannerModelOf(Arm6());
  pm.tau_max[3] = 0.0;
  EXPECT_FALSE(bad.Configure(pm, Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("Init"), std::string::npos) << err;
  EXPECT_FALSE(bad.Configured());
}

TEST(ApproachPlanner, ConfigureRefusesAPositionMarginTheTrustRegionCannotHold) {
  MpcSegmentPlannerParams p = ApproachParams();
  p.m_q = 0.1;  // the core's δ_tr
  MpcSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(MpcSegmentPlannerModelOf(Arm6()), Consts(), p, &FakeClock, &err));
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
    EXPECT_EQ(r.rec.kind, SegmentKind::kFirst);
    if (row.n_pre == 0) {
      EXPECT_FALSE(ok);
      EXPECT_EQ(r.rec.outcome, SegmentOutcome::kTooLate) << row.numer;
      continue;
    }
    // The grid point is recorded before the solve, published or not.
    EXPECT_EQ(r.rec.k, -row.n_pre);
    EXPECT_EQ(r.rec.n_nodes, row.n_pre + 7);
    if (row.n_pre == 1) {
      // One pre-catch interval reaches little (the known limit of n_pre 1 –
      // 2, #661): the catch gate may withhold it, and then says so.
      if (!ok) {
        EXPECT_EQ(r.rec.outcome, SegmentOutcome::kCatchError) << Why(r.rec);
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
    EXPECT_EQ(SegmentNodeTimeNs(r.out, r.out.n_nodes), t_c + 7 * kDt);
    EXPECT_TRUE(rtc::catching::ValidateSegmentNodes(r.out));
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
    // ... and how far the hand's velocity there is from the ball's.
    EXPECT_NEAR((s.c.v - CatchNodeVel(r.arm, r.out)).norm(), r.rec.catch_v_rel, 1e-9);
    EXPECT_TRUE(std::isfinite(r.rec.slack_v));
    // Node N rests to the core's reference tolerance.
    for (int m = 0; m < r.out.nv; ++m) {
      const auto e = static_cast<std::size_t>(r.out.n_nodes * kMaxSegmentNv + m);
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
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotAtRest);
  EXPECT_DOUBLE_EQ(r.rec.x0_speed, 0.051);
  rt.qd_cmd[3] = kNan;
  EXPECT_FALSE(r.planner.PlanFirst(rt, PlanFor(r.arm, c, t_c), BallFor(c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotAtRest);
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
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoState);
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
  MpcSegmentBallTarget ball = BallFor(c);
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
  const auto first = [&](MpcSegmentPlannerParams p, std::int64_t clock_step) {
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
  EXPECT_EQ(first(ApproachParams(), kFirst + 1).outcome, SegmentOutcome::kBudget);
  MpcSegmentPlannerParams p = ApproachParams();
  p.slack_max = -1.0;
  EXPECT_EQ(first(p, 0).outcome, SegmentOutcome::kSlack);
  p = ApproachParams();
  p.slack_terminal_max = kNan;  // a NaN threshold fails, not passes
  EXPECT_EQ(first(p, 0).outcome, SegmentOutcome::kSlack);
  p = ApproachParams();
  p.catch_pos_err_max = 1e-9;
  SegmentRecord rec = first(p, 0);
  EXPECT_EQ(rec.outcome, SegmentOutcome::kCatchError);
  EXPECT_TRUE(std::isfinite(rec.catch_pos_err));
  p.catch_pos_err_max = kNan;
  EXPECT_EQ(first(p, 0).outcome, SegmentOutcome::kCatchError);
}

// MPC MD-91: `catch.rho_v` / `catch.v_rel_allow`, from the YAML to the cores.
// The slack row sits at the catch node, so the catch cores get it — one more
// variable and seven more rows each — and the stop cores stay as they were.
TEST(ApproachPlanner, TheVelocitySlackKeysReachTheCatchCores) {
  const char* grid =
      "horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, 2, 3]}, "
      "replan: {k_max: 2}, approach: {n_pre_max: 6, dt_pre_s: 0.1}";
  const auto params_of = [&](const std::string& catch_keys) {
    return rtc::catching::ParsePlannerParams(
               YAML::Load("planner: {segment: {mpc: {" + std::string(grid) + catch_keys + "}}}"))
        .mpc_segment;
  };
  for (const Arm& arm : {Arm6(), Arm7()}) {
    Rig off(arm, params_of(""));
    Rig on(arm, params_of(", catch: {rho_v: 5.0, v_rel_allow: 0.25}"));
    ASSERT_TRUE(off.planner.Configured());
    ASSERT_TRUE(on.planner.Configured());  // every warm-up solved with the row on
    for (int j = 1; j <= 6; ++j) {
      EXPECT_EQ(off.planner.ApproachCoreParams(j).rho_v, 0.0) << j;
      EXPECT_EQ(off.planner.ApproachCoreParams(j).v_rel_allow, 0.0) << j;
      EXPECT_EQ(on.planner.ApproachCoreParams(j).rho_v, 5.0) << j;
      EXPECT_EQ(on.planner.ApproachCoreParams(j).v_rel_allow, 0.25) << j;
      const auto& qp_off = off.planner.ApproachCore(j).MainQp();
      const auto& qp_on = on.planner.ApproachCore(j).MainQp();
      EXPECT_EQ(qp_on.n_vars, qp_off.n_vars + 1) << j;
      EXPECT_EQ(qp_on.n_ineq, qp_off.n_ineq + 7) << j;
    }
    for (int k = 0; k <= 2; ++k) {
      EXPECT_EQ(on.planner.StopCoreParams(k).rho_v, 0.0) << k;
      EXPECT_EQ(on.planner.Core(k).MainQp().n_vars, off.planner.Core(k).MainQp().n_vars) << k;
      EXPECT_EQ(on.planner.Core(k).MainQp().n_ineq, off.planner.Core(k).MainQp().n_ineq) << k;
    }
  }
}

// s_v is recorded and never a publish gate (formulation §1.6): no threshold
// is defined for it. The same catch is published with the row off (s_v 0)
// and with it on against a bound a 5 m/s ball leaves far behind — the record
// then carries the worst axis' excess over the bound, as a fraction of it.
TEST(ApproachPlanner, TheVelocitySlackIsRecordedNotJudged) {
  for (const Arm& arm : {Arm6(), Arm7()}) {
    Rig off(arm);
    const Started s0 = StartPlan(off, kT0, 800 * kMs, 0.04);
    ASSERT_NE(s0.seq, 0U);
    EXPECT_EQ(off.rec.slack_v, 0.0);

    MpcSegmentPlannerParams p = ApproachParams();
    p.rho_v = 0.05;
    p.v_rel_allow = 0.1;
    Rig on(arm, p);
    const Started s1 = StartPlan(on, kT0, 800 * kMs, 0.04);
    ASSERT_NE(s1.seq, 0U) << Why(on.rec);  // published: the slack withheld nothing
    EXPECT_EQ(on.rec.outcome, SegmentOutcome::kReady);
    const double excess =
        (s1.c.v - CatchNodeVel(on.arm, on.out)).cwiseAbs().maxCoeff() / p.v_rel_allow - 1.0;
    std::printf("[ record ] n = %d: s_v %.3f, FK excess %.3f, |v_rel| %.3f m/s\n",
                static_cast<int>(arm.model->nv), on.rec.slack_v, excess, on.rec.catch_v_rel);
    // Far over the bound (the hand cannot move at the ball's 5 m/s) ...
    ASSERT_GT(excess, 1.0);
    // ... and the recorded slack is that excess: the row's linear model
    // against FK at the solution.
    EXPECT_NEAR(on.rec.slack_v, excess, 0.01 * excess);
  }
}

// The core's design values, from the YAML to every core the planner
// builds — the stop cores (k = 0..k_max) and the catch cores (n_pre = 1..6).
// Twelve distinct values; a core left on MpcSegmentCoreParams{} reads its default.
TEST(ApproachPlanner, TheDesignKeysReachEveryCore) {
  const auto params_of = [](const std::string& body) {
    return rtc::catching::ParsePlannerParams(
               YAML::Load("planner: {segment: {mpc: {horizon: {n_nodes: 7, "
                          "dt_s: 0.05, blocks: [1, 1, 2, 3]}, replan: "
                          "{k_max: 2}, approach: {n_pre_max: 6, "
                          "dt_pre_s: 0.1}, " +
                          body + "}}}"))
        .mpc_segment;
  };
  const MpcSegmentCoreParams defaults{};
  for (const Arm& arm : {Arm6(), Arm7()}) {
    const int n = static_cast<int>(arm.model->nv);
    // One entry per arm joint, in DEVICE order: entry d belongs to the model
    // joint whose device index is d (the arms' orders are permutations).
    std::string list;
    for (int d = 0; d < n; ++d) {
      list += (d == 0 ? "" : ", ") + std::to_string(1.0 + 0.5 * d);
    }
    Rig on(arm, params_of("cost: {jerk_weight: [" + list +
                          "], u_scale: 500.0, w_delta: 2.5, rho_tau: 7.0}, "
                          "catch: {axis_theta_max: 1.2}, "
                          "linearization: {delta_tr: 0.2, reference_rest_tol: 2.0e-5, "
                          "ref_speed_fraction: 0.8}, "
                          "solver: {max_iter: 300, max_iter_in: 150, eps_abs: 2.0e-7, "
                          "eps_rel: 1.0e-5}"));
    Rig off(arm, params_of(""));
    ASSERT_TRUE(on.planner.Configured());
    ASSERT_TRUE(off.planner.Configured());
    const auto check = [&](const MpcSegmentCoreParams& mp, const char* what, int i) {
      SCOPED_TRACE(std::string(what) + " " + std::to_string(i));
      ASSERT_EQ(mp.jerk_weight.size(), n);
      for (int m = 0; m < n; ++m) {
        EXPECT_EQ(mp.jerk_weight[m], 1.0 + 0.5 * static_cast<double>(Dev(arm, m))) << m;
      }
      EXPECT_EQ(mp.u_scale, 500.0);
      EXPECT_EQ(mp.w_delta, 2.5);
      EXPECT_EQ(mp.rho_tau, 7.0);
      EXPECT_EQ(mp.axis_theta_max, 1.2);
      EXPECT_EQ(mp.delta_tr, 0.2);
      EXPECT_EQ(mp.reference_rest_tol, 2.0e-5);
      EXPECT_EQ(mp.solver.max_iter, 300);
      EXPECT_EQ(mp.solver.max_iter_in, 150);
      EXPECT_EQ(mp.solver.eps_abs, 2.0e-7);
      EXPECT_EQ(mp.solver.eps_rel, 1.0e-5);
      // Not design values: the core's own setting stays.
      EXPECT_EQ(mp.solver.update_preconditioner, defaults.solver.update_preconditioner);
      EXPECT_EQ(mp.solver.dense_backend, defaults.solver.dense_backend);
      // `cost.w_perp` is a key of its own now (#698); this profile does not
      // set it, so the cores keep the default (its tests: section 4b).
      EXPECT_EQ(mp.w_perp, defaults.w_perp);
    };
    for (int j = 1; j <= 6; ++j) {
      check(on.planner.ApproachCoreParams(j), "catch core", j);
    }
    for (int k = 0; k <= 2; ++k) {
      check(on.planner.StopCoreParams(k), "stop core", k);
    }
    // The shipped values are the defaults: a planner built with the keys at
    // them has every new field equal to MpcSegmentCoreParams{} (what the planner
    // overwrites — grid, eta, m_q, catch terms — is not compared).
    const auto same = [&](const MpcSegmentCoreParams& mp, const char* what, int i) {
      SCOPED_TRACE(std::string(what) + " " + std::to_string(i));
      EXPECT_EQ(mp.u_scale, defaults.u_scale);
      EXPECT_EQ(mp.w_delta, defaults.w_delta);
      EXPECT_EQ(mp.rho_tau, defaults.rho_tau);
      EXPECT_EQ(mp.axis_theta_max, defaults.axis_theta_max);
      EXPECT_EQ(mp.delta_tr, defaults.delta_tr);
      EXPECT_EQ(mp.reference_rest_tol, defaults.reference_rest_tol);
      EXPECT_EQ(mp.solver.max_iter, defaults.solver.max_iter);
      EXPECT_EQ(mp.solver.max_iter_in, defaults.solver.max_iter_in);
      EXPECT_EQ(mp.solver.eps_abs, defaults.solver.eps_abs);
      EXPECT_EQ(mp.solver.eps_rel, defaults.solver.eps_rel);
      EXPECT_EQ(mp.jerk_weight.size(), 0);
    };
    for (int j = 1; j <= 6; ++j) {
      same(off.planner.ApproachCoreParams(j), "catch core", j);
    }
    for (int k = 0; k <= 2; ++k) {
      same(off.planner.StopCoreParams(k), "stop core", k);
    }
  }
}

// The list is one entry per ARM joint: the parser has no joint count, so the
// planner's configure refuses a wrong length — by key.
TEST(ApproachPlanner, TheJerkWeightListMustMatchTheArm) {
  const Arm arm = Arm6();
  MpcSegmentPlannerParams p = ApproachParams();
  p.jerk_weight = {1.0, 1.0, 1.0, 1.0, 1.0};  // 5 for 6 joints
  MpcSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(MpcSegmentPlannerModelOf(arm), Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("planner.segment.mpc.cost.jerk_weight"), std::string::npos) << err;
  p.jerk_weight = std::vector<double>(7, 1.0);
  EXPECT_FALSE(planner.Configure(MpcSegmentPlannerModelOf(arm), Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("planner.segment.mpc.cost.jerk_weight"), std::string::npos) << err;
  p.jerk_weight = std::vector<double>(6, 1.0);
  EXPECT_TRUE(planner.Configure(MpcSegmentPlannerModelOf(arm), Consts(), p, &FakeClock, &err))
      << err;
}

// m_q against the trust region used to be compared with a default-constructed
// core's: a profile's own delta_tr must decide.
TEST(ApproachPlanner, TheMarginIsComparedWithTheProfilesTrustRegion) {
  const Arm arm = Arm6();
  MpcSegmentPlannerParams p = ApproachParams();
  std::string err;
  MpcSegmentPlanner planner;
  // m_q 0.15 is above the default trust region (0.1) and below this one.
  p.m_q = 0.15;
  p.delta_tr = 0.3;
  EXPECT_TRUE(planner.Configure(MpcSegmentPlannerModelOf(arm), Consts(), p, &FakeClock, &err))
      << err;
  // m_q 0.05 is below the default 0.1 and not below this one.
  p.m_q = 0.05;
  p.delta_tr = 0.04;
  EXPECT_FALSE(planner.Configure(MpcSegmentPlannerModelOf(arm), Consts(), p, &FakeClock, &err));
  EXPECT_NE(err.find("planner.segment.mpc.linearization.delta_tr"), std::string::npos) << err;
  EXPECT_NE(err.find("planner.segment.mpc.m_q"), std::string::npos) << err;
  EXPECT_FALSE(planner.Configured());
}

// `linearization.ref_speed_fraction`: the planner's own number, read at the
// first solve's reference. The shipped 0.9 gives the figure the existing
// clamp test pins; another value moves it.
TEST(ApproachPlanner, TheReferenceSpeedFractionIsTheProfilesOwn) {
  for (const double fraction : {0.9, 0.5}) {
    MpcSegmentPlannerParams p = ApproachParams();
    p.ref_speed_fraction = fraction;
    Rig r(Arm6(), p);
    const Catch c = CatchAt(r.arm, Offset(r.arm, 0.02));
    Catch far = c;
    far.q_catch = r.arm.q_nominal;
    far.q_catch[0] += 1.5;
    SetClock(kT0);
    static_cast<void>(r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                          PlanFor(r.arm, far, kT0 + 800 * kMs), BallFor(c), r.out,
                                          r.rec));
    ASSERT_TRUE(r.rec.ref_scaled) << fraction;
    const double allowed = fraction * 0.9 * 2.0 * 0.6 / 1.875;
    EXPECT_NEAR(r.rec.ref_scale, allowed / 1.5, 1e-12) << fraction;
  }
}

// `linearization.reference_rest_tol` is also the planner's own judgement: a
// published segment is the next solve's reference, so its node N must be at
// rest to the core's tolerance (Judge reads `rest_tol_ref_`), not the
// payload's looser one. The solve's terminal rest is ~1e-12 — below any
// tolerance a profile could set while the solver still converges — so the
// consumer's number is read through its accessor: the profile's value, the
// same the cores were built with, not a default-constructed core's.
TEST(ApproachPlanner, TheReferenceRestToleranceIsTheProfilesInJudge) {
  for (const double tol : {1.0e-4, 3.0e-5, 5.0e-3}) {
    MpcSegmentPlannerParams p = ApproachParams();
    p.reference_rest_tol = tol;
    Rig r(Arm6(), p);
    ASSERT_TRUE(r.planner.Configured());
    EXPECT_EQ(r.planner.ReferenceRestTol(), tol);
    EXPECT_EQ(r.planner.StopCoreParams(0).reference_rest_tol, tol);
    EXPECT_EQ(r.planner.ApproachCoreParams(1).reference_rest_tol, tol);
  }
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
  EXPECT_TRUE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_hi, ratio));
  EXPECT_NEAR(ratio, 1.1 / 1.2, 1e-12);
  // The nodes alone are inside 1.05; the extremum is not.
  EXPECT_FALSE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_lo, ratio));
  EXPECT_NEAR(ratio, 1.1 / 1.05, 1e-12);
  // The spacing matters: read as Δ_s (n_pre 0) the peak is 1.0 + ½·4·0.025.
  EXPECT_TRUE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 0, 0.1, 0.05, lim_lo, ratio));
  EXPECT_NEAR(ratio, 1.05 / 1.05, 1e-12);
  // Same-sign q̈: monotone in between, the nodes decide.
  qdd << 4.0, 4.0, 0.0;
  EXPECT_TRUE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_lo, ratio));
  // A NaN anywhere fails and records +inf.
  qdd(0, 1) = kNan;
  EXPECT_FALSE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_hi, ratio));
  qdd(0, 1) = -4.0;
  qd(0, 2) = kNan;
  EXPECT_FALSE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_hi, ratio));
  EXPECT_TRUE(std::isinf(ratio));
  qd(0, 2) = 0.0;
  const std::array<double, 1> lim_nan{kNan};
  EXPECT_FALSE(rtc::catching::MpcSegmentBetweenNodeSpeedOk(qd, qdd, 2, 0.1, 0.05, lim_nan, ratio));
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
  EXPECT_EQ(r.rec.kind, SegmentKind::kSame);
  EXPECT_FALSE(r.rec.cold_start);
  EXPECT_EQ(r.rec.source_seq, s.seq);
  EXPECT_TRUE(r.rec.from_segment);
  EXPECT_EQ(r.out.t0_ns, t0);
  EXPECT_EQ(r.out.n_pre, 6);
  EXPECT_GT(r.rec.w_delta_scale, 0.0);  // σ 0.01 m: tr Σ / σ_ref² = 3e-4/9e-4
  EXPECT_NEAR(r.rec.w_delta_scale, 3.0 * 1e-4 / 9e-4, 1e-12);
  // `replan.same_point: false` makes it up to date instead.
  MpcSegmentPlannerParams p = ApproachParams();
  p.replan_same_point = false;
  Rig q(Arm7(), p);
  const Started s2 = StartPlan(q, kT0, 800 * kMs);
  SetClock(now);
  EXPECT_FALSE(q.planner.Replan(FollowingRt(q.arm, q.arm.q_nominal, now - kH, s2.t_c, s2.seq, 0),
                                BallFor(s2.c), q.out, q.rec));
  EXPECT_EQ(q.rec.outcome, SegmentOutcome::kUpToDate);
}

TEST(ApproachPlanner, AdvanceIsColdAndStartsOnTheSource) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const SegmentSnapshot source = r.out;
  // Late enough that only five pre-catch intervals fit.
  const std::int64_t now = s.t_c - kTArm - kReplan - 2 * kH - 5 * kDtPre - 10 * kMs;
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                               BallFor(s.c), r.out, r.rec))
      << Why(r.rec);
  EXPECT_EQ(r.rec.kind, SegmentKind::kAdvance);
  EXPECT_TRUE(r.rec.cold_start);
  EXPECT_EQ(r.rec.k, -5);
  EXPECT_EQ(r.out.n_pre, 5);
  EXPECT_EQ(r.out.t0_ns, s.t_c - 5 * kDtPre);
  // Node 0 is the source at t_eff — continuous where the RT switches.
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(rtc::catching::NodeTrajectoryFollower::SampleJoints(source, r.out.t0_ns, q, qd, qdd));
  for (int j = 0; j < r.out.nv; ++j) {
    const auto u = static_cast<std::size_t>(j);
    EXPECT_NEAR(r.out.q[u], q[u], 1e-9) << j;
    EXPECT_NEAR(r.out.qd[u], qd[u], 1e-6) << j;
  }
  EXPECT_EQ(SegmentNodeTimeNs(r.out, r.out.n_nodes), SegmentNodeTimeNs(source, source.n_nodes));
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
    EXPECT_EQ(r.rec.kind, SegmentKind::kStop);
    EXPECT_EQ(r.rec.k, k);
    EXPECT_TRUE(r.rec.cold_start) << k;  // another core / grid point
    EXPECT_EQ(r.out.n_pre, 0);
    EXPECT_EQ(r.out.k0, k);
    EXPECT_EQ(r.out.t0_ns, s.t_c + k * kDt);
    EXPECT_EQ(SegmentNodeTimeNs(r.out, r.out.n_nodes), s.t_c + 7 * kDt);
    EXPECT_TRUE(std::isnan(r.rec.catch_pos_err));  // no catch terms after t_c
    EXPECT_TRUE(std::isnan(r.rec.catch_v_rel));
    EXPECT_TRUE(std::isnan(r.rec.slack_v));
    followed = r.Publish(now);
    // At most one solve per stop grid point.
    ASSERT_FALSE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, followed),
                                  BallFor(s.c), r.out, r.rec));
    EXPECT_EQ(r.rec.outcome, SegmentOutcome::kUpToDate);
  }
  const std::int64_t now = s.t_c + 3 * kDt - lag - 1 * kMs;
  SetClock(now);
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, followed),
                                BallFor(s.c), r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kPastReplanWindow);
  EXPECT_EQ(r.rec.k, 3);
}

TEST(ApproachPlanner, TheSourceIsWhatTheRtReports) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const std::int64_t now = kT0 + 20 * kMs;
  const MpcSegmentBallTarget ball = BallFor(s.c);
  const Eigen::VectorXd& q = r.arm.q_nominal;
  SetClock(now);
  // Nothing reported: not followed — never "it is due, so it must be".
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, 0, 0), ball, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotFollowed);
  // A seq the ring never held.
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, 99, 0), ball, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotFollowed);
  // Another plan: not followed, and the ring survives it.
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, q, now - kH, s.t_c, s.seq, 0, /*plan_id=*/8),
                                ball, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotFollowed);
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
  const MpcSegmentBallTarget ball = BallFor(s.c);
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

TEST(ApproachPlanner, APreCatchGridPointNeedsABall) {
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  EXPECT_FALSE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                MpcSegmentBallTarget{}, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoBall);
  EXPECT_EQ(r.rec.k, -6);  // the grid point is recorded even when withheld
}

TEST(ApproachPlanner, TheCatchNodeFollowsAMovedBall) {
  Rig r(Arm7());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const Eigen::Vector3d before = CatchNodePos(r.arm, r.out);
  MpcSegmentBallTarget moved = BallFor(s.c);
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
  SegmentSnapshot edge = r.out;
  const auto e = static_cast<std::size_t>(edge.n_pre * kMaxSegmentNv) + Dev(r.arm, 2);
  edge.qd[e] = 0.9 * 3.0 * (1.0 + 1e-7);
  edge.segment_seq = 50;
  r.planner.NotePublished(edge);
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  const std::int64_t now = s.t_c - lag - 1 * kMs;  // the catch node (stop k = 0)
  SetClock(now);
  static_cast<void>(
      r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, edge.segment_seq),
                       BallFor(s.c), r.out, r.rec));
  EXPECT_EQ(r.rec.kind, SegmentKind::kStop);
  EXPECT_TRUE(r.rec.x0_clamped) << Why(r.rec);
  EXPECT_NE(r.rec.core_reason, MpcSegmentCoreReason::kInitialStateOutsideBox);
}

TEST(ApproachPlanner, AStartInsideThePositionMarginIsProjectedIntoTheBox) {
  // A wait pose within m_q of a limit: the core refuses a start outside its
  // box, and a refused first segment withholds the plan on every wake.
  Rig r(Arm6());
  const int j = 5;
  const double q_hi = r.arm.model->upperPositionLimit[j] - r.planner.Params().m_q;
  Eigen::VectorXd q_start = r.arm.q_nominal;
  q_start[j] = q_hi + 0.04;
  Eigen::VectorXd q_catch = Offset(r.arm, 0.03);
  q_catch[j] = q_hi - 0.02;
  const Catch c = CatchAt(r.arm, q_catch);
  const std::int64_t t_c = kT0 + kTArm + 800 * kMs;
  SetClock(kT0);
  ASSERT_TRUE(r.planner.PlanFirst(RestingRt(r.arm, q_start, kT0 - kH), PlanFor(r.arm, c, t_c),
                                  BallFor(c), r.out, r.rec))
      << Why(r.rec);
  EXPECT_TRUE(r.rec.x0_clamped);
  EXPECT_TRUE(r.out.x0_clamped);
  EXPECT_NEAR(r.out.q[Dev(r.arm, j)], q_hi, 1e-9);
  // The other joints start where the RT reported them.
  EXPECT_NEAR(r.out.q[Dev(r.arm, 0)], q_start[0], 1e-9);
  // A start inside the box is left alone.
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  EXPECT_FALSE(r.rec.x0_clamped);
  EXPECT_FALSE(r.out.x0_clamped);
}

TEST(ApproachPlanner, ANarrowJointUsesTheCoresBox) {
  // A joint whose range is under 2·m_q: the core's margin is half the range
  // (its box is the midpoint), and the planner must clamp into THAT box — the
  // limits ± m_q would be an inverted interval.
  Arm arm = Arm6();
  const int j = 5;
  MpcSegmentPlannerModel pm = MpcSegmentPlannerModelOf(arm);
  pm.q_min[static_cast<std::size_t>(j)] = arm.q_nominal[j] - 0.02;
  pm.q_max[static_cast<std::size_t>(j)] = arm.q_nominal[j] + 0.02;
  MpcSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(pm, Consts(), ApproachParams(), &FakeClock, &err)) << err;
  EXPECT_NEAR(planner.ApproachCore(1).PositionLow()[j], arm.q_nominal[j], 1e-12);
  EXPECT_NEAR(planner.ApproachCore(1).PositionHigh()[j], arm.q_nominal[j], 1e-12);
  // The catch asks that joint for −0.03 rad; the reference is held at the box.
  const Catch c = CatchAt(arm, Offset(arm, 0.03));
  const std::int64_t t_c = kT0 + kTArm + 800 * kMs;
  SegmentSnapshot out{};
  SegmentRecord rec{};
  SetClock(kT0);
  ASSERT_TRUE(planner.PlanFirst(RestingRt(arm, arm.q_nominal, kT0 - kH), PlanFor(arm, c, t_c),
                                BallFor(c), out, rec))
      << Why(rec);
  EXPECT_TRUE(rec.ref_clamped);
  EXPECT_FALSE(rec.x0_clamped);
  for (int k = 0; k <= out.n_nodes; ++k) {
    EXPECT_NEAR(out.q[static_cast<std::size_t>(k * kMaxSegmentNv) + Dev(arm, j)], arm.q_nominal[j],
                1e-6)
        << k;
  }
}

TEST(ApproachPlanner, ARefusedSolveMakesTheNextOneCold) {
  // An advance the core refuses BEFORE its QP leaves that core's solver with
  // another problem's iterates (the warm-up's, an earlier trial's). The next
  // solve of the same point must not be taken for a warm re-solve.
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  const std::int64_t now = s.t_c - kTArm - kReplan - 2 * kH - 5 * kDtPre - 10 * kMs;
  const PlannerRtState rt = FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0);
  MpcSegmentBallTarget bad = BallFor(s.c);
  bad.a_d *= 2.0;  // not a unit axis: refused by the core's input check
  SetClock(now);
  ASSERT_FALSE(r.planner.Replan(rt, bad, r.out, r.rec));
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kSolveFailed);
  EXPECT_EQ(r.rec.core_reason, MpcSegmentCoreReason::kDirectionNotUnit);
  EXPECT_EQ(r.rec.k, -5);
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(rt, BallFor(s.c), r.out, r.rec)) << Why(r.rec);
  EXPECT_EQ(r.rec.k, -5);
  EXPECT_TRUE(r.rec.cold_start);
  // That one did reach the QP: the same point again is a warm re-solve.
  const std::uint32_t seq2 = r.Publish(now);
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, seq2, 0),
                               BallFor(s.c), r.out, r.rec))
      << Why(r.rec);
  EXPECT_EQ(r.rec.kind, SegmentKind::kSame);
  EXPECT_FALSE(r.rec.cold_start);
}

TEST(ApproachPlanner, ASegmentCarriesThePlansTrackNotTheRtsLatest) {
  // After the freeze the RT keeps the committed track while it reports the
  // one it consumed last. The plan's segments name the plan's track.
  Rig r(Arm6());
  const Started s = StartPlan(r, kT0, 800 * kMs);
  ASSERT_NE(s.seq, 0U);
  EXPECT_EQ(r.out.token.generation, 5U);  // the plan's token (PlanFor)
  const std::int64_t now = s.t_c - kTArm - kReplan - 2 * kH - 1 * kMs;  // stop k = 0
  PlannerRtState rt = FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq);
  rt.track_generation = 6;
  std::uint64_t track = 0;
  ASSERT_TRUE(r.planner.FollowedTrack(rt, track));
  EXPECT_EQ(track, 5U);
  SetClock(now);
  ASSERT_TRUE(r.planner.Replan(rt, MpcSegmentBallTarget{}, r.out, r.rec)) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, SegmentKind::kStop);
  EXPECT_EQ(r.out.token.generation, 5U);
  // Another plan, or none: no track to name.
  PlannerRtState other = rt;
  other.plan_id = 8;
  EXPECT_FALSE(r.planner.FollowedTrack(other, track));
  other = rt;
  other.plan_active = false;
  EXPECT_FALSE(r.planner.FollowedTrack(other, track));
}

// ── 4b. The stop-path line (`cost.w_perp`, #698) ─────────────────────────────
// With w_⊥ > 0 every solve runs on a line the planner built: the ball's on a
// catch core, the followed segment's on a stop core, a synthetic one in the
// warm-ups — and a solve whose line cannot be built is withheld. With
// w_⊥ = 0 none of that exists: each test's off rig solves what it solved.

constexpr double kPerp = 2000.0;  // [1/m²]

MpcSegmentPlannerParams PerpParams(double w_perp) {
  MpcSegmentPlannerParams p = ApproachParams();
  p.w_perp = w_perp;
  return p;
}

// A ball the plan's own does not equal: 10 mm aside, its travel turned by
// 0.05 rad — so a line built from the wrong ball shows in both members.
// `step` scales both (a sequence of predictions, each with a line of its own).
MpcSegmentBallTarget MovedBall(const Catch& c, double step = 1.0) {
  MpcSegmentBallTarget b = BallFor(c);
  b.p_b += step * Eigen::Vector3d(0.006, -0.008, 0.0);
  const Eigen::Vector3d side = c.v.unitOrthogonal();
  const double a = 0.05 * step;
  b.v_b = c.v.norm() * (std::cos(a) * c.v.normalized() + std::sin(a) * side);
  b.a_d = -b.v_b.normalized();
  return b;
}

// Σ over the stop nodes (the catch node to node N) of the catch frame's
// squared distance from the line through `p` along the unit `d` [m²].
double StopPathDeviation(const Arm& a, const SegmentSnapshot& s, const Eigen::Vector3d& p,
                         const Eigen::Vector3d& d) {
  const Eigen::Matrix3d perp = Eigen::Matrix3d::Identity() - d * d.transpose();
  double sum = 0.0;
  Eigen::VectorXd q(a.model->nv);
  for (int k = s.n_pre; k <= s.n_nodes; ++k) {
    for (int m = 0; m < q.size(); ++m) {
      q[m] = s.q[static_cast<std::size_t>(k * kMaxSegmentNv) + Dev(a, m)];
    }
    sum += (perp * (FkPos(a, q) - p)).squaredNorm();
  }
  return sum;
}

// A withheld record: no QP ran for it.
void ExpectNotSolved(const SegmentRecord& rec) {
  EXPECT_EQ(rec.qp_status, -1);
  EXPECT_EQ(rec.iterations, 0);
  EXPECT_EQ(rec.solve_ns, 0);
  EXPECT_EQ(rec.core_reason, MpcSegmentCoreReason::kNone);
}

// YAML → planner → BOTH core kinds, and the configure warm-ups: each core is
// solved once on the synthetic catch's line (through the catch frame at the
// mid pose, along the synthetic ball's travel), never on the input's default.
TEST(ApproachPlanner, TheStopPathWeightReachesEveryCoreAndTheWarmUpsSolveOnOneLine) {
  const auto params_of = [](const std::string& body) {
    return rtc::catching::ParsePlannerParams(
               YAML::Load("planner: {segment: {mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: "
                          "[1, 1, 2, 3]}, replan: {k_max: 2}, approach: {n_pre_max: 6, "
                          "dt_pre_s: 0.1}" +
                          body + "}}}"))
        .mpc_segment;
  };
  for (const Arm& arm : {Arm6(), Arm7()}) {
    Rig on(arm, params_of(", cost: {w_perp: 40.0}"));
    Rig off(arm, params_of(""));
    ASSERT_TRUE(on.planner.Configured());  // every warm-up solved with the term on
    ASSERT_TRUE(off.planner.Configured());
    Eigen::VectorXd q_mid(arm.model->nv);
    for (int m = 0; m < q_mid.size(); ++m) {
      q_mid[m] = 0.5 * (arm.model->lowerPositionLimit[m] + arm.model->upperPositionLimit[m]);
    }
    pinocchio::Data data(*arm.model);
    pinocchio::forwardKinematics(*arm.model, data, q_mid);
    pinocchio::updateFramePlacement(*arm.model, data, arm.frame);
    const Eigen::Vector3d p = data.oMf[arm.frame].translation();
    const Eigen::Vector3d d = -data.oMf[arm.frame].rotation().col(2);
    // The fixture must tell the built line from the input's default (origin,
    // x). p is far from the origin on both arms; the 7-joint arm's mid-pose
    // axis happens to lie within 1e-5 of x — still far outside the 1e-12 the
    // comparison below allows — and the 6-joint arm's is nowhere near it.
    ASSERT_GT(p.norm(), 0.05);
    ASSERT_GT((d - Eigen::Vector3d::UnitX()).norm(), arm.model->nv == 6 ? 0.05 : 1e-9);
    const auto check = [&](const rtc::catching::MpcSegmentCoreInput& in_on,
                           const rtc::catching::MpcSegmentCoreInput& in_off,
                           const MpcSegmentCoreParams& mp_on, const MpcSegmentCoreParams& mp_off,
                           const char* what, int i) {
      SCOPED_TRACE(std::string(what) + " " + std::to_string(i));
      EXPECT_EQ(mp_on.w_perp, 40.0);
      EXPECT_EQ(mp_off.w_perp, 0.0);
      EXPECT_LT((in_on.p_c - p).norm(), 1e-12);
      EXPECT_LT((in_on.d_hat - d).norm(), 1e-12);
      EXPECT_NEAR(in_on.d_hat.norm(), 1.0, 1e-12);
      // Off, nothing is written: the input keeps its default.
      EXPECT_EQ(in_off.p_c, Eigen::Vector3d::Zero());
      EXPECT_EQ(in_off.d_hat, Eigen::Vector3d::UnitX());
    };
    for (int j = 1; j <= 6; ++j) {
      check(on.planner.ApproachCoreInput(j), off.planner.ApproachCoreInput(j),
            on.planner.ApproachCoreParams(j), off.planner.ApproachCoreParams(j), "catch core", j);
      // The catch warm-up's line is its own synthetic ball's.
      EXPECT_EQ(on.planner.ApproachCoreInput(j).p_c, on.planner.ApproachCoreInput(j).p_b) << j;
    }
    for (int k = 0; k <= 2; ++k) {
      check(on.planner.StopCoreInput(k), off.planner.StopCoreInput(k), on.planner.StopCoreParams(k),
            off.planner.StopCoreParams(k), "stop core", k);
      // One line for both warm-ups.
      EXPECT_EQ(on.planner.StopCoreInput(k).p_c, on.planner.ApproachCoreInput(1).p_c) << k;
      EXPECT_EQ(on.planner.StopCoreInput(k).d_hat, on.planner.ApproachCoreInput(1).d_hat) << k;
    }
  }
}

// PlanFirst: the line is the PLAN's catch point along the plan's ball velocity
// — the vectors the solve hands the core as p_b and v_b — not the ball
// target's (which only carries Σ_p there), and not the warm-up's.
TEST(ApproachPlanner, TheFirstSolveStopsOnTheBallsLine) {
  for (const Arm& arm : {Arm6(), Arm7()}) {
    Rig r(arm, PerpParams(kPerp));
    const Catch c = CatchAt(r.arm, Offset(r.arm, 0.04));
    const std::int64_t t_c = kT0 + kTArm + 800 * kMs;
    SetClock(kT0);
    ASSERT_TRUE(r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                    PlanFor(r.arm, c, t_c), MovedBall(c), r.out, r.rec))
        << Why(r.rec);
    ASSERT_EQ(r.out.n_pre, 6);
    const auto& in = r.planner.ApproachCoreInput(6);
    EXPECT_EQ(in.p_c, c.p);
    EXPECT_LT((in.d_hat - c.v.normalized()).norm(), 1e-12);
    EXPECT_NEAR(in.d_hat.norm(), 1.0, 1e-12);
    // The line and the catch terms are built from the same two vectors.
    EXPECT_EQ(in.p_c, in.p_b);
    EXPECT_LT((in.d_hat - in.v_b.normalized()).norm(), 1e-12);
    EXPECT_GT((in.p_c - MovedBall(c).p_b).norm(), 0.005);
  }
}

// A pre-catch replan: the line is THIS replan's ball — the newer prediction
// moves it — on the core that solves the grid point.
TEST(ApproachPlanner, APreCatchReplanTakesTheLineOfItsOwnBall) {
  for (const Arm& arm : {Arm6(), Arm7()}) {
    Rig r(arm, PerpParams(kPerp));
    const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
    ASSERT_NE(s.seq, 0U);
    const MpcSegmentBallTarget moved = MovedBall(s.c);
    // The same grid point (n_pre 6), then a later one (n_pre 4, another core).
    const std::int64_t same = kT0 + 20 * kMs;
    const std::int64_t adv = s.t_c - kTArm - kReplan - 2 * kH - 4 * kDtPre - 10 * kMs;
    for (const auto& [now, n_pre] : {std::pair{same, 6}, std::pair{adv, 4}}) {
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                   moved, r.out, r.rec))
          << n_pre << ": " << Why(r.rec);
      ASSERT_EQ(r.out.n_pre, n_pre);
      const auto& in = r.planner.ApproachCoreInput(n_pre);
      EXPECT_EQ(in.p_c, moved.p_b) << n_pre;
      EXPECT_LT((in.d_hat - moved.v_b.normalized()).norm(), 1e-12) << n_pre;
      EXPECT_GT((in.d_hat - s.c.v.normalized()).norm(), 0.04) << n_pre;  // not the plan's
    }
  }
}

// What the weight buys: on the same catch the hand leaves the ball's line
// less over the stop nodes with the term on than with it off — on the first
// segment (a catch core) and on the stop core that takes over at the catch.
TEST(ApproachPlanner, TheStopPathWeightKeepsTheStopNearerTheLine) {
  for (const Arm& arm : {Arm6(), Arm7()}) {
    double first[2] = {0.0, 0.0};
    double stop[2] = {0.0, 0.0};
    int i = 0;
    for (const double w : {0.0, kPerp}) {
      Rig r(arm, PerpParams(w));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U) << w << ": " << Why(r.rec);
      const Eigen::Vector3d d = s.c.v.normalized();
      first[i] = StopPathDeviation(r.arm, r.out, s.c.p, d);
      const std::int64_t now = s.t_c - kTArm - kReplan - 2 * kH - 1 * kMs;  // stop k = 0
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq),
                                   MpcSegmentBallTarget{}, r.out, r.rec))
          << w << ": " << Why(r.rec);
      ASSERT_EQ(r.rec.kind, SegmentKind::kStop);
      stop[i] = StopPathDeviation(r.arm, r.out, s.c.p, d);
      ++i;
    }
    std::printf(
        "[ record ] n = %d: off-line distance over the stop nodes, rms mm — first segment "
        "%.2f -> %.2f, stop core %.2f -> %.2f (w_perp 0 -> %.0f)\n",
        static_cast<int>(arm.model->nv), 1e3 * std::sqrt(first[0] / 8.0),
        1e3 * std::sqrt(first[1] / 8.0), 1e3 * std::sqrt(stop[0] / 8.0),
        1e3 * std::sqrt(stop[1] / 8.0), kPerp);
    EXPECT_LT(first[1], first[0]);
    EXPECT_LT(stop[1], stop[0]);
  }
}

// A stop core has no ball: it solves on the line of its SOURCE — the segment
// the RT follows, which its x₀ and reference come from — not on the line of
// whatever catch-core segment was published last. The two differ when a late
// prediction jump is published (S2, line L2) and the RT does not take it:
// the hand is on S1 and must stop on L1. A published stop segment inherits
// its source's line, so the later stop grid points find it too.
TEST(ApproachPlanner, AStopCoreSolvesOnTheLineOfTheSegmentTheRtFollows) {
  const std::int64_t lag = kTArm + kReplan + 2 * kH;

  // What the RT reports at the catch node: S1 only (it refused S2, or S2 aged
  // out), S2 only, or S1 with S2 pending (S2 starts before t_c, so S2 it is).
  struct Report {
    const char* what;
    bool pending_s2;
    bool active_s2;
    bool expect_l2;
  };

  for (const Arm& arm : {Arm6(), Arm7()}) {
    for (const Report& report :
         {Report{"follows S1", false, false, false}, Report{"follows S2", false, true, true},
          Report{"follows S1, S2 pending", true, false, true}}) {
      SCOPED_TRACE(std::to_string(arm.model->nv) + " joints, RT " + report.what);
      Rig r(arm, PerpParams(kPerp));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);  // S1, on the plan's line L1
      ASSERT_NE(s.seq, 0U);
      const MpcSegmentBallTarget moved = MovedBall(s.c);
      std::int64_t now = kT0 + 20 * kMs;
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                   moved, r.out, r.rec))
          << Why(r.rec);
      const std::uint32_t s2 = r.Publish(now);  // S2, on the moved ball's line L2
      const Eigen::Vector3d l1_d = s.c.v.normalized();
      const Eigen::Vector3d l2_d = moved.v_b.normalized();
      // The two lines are apart in both members, and the LAST published
      // catch-core line is L2 in every case.
      ASSERT_GT((moved.p_b - s.c.p).norm(), 0.005);
      ASSERT_GT((l2_d - l1_d).norm(), 0.04);
      ASSERT_EQ(r.planner.ApproachCoreInput(6).p_c, moved.p_b);

      const Eigen::Vector3d& want_p = report.expect_l2 ? moved.p_b : s.c.p;
      const Eigen::Vector3d& want_d = report.expect_l2 ? l2_d : l1_d;
      std::uint32_t pending = report.pending_s2 ? s2 : 0;
      std::uint32_t active = report.active_s2 ? s2 : s.seq;
      const std::uint32_t source0 = report.expect_l2 ? s2 : s.seq;
      for (const int k : {0, 1, 2}) {
        now = s.t_c + k * kDt - lag - 1 * kMs;
        SetClock(now);
        // A valid ball is passed on every wake: a stop core does not read it.
        ASSERT_TRUE(
            r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, pending, active),
                             moved, r.out, r.rec))
            << k << ": " << Why(r.rec);
        EXPECT_EQ(r.rec.kind, SegmentKind::kStop);
        // k = 0 starts on the catch-core segment the RT reports; k = 1, 2 on
        // the stop segment published just before — which carries no line of
        // its own, only its source's.
        EXPECT_EQ(r.rec.source_seq, k == 0 ? source0 : active) << k;
        EXPECT_EQ(r.planner.StopCoreInput(k).p_c, want_p) << k;
        EXPECT_LT((r.planner.StopCoreInput(k).d_hat - want_d).norm(), 1e-12) << k;
        active = r.Publish(now);
        pending = 0;
      }
    }

    // A re-solve that is NOT published gives no segment its line: the stop
    // core still starts on S1, on L1.
    {
      Rig r(arm, PerpParams(kPerp));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U);
      std::int64_t now = kT0 + 20 * kMs;
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                   MovedBall(s.c), r.out, r.rec))
          << Why(r.rec);
      now = s.t_c - lag - 1 * kMs;
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq),
                                   MpcSegmentBallTarget{}, r.out, r.rec))
          << Why(r.rec);
      EXPECT_EQ(r.planner.StopCoreInput(0).p_c, s.c.p);
      EXPECT_LT((r.planner.StopCoreInput(0).d_hat - s.c.v.normalized()).norm(), 1e-12);
    }

    // The retry without a reference keeps the line. The published S2 is made
    // to end off rest (5e-4 > the core's 1e-4), so the stop core refuses it as
    // a reference and is re-solved from nothing — still on L2.
    {
      Rig r(arm, PerpParams(kPerp));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U);
      const MpcSegmentBallTarget moved = MovedBall(s.c);
      std::int64_t now = kT0 + 20 * kMs;
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                   moved, r.out, r.rec))
          << Why(r.rec);
      for (int j = 0; j < r.out.nv; ++j) {
        r.out.qd[static_cast<std::size_t>(r.out.n_nodes * kMaxSegmentNv + j)] = 5e-4;
      }
      const std::uint32_t s2 = r.Publish(now);
      now = s.t_c - lag - 1 * kMs;
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s2),
                                   MpcSegmentBallTarget{}, r.out, r.rec))
          << Why(r.rec);
      EXPECT_TRUE(r.rec.cold_retry);
      EXPECT_EQ(r.planner.StopCoreInput(0).p_c, moved.p_b);
      EXPECT_LT((r.planner.StopCoreInput(0).d_hat - moved.v_b.normalized()).norm(), 1e-12);
    }
  }
}

// The lines sit beside the ring and move with it: after more re-solves than
// the ring holds — each on a ball of its own — every segment still in the
// ring answers with ITS line, the never-evicted followed one included.
TEST(ApproachPlanner, EvictionKeepsEachSegmentsLineWithIt) {
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  Rig r(Arm6(), PerpParams(kPerp));
  const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
  ASSERT_NE(s.seq, 0U);

  struct Line {
    std::uint32_t seq;
    Eigen::Vector3d p;
    Eigen::Vector3d d;
  };

  std::vector<Line> lines{Line{s.seq, s.c.p, s.c.v.normalized()}};
  // The RT keeps following S1 while twelve same-point re-solves are published.
  for (int i = 1; i <= 12; ++i) {
    const MpcSegmentBallTarget ball = MovedBall(s.c, 0.1 * i);
    const std::int64_t now = kT0 + (10 + i) * kMs;
    SetClock(now);
    ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq),
                                 ball, r.out, r.rec))
        << i << ": " << Why(r.rec);
    lines.push_back(Line{r.Publish(now), ball.p_b, ball.v_b.normalized()});
  }
  const std::int64_t now = s.t_c - lag - 1 * kMs;  // the catch node (stop k = 0)
  int in_ring = 0;
  for (const Line& line : lines) {
    const PlannerRtState rt = FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, line.seq);
    if (r.planner.SourceSeq(rt, s.t_c) == 0) {
      continue;  // evicted
    }
    ++in_ring;
    SetClock(now);
    ASSERT_TRUE(r.planner.Replan(rt, MpcSegmentBallTarget{}, r.out, r.rec))
        << line.seq << ": " << Why(r.rec);
    EXPECT_EQ(r.rec.source_seq, line.seq);
    EXPECT_EQ(r.planner.StopCoreInput(0).p_c, line.p) << line.seq;
    EXPECT_LT((r.planner.StopCoreInput(0).d_hat - line.d).norm(), 1e-12) << line.seq;
  }
  EXPECT_EQ(in_ring, 8);  // the ring is full: S1 and the seven newest
  EXPECT_NE(
      r.planner.SourceSeq(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq), s.t_c),
      0U);
}

// A source segment that carries no line → the stop core is withheld,
// recorded, and no QP runs; it is never solved on a default line. A line
// goes to ONE segment: the one published right after the solve that built
// it. The same sequences with the term off solve as they always did.
TEST(ApproachPlanner, AStopCoreWhoseSourceHasNoLineIsWithheld) {
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  for (const double w : {kPerp, 0.0}) {
    const bool on = w > 0.0;
    SCOPED_TRACE(on ? "w_perp on" : "w_perp off");
    const auto expect = [on](bool ok, const SegmentRecord& rec, std::uint32_t source) {
      EXPECT_EQ(rec.kind, SegmentKind::kStop);
      EXPECT_EQ(rec.k, 0);
      EXPECT_EQ(rec.source_seq, source);  // followed: only the line is missing
      if (on) {
        EXPECT_FALSE(ok);
        EXPECT_EQ(rec.outcome, SegmentOutcome::kNoBall);
        ExpectNotSolved(rec);
      } else {
        EXPECT_TRUE(ok) << Why(rec);
      }
    };
    // (a) A segment this planner never solved (another plan's, handed to the
    // ring): the replan has its source, the source has no line. The line of
    // the first segment went to the first segment — once.
    {
      Rig r(Arm6(), PerpParams(w));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U);
      SegmentSnapshot foreign = r.out;
      foreign.plan_id = 8;
      foreign.segment_seq = 50;
      r.planner.NotePublished(foreign);
      const std::int64_t now = s.t_c - lag - 1 * kMs;
      SetClock(now);
      const bool ok = r.planner.Replan(
          FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, 50, /*plan_id=*/8), BallFor(s.c),
          r.out, r.rec);
      expect(ok, r.rec, 50U);
    }
    // (b) ResetTrial drops the lines with the ring — the published segment's
    // and the one a publishable, not yet published re-solve is holding. The
    // first segment handed back to the ring is followed again, without a line.
    {
      Rig r(Arm6(), PerpParams(w));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U);
      const SegmentSnapshot first = r.out;
      std::int64_t now = kT0 + 20 * kMs;
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                   BallFor(s.c), r.out, r.rec))
          << Why(r.rec);
      now = s.t_c - lag - 1 * kMs;
      const PlannerRtState rt = FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, s.seq);
      r.planner.ResetTrial();
      EXPECT_EQ(r.planner.SourceSeq(rt, s.t_c), 0U);  // the ring is gone
      // Straight after the reset, with no solve call in between: the line the
      // unpublished re-solve held must not land on this segment.
      r.planner.NotePublished(first);
      SetClock(now);
      const bool ok = r.planner.Replan(rt, BallFor(s.c), r.out, r.rec);
      expect(ok, r.rec, s.seq);
    }
    // (c) A line belongs to the solve that built it: a publishable re-solve
    // is NOT published, another Replan call comes (and withholds), and only
    // then is the re-solve's segment handed to the ring — it carries no line.
    {
      Rig r(Arm6(), PerpParams(w));
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U);
      std::int64_t now = kT0 + 20 * kMs;
      const PlannerRtState pre = FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0);
      SetClock(now);
      ASSERT_TRUE(r.planner.Replan(pre, BallFor(s.c), r.out, r.rec)) << Why(r.rec);
      SegmentSnapshot late = r.out;
      SegmentSnapshot scratch{};
      SetClock(now);
      ASSERT_FALSE(r.planner.Replan(pre, MpcSegmentBallTarget{}, scratch, r.rec));
      ASSERT_EQ(r.rec.outcome, SegmentOutcome::kNoBall);  // the pre-catch meaning: no ball
      late.segment_seq = 60;
      r.planner.NotePublished(late);
      now = s.t_c - lag - 1 * kMs;
      SetClock(now);
      const bool ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, 60),
                                       BallFor(s.c), r.out, r.rec);
      expect(ok, r.rec, 60U);
    }
  }
}

// The parser's upper bound is a value the cores do solve with: at
// kMpcSegmentStopPathWeightMax the planner configures (every warm-up), and the
// first solve, a re-solve on a moved ball and the three stop cores all come
// out publishable, on both arms.
TEST(ApproachPlanner, TheStopPathWeightSolvesAtItsUpperBound) {
  const std::int64_t lag = kTArm + kReplan + 2 * kH;
  for (const Arm& arm : {Arm6(), Arm7()}) {
    Rig r(arm, PerpParams(rtc::catching::kMpcSegmentStopPathWeightMax));
    ASSERT_TRUE(r.planner.Configured());
    ASSERT_EQ(r.planner.StopCoreParams(0).w_perp, 1e4);
    const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
    ASSERT_NE(s.seq, 0U) << Why(r.rec);
    int iterations = r.rec.iterations;
    std::int64_t now = kT0 + 20 * kMs;
    SetClock(now);
    ASSERT_TRUE(r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                                 MovedBall(s.c), r.out, r.rec))
        << Why(r.rec);
    iterations = std::max(iterations, r.rec.iterations);
    std::uint32_t followed = r.Publish(now);
    for (const int k : {0, 1, 2}) {
      now = s.t_c + k * kDt - lag - 1 * kMs;
      SetClock(now);
      ASSERT_TRUE(
          r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, followed),
                           MpcSegmentBallTarget{}, r.out, r.rec))
          << k << ": " << Why(r.rec);
      iterations = std::max(iterations, r.rec.iterations);
      followed = r.Publish(now);
    }
    std::printf("[ record ] n = %d: w_perp %.0f, most QP iterations in one solve %d\n",
                static_cast<int>(arm.model->nv), rtc::catching::kMpcSegmentStopPathWeightMax,
                iterations);
  }
}

// A ball at or below v_eps has no direction of travel: with the term on the
// solve is withheld and says so; with it off the same input solves as it
// always did (the catch terms take a ball at rest).
TEST(ApproachPlanner, ABallWithoutADirectionIsWithheldOnlyWithTheTermOn) {
  const double v_eps = Consts().v_eps;
  for (const double w : {kPerp, 0.0}) {
    const bool on = w > 0.0;
    SCOPED_TRACE(on ? "w_perp on" : "w_perp off");
    for (const Arm& arm : {Arm6(), Arm7()}) {
      Rig r(arm, PerpParams(w));
      const Catch c = CatchAt(r.arm, Offset(r.arm, 0.04));
      const std::int64_t t_c = kT0 + kTArm + 800 * kMs;
      const rtc::catching::MpcSegmentCoreInput warm = r.planner.ApproachCoreInput(6);

      // The first solve: below v_eps, exactly at it (an axis vector, so the
      // norm IS v_eps), and just above it.
      struct Row {
        Eigen::Vector3d v;
        bool has_direction;
      };

      for (const Row& row : {Row{0.1 * v_eps * c.v.normalized(), false},
                             Row{Eigen::Vector3d(0.0, v_eps, 0.0), false},
                             Row{Eigen::Vector3d(0.0, 2.0 * v_eps, 0.0), true}}) {
        Catch still = c;
        still.v = row.v;  // a_d stays the plan's unit axis
        SetClock(kT0);
        const bool ok =
            r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                                PlanFor(r.arm, still, t_c), BallFor(still), r.out, r.rec);
        EXPECT_EQ(r.rec.kind, SegmentKind::kFirst);
        if (on && !row.has_direction) {
          EXPECT_FALSE(ok);
          EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoBall) << row.v.norm();
          EXPECT_EQ(r.rec.k, -6);  // the grid point is recorded even when withheld
          ExpectNotSolved(r.rec);
          // No line was written: the core's input still holds the warm-up's.
          EXPECT_EQ(r.planner.ApproachCoreInput(6).p_c, warm.p_c);
          EXPECT_EQ(r.planner.ApproachCoreInput(6).d_hat, warm.d_hat);
        } else {
          EXPECT_TRUE(ok) << row.v.norm() << ": " << Why(r.rec);
        }
      }
      // A pre-catch replan with such a ball (a hand-made target: the one
      // MakeMpcSegmentBallTarget builds is already invalid below v_eps).
      const Started s = StartPlan(r, kT0, 800 * kMs, 0.04);
      ASSERT_NE(s.seq, 0U);
      MpcSegmentBallTarget ball = BallFor(s.c);
      ball.v_b = 0.1 * v_eps * s.c.v.normalized();
      const std::int64_t now = kT0 + 20 * kMs;
      const PlannerRtState rt = FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0);
      SetClock(now);
      const bool ok = r.planner.Replan(rt, ball, r.out, r.rec);
      EXPECT_EQ(r.rec.kind, SegmentKind::kSame);
      if (on) {
        EXPECT_FALSE(ok);
        EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoBall);
        ExpectNotSolved(r.rec);
        // Finite components whose norm is not: no direction either, and a
        // different reason.
        ball.v_b = Eigen::Vector3d(1e200, 1e200, 0.0);
        SetClock(now);
        EXPECT_FALSE(r.planner.Replan(rt, ball, r.out, r.rec));
        EXPECT_EQ(r.rec.outcome, SegmentOutcome::kInputNonFinite);
        ExpectNotSolved(r.rec);
      } else {
        EXPECT_TRUE(ok) << Why(r.rec);
      }
    }
  }
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
  const MpcSegmentBallTarget b =
      rtc::catching::MakeMpcSegmentBallTarget(t, c, true, t.s[3].t_ns, 1e-6);
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
  const MpcSegmentBallTarget b = rtc::catching::MakeMpcSegmentBallTarget(t, c, true, at, 1e-6);
  ASSERT_TRUE(b.sigma_valid);
  EXPECT_NEAR(b.sigma_p(1, 1), 0.8 * c.c[2][7] + 0.2 * c.c[3][7], 1e-18);
  EXPECT_NEAR(b.sigma_p(1, 0), 0.8 * c.c[2][6] + 0.2 * c.c[3][6], 1e-18);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(b.sigma_p);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
  // The last sample is itself.
  const MpcSegmentBallTarget last =
      rtc::catching::MakeMpcSegmentBallTarget(t, c, true, t.s[9].t_ns, 1e-6);
  ASSERT_TRUE(last.sigma_valid);
  EXPECT_EQ(last.sigma_p(0, 0), c.c[9][0]);
}

TEST(ApproachBallTarget, RefusesWhatItCannotTrust) {
  const auto t = Line();
  const auto c = Cov(t);
  const std::int64_t mid = t.s[2].t_ns + 10 * kMs;
  // Another snapshot's covariance: the ball, but no Σ_p.
  MpcSegmentBallTarget b = rtc::catching::MakeMpcSegmentBallTarget(t, c, false, mid, 1e-6);
  EXPECT_TRUE(b.valid);
  EXPECT_FALSE(b.sigma_valid);
  auto bad = c;
  bad.c[3][7] = kNan;
  EXPECT_FALSE(rtc::catching::MakeMpcSegmentBallTarget(t, bad, true, mid, 1e-6).sigma_valid);
  bad = c;
  bad.n = 9;
  EXPECT_FALSE(rtc::catching::MakeMpcSegmentBallTarget(t, bad, true, mid, 1e-6).sigma_valid);
  // Outside the horizon: extrapolated, neither.
  b = rtc::catching::MakeMpcSegmentBallTarget(t, c, true, t.s[9].t_ns + 1, 1e-6);
  EXPECT_FALSE(b.valid);
  EXPECT_FALSE(b.sigma_valid);
  b = rtc::catching::MakeMpcSegmentBallTarget(t, c, true, t.s[0].t_ns - 1, 1e-6);
  EXPECT_FALSE(b.valid);
  // A ball slower than v_eps has no direction of travel.
  EXPECT_FALSE(rtc::catching::MakeMpcSegmentBallTarget(t, c, true, mid, 5.0).valid);
  EXPECT_FALSE(rtc::catching::MakeMpcSegmentBallTarget(t, c, true, mid, kNan).valid);
}

// ── 5a. Configure is a full reset (E1-F12 #738) ──────────────────────────────

// One fixed sequence — the first segment of a plan for a catch `reach` away,
// then two replans from what the RT reports — published as the cycle does,
// under seqs 1, 2, 3, with every segment and record digested
// (planner_trace_digest.hpp). The plan id and the catch instant are the same
// whatever `reach` is.
std::uint64_t SolveSequenceDigest(Rig& r, double reach) {
  rtc::testing::ValueDigest h;
  const auto add = [&](bool ok) {
    h.Add(ok);
    rtc::testing::AddSegment(h, r.out);
    rtc::testing::AddSegmentRecord(h, r.rec);
  };
  r.seq = 0;
  const std::int64_t now = kT0;
  const Catch c = CatchAt(r.arm, Offset(r.arm, reach));
  const std::int64_t t_c = now + kTArm + 800 * kMs;
  SetClock(now);
  bool ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, now - kH), PlanFor(r.arm, c, t_c),
                                BallFor(c), r.out, r.rec);
  add(ok);
  std::uint32_t reported = ok ? r.Publish(now) : 0;
  for (int i = 1; i <= 2; ++i) {
    const std::int64_t later = now + i * 20 * kMs;
    SetClock(later);
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, later - kH, t_c, reported, 0),
                          BallFor(c), r.out, r.rec);
    add(ok);
    if (ok) {
      reported = r.Publish(later);
    }
  }
  SetClock(kT0);
  return h.Value();
}

TEST(ApproachPlanner, AReconfiguredPlannerIsANewOne) {
  // Configure is a full reset: a planner that has solved and published and is
  // configured again answers exactly as one built and configured now.
  // PlannerCycle relies on it — ConfigureMpcSegmentPlanner installs a NEW planner on every
  // configure (E1-F12 #738) where it once configured the one in place again.
  // The planner is first used on ANOTHER catch under the same plan id, catch
  // instant and seqs: a planner that kept its ring would then start a replan
  // from that catch's segment.
  constexpr double kReach = 0.03;
  constexpr double kOtherReach = 0.05;
  for (const double w_perp : {0.0, kPerp}) {
    SCOPED_TRACE("w_perp " + std::to_string(w_perp));
    Rig fresh(Arm6(), PerpParams(w_perp));
    const std::uint64_t expected = SolveSequenceDigest(fresh, kReach);

    // Non-vacuity: WITHOUT a Configure in between, the other catch does change
    // what the sequence answers — there is state for Configure to clear.
    Rig dirty(Arm6(), PerpParams(w_perp));
    static_cast<void>(SolveSequenceDigest(dirty, kOtherReach));
    EXPECT_NE(SolveSequenceDigest(dirty, kReach), expected);

    Rig used(Arm6(), PerpParams(w_perp));
    static_cast<void>(SolveSequenceDigest(used, kOtherReach));
    std::string err;
    ASSERT_TRUE(used.planner.Configure(MpcSegmentPlannerModelOf(used.arm), Consts(),
                                       PerpParams(w_perp), &FakeClock, &err))
        << err;
    EXPECT_EQ(SolveSequenceDigest(used, kReach), expected);
  }
}

// ── 5b. The SegmentPlanner entries (E1-F12 #738) ─────────────────────────────

// One solve's whole result as a number: the segment and the record, every
// value (planner_trace_digest.hpp).
std::uint64_t SolveDigest(bool ok, const SegmentSnapshot& out, const SegmentRecord& rec) {
  rtc::testing::ValueDigest h;
  h.Add(ok);
  rtc::testing::AddSegment(h, out);
  rtc::testing::AddSegmentRecord(h, rec);
  return h.Value();
}

TEST(ApproachPlanner, TheViewEntriesSolveWhatTheValueEntriesSolve) {
  // PlannerCycle hands the ball over as a view of the wake's trajectory and
  // covariance (SegmentPlanner::PlanFirst / Replan). Before the interface it
  // built the ball target itself — MakeMpcSegmentBallTarget at the plan's t_c, with
  // the planner's v_eps — and called the value entries. Two planners, one
  // driven each way from the same inputs, on a clock that STEPS on every read:
  // the same segment and the same record, bit for bit — solve_ns and the
  // number of clock reads with it, so the budget still measures the interval
  // it measured.
  constexpr std::int64_t kStep = 1000;  // ns per clock read

  struct BallCase {
    const char* name;
    bool empty;
    bool matched;
  };

  for (const double w_perp : {0.0, 2000.0}) {
    std::uint64_t matched_first = 0;
    std::uint64_t matched_replan = 0;
    for (const BallCase bc : {BallCase{"the trajectory's covariance", false, true},
                              BallCase{"another snapshot's covariance", false, false},
                              BallCase{"no prediction", true, false}}) {
      SCOPED_TRACE(std::string(bc.name) + ", w_perp " + std::to_string(w_perp));
      Rig by_value(Arm6(), PerpParams(w_perp));
      Rig by_view(Arm6(), PerpParams(w_perp));
      // Through the interface, as the cycle calls it.
      rtc::catching::SegmentPlanner& iface = by_view.planner;
      const Arm& arm = by_value.arm;
      const std::int64_t now = kT0;
      const Catch c = CatchAt(arm, Offset(arm, 0.03));
      const std::int64_t t_c = now + kTArm + 800 * kMs;
      // The ball passes the catch point at t_c, which is sample 14.
      const auto traj =
          rtc::testing::LineTrajectory(c.p, c.v, t_c - 14 * 50 * kMs, 50 * kMs, 20, 14,
                                       /*seq=*/1, /*gen=*/5, /*activation=*/3, now - 5 * kMs);
      const auto cov = rtc::testing::IsotropicCovariance(traj, 0.01);
      const rtc::catching::BallPrediction view =
          bc.empty ? rtc::catching::BallPrediction{}
                   : rtc::catching::BallPrediction{&traj, &cov, bc.matched};
      const auto value_at = [&](std::int64_t t_ns) {
        return bc.empty ? MpcSegmentBallTarget{}
                        : rtc::catching::MakeMpcSegmentBallTarget(traj, cov, bc.matched, t_ns,
                                                                  Consts().v_eps);
      };
      const PlannerRtState rest = RestingRt(arm, arm.q_nominal, now - kH);
      const PlanSnapshot plan = PlanFor(arm, c, t_c);

      // ── PlanFirst: the ball at plan.t_c_ns ────────────────────────────────
      SetClock(now, kStep);
      const bool ok_value =
          by_value.planner.PlanFirst(rest, plan, value_at(plan.t_c_ns), by_value.out, by_value.rec);
      const std::int64_t reads_value = (g_now.load() - now) / kStep;
      SetClock(now, kStep);
      const bool ok_view = iface.PlanFirst(rest, plan, view, by_view.out, by_view.rec);
      const std::int64_t reads_view = (g_now.load() - now) / kStep;
      ASSERT_TRUE(ok_value) << Why(by_value.rec);
      const std::uint64_t first = SolveDigest(ok_value, by_value.out, by_value.rec);
      EXPECT_EQ(SolveDigest(ok_view, by_view.out, by_view.rec), first) << Why(by_view.rec);
      EXPECT_EQ(reads_view, reads_value);
      EXPECT_GT(by_value.rec.solve_ns, 0) << "the stepping clock did not reach the solve";
      EXPECT_EQ(by_view.rec.solve_ns, by_value.rec.solve_ns);
      // Σ_p reaches the first solve only from a covariance that is the
      // trajectory's.
      EXPECT_EQ(by_view.rec.w_p_fallback, !(bc.matched && !bc.empty));
      ASSERT_TRUE(ok_view);
      by_value.Publish(now);
      by_view.Publish(now);

      // ── Replan: the ball at rt.plan_t_c_ns ────────────────────────────────
      const std::int64_t later = now + 20 * kMs;
      const PlannerRtState following =
          FollowingRt(arm, arm.q_nominal, later - kH, t_c, /*pending=*/1, /*active=*/0);
      SetClock(later, kStep);
      const bool re_value = by_value.planner.Replan(following, value_at(following.plan_t_c_ns),
                                                    by_value.out, by_value.rec);
      const std::int64_t re_reads_value = (g_now.load() - later) / kStep;
      SetClock(later, kStep);
      const bool re_view = iface.Replan(following, view, by_view.out, by_view.rec);
      const std::int64_t re_reads_view = (g_now.load() - later) / kStep;
      const std::uint64_t replan = SolveDigest(re_value, by_value.out, by_value.rec);
      EXPECT_EQ(SolveDigest(re_view, by_view.out, by_view.rec), replan) << Why(by_view.rec);
      EXPECT_EQ(re_reads_view, re_reads_value);
      EXPECT_EQ(by_view.rec.solve_ns, by_value.rec.solve_ns);
      // A pre-catch grid point needs the ball: an empty view is none.
      EXPECT_EQ(re_view, !bc.empty) << Why(by_view.rec);
      if (bc.empty) {
        EXPECT_EQ(by_view.rec.outcome, SegmentOutcome::kNoBall);
      }

      // Non-vacuity: the pairing flag and the emptiness the view carries DO
      // reach the solve — a view entry that dropped either would still match
      // a value entry fed the same wrong ball, but not the matched case's.
      if (bc.matched) {
        matched_first = first;
        matched_replan = replan;
      } else {
        EXPECT_NE(first, matched_first);
        EXPECT_NE(replan, matched_replan);
      }
    }
  }
  SetClock(kT0);
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
  const MpcSegmentBallTarget ball = BallFor(s.c);
  const PlanSnapshot plan = PlanFor(r.arm, s.c, s.t_c, 8);
  const std::int64_t now = kT0 + 20 * kMs;
  SetClock(now);
  bool ok = true;
  PlannerRtState moving = RestingRt(r.arm, r.arm.q_nominal, now - kH);
  moving.qd_cmd[0] = 0.3;
  Counts c = Gated([&] { ok = r.planner.PlanFirst(moving, plan, ball, r.out, r.rec); });
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotAtRest);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "not-at-rest path";
  PlanSnapshot late = plan;
  late.t_c_ns = now + kTArm + kFirst;
  c = Gated([&] {
    ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, now - kH), late, ball, r.out, r.rec);
  });
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kTooLate);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "too-late path";
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, 0, 0), ball, r.out,
                          r.rec);
  });
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNotFollowed);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "not-followed path";
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, now - kH, s.t_c, s.seq, 0),
                          MpcSegmentBallTarget{}, r.out, r.rec);
  });
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoBall);
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
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kUpToDate);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "up-to-date path";
  EXPECT_FALSE(ok);
}

TEST(ApproachPlanner, SolvesAllocateNothingOutsideProxQp) {
  // Through the solve the C mallocs are ProxQP's (#654) and cannot be told
  // apart from ours, so operator new is the gate (MD-23).
  Rig r(Arm7());
  const Started s = StartPlan(r, kT0, 800 * kMs);  // warm-up outside the gates
  ASSERT_NE(s.seq, 0U);
  const MpcSegmentBallTarget ball = BallFor(s.c);
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
  EXPECT_EQ(r.rec.kind, SegmentKind::kSame);
  EXPECT_EQ(c.op_new, 0U) << "same-point re-solve";
  r.Publish(same);
  const std::int64_t adv = s.t_c - kTArm - kReplan - 2 * kH - 4 * kDtPre - 10 * kMs;
  SetClock(adv);
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, adv - kH, s.t_c, r.seq, 0), ball,
                          r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, SegmentKind::kAdvance);
  EXPECT_EQ(c.op_new, 0U) << "grid advance";
  r.Publish(adv);
  const std::int64_t stop = s.t_c - kTArm - kReplan - 2 * kH - kMs;
  SetClock(stop);
  c = Gated([&] {
    ok = r.planner.Replan(FollowingRt(r.arm, r.arm.q_nominal, stop - kH, s.t_c, 0, r.seq), ball,
                          r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, SegmentKind::kStop);
  EXPECT_EQ(c.op_new, 0U) << "catch node → stop core";
  mallocs += c.c_malloc;
  RecordProperty("approach_qp_solver_mallocs", std::to_string(mallocs));
  std::printf("[alloc] approach first + stop: %zu C mallocs (ProxQP, #654)\n", mallocs);
}

TEST(ApproachPlanner, TheStopLineAllocatesNothing) {
  // `cost.w_perp` > 0: building the line, keeping it beside the ring at the
  // publish and the two withholds are fixed-size work — nothing on the heap. Through a
  // solve the C mallocs are ProxQP's (#654), so operator new is the gate.
  Rig r(Arm7(), PerpParams(kPerp));
  const Started s = StartPlan(r, kT0, 800 * kMs);  // warm-up outside the gates
  ASSERT_NE(s.seq, 0U);
  const MpcSegmentBallTarget ball = BallFor(s.c);
  bool ok = false;
  SetClock(kT0);
  Counts c = Gated([&] {
    ok = r.planner.PlanFirst(RestingRt(r.arm, r.arm.q_nominal, kT0 - kH),
                             PlanFor(r.arm, s.c, s.t_c), ball, r.out, r.rec);
  });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(c.op_new, 0U) << "first solve";
  r.out.segment_seq = ++r.seq;
  r.out.publish_ns = kT0;
  c = Gated([&] { r.planner.NotePublished(r.out); });
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "keeping the line with the segment";
  const std::int64_t same = kT0 + 20 * kMs;
  const PlannerRtState pre = FollowingRt(r.arm, r.arm.q_nominal, same - kH, s.t_c, r.seq, 0);
  SetClock(same);
  c = Gated([&] { ok = r.planner.Replan(pre, ball, r.out, r.rec); });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(c.op_new, 0U) << "same-point re-solve";
  MpcSegmentBallTarget still = ball;
  still.v_b = 1e-8 * ball.v_b.normalized();
  SetClock(same);
  c = Gated([&] { ok = r.planner.Replan(pre, still, r.out, r.rec); });
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoBall);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "no-direction path";
  const std::int64_t stop = s.t_c - kTArm - kReplan - 2 * kH - kMs;
  const PlannerRtState post = FollowingRt(r.arm, r.arm.q_nominal, stop - kH, s.t_c, 0, r.seq);
  SetClock(stop);
  c = Gated([&] { ok = r.planner.Replan(post, MpcSegmentBallTarget{}, r.out, r.rec); });
  EXPECT_TRUE(ok) << Why(r.rec);
  EXPECT_EQ(r.rec.kind, SegmentKind::kStop);
  EXPECT_EQ(c.op_new, 0U) << "stop core on its source's line";
  // A source without a line (a segment handed back after a reset): the next
  // stop grid point is withheld.
  SegmentSnapshot again = r.out;
  r.planner.ResetTrial();
  again.segment_seq = 70;
  r.planner.NotePublished(again);
  PlannerRtState lost = post;
  lost.segment_seq = 70;
  SetClock(stop + kDt);
  lost.rt_state_ns = stop + kDt - kH;
  c = Gated([&] { ok = r.planner.Replan(lost, MpcSegmentBallTarget{}, r.out, r.rec); });
  EXPECT_FALSE(ok);
  EXPECT_EQ(r.rec.outcome, SegmentOutcome::kNoBall) << Why(r.rec);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "no-line path";
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
    const MpcSegmentPlannerModel pm = MpcSegmentPlannerModelOf(arm);
    const double rss0 = RssMb();
    const auto t0 = std::chrono::steady_clock::now();
    MpcSegmentPlanner planner;
    std::string err;
    ASSERT_TRUE(planner.Configure(pm, Consts(), ApproachParams(), &FakeClock, &err)) << err;
    const double configure_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const double rss_mb = RssMb() - rss0;
    // The first trial solve of each catch core after the warm-up: n_pre 6 … 2
    // by shrinking the lead (n_pre 1 reaches too little to be a fair solve).
    const Catch c = CatchAt(arm, Offset(arm, 0.03));
    SegmentSnapshot out{};
    SegmentRecord rec{};
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
