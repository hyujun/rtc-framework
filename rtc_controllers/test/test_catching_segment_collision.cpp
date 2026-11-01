// The mpc segment planner with collision rows (#642) on the synthetic tree: the
// configure (the capsule file, its refusals, the ball's radius), a first
// segment whose plan sends the OTHER arm through the catch arm (rows active,
// the real distances re-measured, the control planner without rows inside
// d_safe), the ball rows from a prediction that crosses the other arm, the
// inert rows of a prediction that is empty, and the publish gate on the real
// distances (MpcSegmentCollisionClear).
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

namespace fx = rtc::testing::mpc_segment_core;
namespace rub = rtc_urdf_bridge;
using rtc::catching::BallPrediction;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::MpcSegmentBallTarget;
using rtc::catching::MpcSegmentCollisionCheck;
using rtc::catching::MpcSegmentCollisionClear;
using rtc::catching::MpcSegmentPlanner;
using rtc::catching::MpcSegmentPlannerConstants;
using rtc::catching::MpcSegmentPlannerModel;
using rtc::catching::MpcSegmentPlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;
using rtc::testing::SyntheticTree;

constexpr int kTrunk = 3;
constexpr int kArm = 7;
constexpr int kNv = kTrunk + 2 * kArm;  // 17
constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;
constexpr std::int64_t kTArm = 30 * kMs;
constexpr std::int64_t kH = 2 * kMs;
constexpr double kDSafe = 0.02;
constexpr double kMargin = 0.01;
constexpr double kBallDSafe = 0.02;
constexpr double kBallRadius = 0.035;

std::atomic<std::int64_t> g_now{kT0};

std::int64_t FixedClock() noexcept {
  return g_now.load(std::memory_order_relaxed);
}

std::size_t U(int i) {
  return static_cast<std::size_t>(i);
}

int OtherArm(int j) {
  return kTrunk + kArm + j;
}

// The model with a BODY frame per joint (the spec names links; the synthetic
// tree declares none). Device order = model order here.
fx::ArmModel WithLinkFrames() {
  fx::ArmModel arm = SyntheticTree(kTrunk, kArm, 2);
  auto model = std::make_shared<pinocchio::Model>(*arm.model);
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model->njoints); ++j) {
    model->addJointFrame(j);
    model->addBodyFrame(model->names[j] + "_link", j);
  }
  arm.model = model;
  return arm;
}

std::string SpecYaml(bool with_ball) {
  std::string y = "collision_capsules:\n  capsules:\n";
  y += "    - {name: trunk, link: trunk_2_link, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.05}\n";
  for (int b = 0; b < 2; ++b) {
    for (int j = 0; j < kArm; ++j) {
      y += "    - {name: a" + std::to_string(b) + "_" + std::to_string(j) + ", link: arm_" +
           std::to_string(b) + "_" + std::to_string(j) +
           "_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.03}\n";
    }
  }
  y += "  pairs:\n";
  for (const int i : {1, 3, 5}) {
    for (const int j : {1, 3, 5}) {
      y += "    - [a0_" + std::to_string(i) + ", a1_" + std::to_string(j) + "]\n";
    }
  }
  if (with_ball) {
    y += "  ball_capsules: [a1_2, a1_4, a1_6]\n";
  }
  return y;
}

// The spec written to a file the planner loads (`capsules_file`).
std::string WriteSpec(const std::string& name, bool with_ball) {
  const std::filesystem::path path =
      std::filesystem::path(::testing::TempDir()) / ("rtc_capsules_" + name + ".yaml");
  std::ofstream out(path);
  out << SpecYaml(with_ball);
  return path.string();
}

MpcSegmentPlannerModel ModelOf(const fx::ArmModel& a) {
  MpcSegmentPlannerModel pm;
  pm.arm = a.model;
  pm.catch_frame = a.frame;
  pm.nv = a.model->nv;
  for (int m = 0; m < pm.nv; ++m) {
    pm.device_of_model[U(m)] = m;
    pm.q_min[U(m)] = a.model->lowerPositionLimit[m];
    pm.q_max[U(m)] = a.model->upperPositionLimit[m];
    pm.qdot_max[U(m)] = a.model->upperVelocityLimit[m];
    pm.tau_max[U(m)] = a.model->upperEffortLimit[m];
  }
  return pm;
}

MpcSegmentPlannerParams Params(const std::string& capsules_file) {
  MpcSegmentPlannerParams p;
  p.n_nodes = 7;
  p.dt_s = 0.05;
  p.blocks = {1, 1, 2, 3};
  p.n_blocks = 4;
  p.k_max = 2;
  p.n_pre_max = 6;
  p.dt_pre_s = 0.1;
  p.horizon_explicit = true;
  p.collision.capsules_file = capsules_file;
  p.collision.d_safe = kDSafe;
  p.collision.margin = kMargin;
  p.collision.rho_d = 10.0;
  p.collision.ball_d_safe = kBallDSafe;
  p.collision.ball_n_sigma = 2.0;
  p.collision.ball_margin_fallback = 0.05;
  // The slack gate stays out of the way: the fixture reads the real distances.
  p.collision_slack_max = 1.0;
  return p;
}

MpcSegmentPlannerConstants Consts(double ball_radius = kBallRadius) {
  MpcSegmentPlannerConstants c;
  c.eta_v = 0.9;
  c.t_arm_s = static_cast<double>(kTArm) * 1e-9;
  c.control_dt = static_cast<double>(kH) * 1e-9;
  c.v_eps = 1e-6;
  c.ball_radius = ball_radius;
  return c;
}

struct Scene {
  fx::ArmModel tree = WithLinkFrames();
  pinocchio::Data data{*tree.model};
  std::shared_ptr<const rub::CapsuleModel> spec =
      rub::CapsuleModel::ParseYaml(SpecYaml(true), "scene");
  rub::CapsuleWorkspace ws;

  Scene() {
    rub::CapsuleWorkspace::BindOptions opt;
    opt.with_ball = true;
    opt.catch_frame = tree.frame;
    EXPECT_EQ(ws.Bind(spec, *tree.model, opt), rub::CapsuleBindStatus::kOk) << ws.BindError();
  }

  double MinSelfDistance(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(*tree.model, data, q);
    EXPECT_TRUE(ws.UpdatePlacements(*tree.model, data));
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
    double m = std::numeric_limits<double>::infinity();
    for (std::size_t p = 0; p < ws.NumSelfPairs(); ++p) {
      m = std::min(m, ws.Distance(p));
    }
    return m;
  }

  // A pose of the other arm that crosses the catch arm (see the core test).
  Eigen::VectorXd CrossingPose() {
    std::mt19937 rng(642);
    std::uniform_real_distribution<double> u(-0.7, 0.7);
    for (int trial = 0; trial < 50000; ++trial) {
      Eigen::VectorXd q = tree.q_nominal;
      for (int j = 0; j < kArm; ++j) {
        q[OtherArm(j)] =
            std::clamp(q[OtherArm(j)] + u(rng), tree.model->lowerPositionLimit[OtherArm(j)] + 0.1,
                       tree.model->upperPositionLimit[OtherArm(j)] + -0.1);
      }
      const double d = MinSelfDistance(q);
      if (d > -0.03 && d < -0.005) {
        return q;
      }
    }
    ADD_FAILURE() << "no crossing pose found — the fixture's geometry changed";
    return tree.q_nominal;
  }

  // The smallest real self distance along a published segment's nodes 1..N.
  double SegmentMinSelfDistance(const SegmentSnapshot& seg) {
    double m = std::numeric_limits<double>::infinity();
    for (int k = 1; k <= seg.n_nodes; ++k) {
      Eigen::VectorXd q(kNv);
      for (int j = 0; j < kNv; ++j) {
        q[j] = seg.q[U(k * kMaxSegmentNv + j)];
      }
      m = std::min(m, MinSelfDistance(q));
    }
    return m;
  }

  // The plan: the catch arm reaches a little, the other arm goes to q_other.
  PlanSnapshot PlanTo(const Eigen::VectorXd& q_other, std::int64_t t_c, Eigen::Vector3d& p_c,
                      Eigen::Vector3d& a_d) {
    Eigen::VectorXd q_catch = q_other;
    for (int m = 0; m < kTrunk + kArm; ++m) {
      q_catch[m] = tree.q_nominal[m] + (m % 2 == 0 ? 0.04 : -0.04);
    }
    pinocchio::forwardKinematics(*tree.model, data, q_catch);
    pinocchio::updateFramePlacement(*tree.model, data, tree.frame);
    p_c = data.oMf[tree.frame].translation();
    a_d = data.oMf[tree.frame].rotation().col(2);
    const Eigen::Vector3d v_c = -5.0 * a_d;
    PlanSnapshot plan{};
    plan.valid = true;
    plan.plan_id = 7;
    plan.t_c_ns = t_c;
    plan.token.activation_generation = 3;
    plan.token.generation = 5;
    plan.nv = kNv;
    plan.joint_set = rtc::catching::FullJointSet(kNv);
    for (int m = 0; m < kNv; ++m) {
      plan.q_star[U(m)] = q_catch[m];
    }
    for (int i = 0; i < 3; ++i) {
      plan.p_c[U(i)] = p_c[i];
      plan.v_c[U(i)] = v_c[i];
      plan.a_d[U(i)] = a_d[i];
    }
    return plan;
  }

  PlannerRtState RestingRt() const {
    PlannerRtState rt{};
    rt.valid = true;
    rt.activation_generation = 3;
    rt.rt_iteration = 1000;
    rt.rt_state_ns = kT0 - kH;
    rt.reset_epoch = 1;
    rt.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kTracking);
    rt.nv = kNv;
    rt.cmd_seeded = true;
    rt.track_seen = true;
    rt.track_generation = 5;
    for (int m = 0; m < kNv; ++m) {
      rt.q_cmd[U(m)] = tree.q_nominal[m];
    }
    return rt;
  }
};

MpcSegmentBallTarget BallAt(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                            const Eigen::Vector3d& a_d) {
  MpcSegmentBallTarget b;
  b.valid = true;
  b.p_b = p;
  b.v_b = v;
  b.a_d = a_d;
  b.sigma_p = 1e-4 * Eigen::Matrix3d::Identity();
  b.sigma_valid = true;
  return b;
}

}  // namespace

// ── Configure ────────────────────────────────────────────────────────────────

TEST(SegmentCollision, ConfigureLoadsTheSpecAndRefusesByName) {
  Scene sc;
  g_now = kT0;
  {
    MpcSegmentPlanner planner;
    std::string err;
    ASSERT_TRUE(planner.Configure(ModelOf(sc.tree), Consts(), Params(WriteSpec("ok", true)),
                                  &FixedClock, &err))
        << err;
    EXPECT_TRUE(planner.CollisionOn());
    ASSERT_NE(planner.Capsules(), nullptr);
    EXPECT_EQ(planner.Capsules()->NumPairs(), 9U);
    // Every core carries the self rows; the catch cores the ball rows too.
    EXPECT_EQ(planner.Core(0).NumCollisionPairs(), 9);
    EXPECT_EQ(planner.Core(0).NumBallPairs(), 0);
    EXPECT_EQ(planner.ApproachCore(3).NumCollisionPairs(), 9);
    EXPECT_EQ(planner.ApproachCore(3).NumBallPairs(), 3);
    EXPECT_EQ(planner.ApproachCoreInput(3).ball_radius, kBallRadius);
  }
  {
    MpcSegmentPlanner planner;
    std::string err;
    EXPECT_FALSE(planner.Configure(ModelOf(sc.tree), Consts(), Params("/nonexistent/capsules.yaml"),
                                   &FixedClock, &err));
    EXPECT_NE(err.find("capsules_file"), std::string::npos) << err;
    EXPECT_NE(err.find("/nonexistent/capsules.yaml"), std::string::npos) << err;
    EXPECT_FALSE(planner.CollisionOn());
  }
  {
    // Ball capsules without the ball's radius.
    MpcSegmentPlanner planner;
    std::string err;
    EXPECT_FALSE(planner.Configure(ModelOf(sc.tree),
                                   Consts(std::numeric_limits<double>::quiet_NaN()),
                                   Params(WriteSpec("ok", true)), &FixedClock, &err));
    EXPECT_NE(err.find("core.ball.diameter"), std::string::npos) << err;
    // ... which a spec without ball capsules does not need.
    EXPECT_TRUE(planner.Configure(ModelOf(sc.tree),
                                  Consts(std::numeric_limits<double>::quiet_NaN()),
                                  Params(WriteSpec("noball", false)), &FixedClock, &err))
        << err;
    EXPECT_EQ(planner.ApproachCore(3).NumBallPairs(), 0);
  }
  {
    // A spec the model refuses (a ball capsule on the catch arm): the core's
    // reason and the workspace's message, by name.
    const std::filesystem::path path =
        std::filesystem::path(::testing::TempDir()) / "rtc_capsules_bad.yaml";
    std::ofstream(path) << R"(
collision_capsules:
  capsules:
    - {name: a0_5, link: arm_0_5_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.03}
    - {name: a1_5, link: arm_1_5_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.03}
  pairs: [[a0_5, a1_5]]
  ball_capsules: [a0_5])";
    MpcSegmentPlanner planner;
    std::string err;
    EXPECT_FALSE(
        planner.Configure(ModelOf(sc.tree), Consts(), Params(path.string()), &FixedClock, &err));
    EXPECT_NE(err.find("collision_invalid"), std::string::npos) << err;
    EXPECT_NE(err.find("a0_5"), std::string::npos) << err;
  }
}

// ── A first segment through the catch arm ────────────────────────────────────

TEST(SegmentCollision, FirstSegmentHoldsTheRealDistancesAndTheControlDoesNot) {
  Scene sc;
  const Eigen::VectorXd q_cross = sc.CrossingPose();
  ASSERT_LT(sc.MinSelfDistance(q_cross), 0.0);
  g_now = kT0;
  MpcSegmentPlanner on;
  MpcSegmentPlanner off;
  std::string err;
  ASSERT_TRUE(
      on.Configure(ModelOf(sc.tree), Consts(), Params(WriteSpec("on", true)), &FixedClock, &err))
      << err;
  ASSERT_TRUE(off.Configure(ModelOf(sc.tree), Consts(), Params(""), &FixedClock, &err)) << err;
  EXPECT_FALSE(off.CollisionOn());

  const std::int64_t t_c = kT0 + kTArm + 800 * kMs;
  Eigen::Vector3d p_c;
  Eigen::Vector3d a_d;
  const PlanSnapshot plan = sc.PlanTo(q_cross, t_c, p_c, a_d);
  const MpcSegmentBallTarget ball = BallAt(p_c, -5.0 * a_d, a_d);
  const PlannerRtState rt = sc.RestingRt();

  SegmentSnapshot seg_on{};
  SegmentRecord rec_on{};
  const bool ok_on = on.PlanFirst(rt, plan, ball, seg_on, rec_on);
  SegmentSnapshot seg_off{};
  SegmentRecord rec_off{};
  const bool ok_off = off.PlanFirst(rt, plan, ball, seg_off, rec_off);
  ASSERT_TRUE(ok_off) << SegmentOutcomeName(rec_off.outcome) << " / " << rec_off.core_reason_name;
  EXPECT_FALSE(rec_off.coll_rows);
  EXPECT_TRUE(std::isnan(rec_off.coll_min_distance));
  EXPECT_EQ(rec_off.coll_active_nodes, 0);

  EXPECT_TRUE(rec_on.coll_rows);
  ASSERT_TRUE(ok_on) << SegmentOutcomeName(rec_on.outcome) << " / " << rec_on.core_reason_name
                     << " coll_min " << rec_on.coll_min_distance << " slack "
                     << rec_on.coll_slack_max;
  EXPECT_GE(rec_on.coll_active_nodes, 1);
  ASSERT_TRUE(std::isfinite(rec_on.coll_min_distance));
  ASSERT_TRUE(std::isfinite(rec_on.coll_slack_max));
  EXPECT_TRUE(std::isfinite(rec_on.coll_eps_lin));
  // The real distances (the record's are the re-check's) honour d_safe less
  // the slack, and the published nodes measure the same with the test's own
  // workspace.
  EXPECT_GE(rec_on.coll_min_distance, kDSafe - rec_on.coll_slack_max - 1e-9);
  EXPECT_LE(rec_on.coll_eps_lin, kMargin);
  const double seg_min_on = sc.SegmentMinSelfDistance(seg_on);
  const double seg_min_off = sc.SegmentMinSelfDistance(seg_off);
  EXPECT_NEAR(seg_min_on, rec_on.coll_min_distance, 1e-9);
  EXPECT_LT(seg_min_off, kDSafe) << "the control group never came close — the gate is vacuous";
  EXPECT_GT(seg_min_on, seg_min_off);
  // The ball rows: the plan's ball target is a point at t_c; without a
  // prediction every pre-catch node's ball row is inert and recorded.
  EXPECT_EQ(rec_on.ball_inert_nodes, seg_on.n_pre);
  EXPECT_TRUE(rec_on.coll_fallback);
  EXPECT_EQ(rec_on.ball_active_nodes, 0);
  std::cout << "[segment collision] outcome=" << SegmentOutcomeName(rec_on.outcome)
            << " active_nodes=" << rec_on.coll_active_nodes
            << " slack_max=" << rec_on.coll_slack_max << " d_min on/off=" << seg_min_on << "/"
            << seg_min_off << " eps_lin=" << rec_on.coll_eps_lin
            << " solve_ns on/off=" << rec_on.solve_ns << "/" << rec_off.solve_ns << "\n";
}

// ── The ball rows from a prediction ──────────────────────────────────────────

TEST(SegmentCollision, BallRowsFromThePredictionAndInertWithoutOne) {
  Scene sc;
  g_now = kT0;
  MpcSegmentPlanner on;
  MpcSegmentPlanner off;
  std::string err;
  ASSERT_TRUE(
      on.Configure(ModelOf(sc.tree), Consts(), Params(WriteSpec("ball", true)), &FixedClock, &err))
      << err;
  ASSERT_TRUE(off.Configure(ModelOf(sc.tree), Consts(), Params(""), &FixedClock, &err)) << err;
  // The other arm rests; the ball's path crosses its forearm two pre-catch
  // intervals before t_c on its way to the catch point.
  const std::int64_t t_c = kT0 + kTArm + 800 * kMs;
  Eigen::Vector3d p_c;
  Eigen::Vector3d a_d;
  const PlanSnapshot plan = sc.PlanTo(sc.tree.q_nominal, t_c, p_c, a_d);
  pinocchio::forwardKinematics(*sc.tree.model, sc.data, sc.tree.q_nominal);
  const Eigen::Vector3d p_hit =
      sc.data.oMi[sc.tree.model->getJointId("arm_1_4")].translation() + Eigen::Vector3d(0.05, 0, 0);
  // A straight ball through p_hit at t_c − 0.2 s ending at the catch point.
  const Eigen::Vector3d v = (p_c - p_hit) / 0.2;
  const auto traj =
      rtc::testing::LineTrajectory(p_c, v, t_c - 14 * 50 * kMs, 50 * kMs, 20, 14,
                                   /*seq=*/1, /*gen=*/5, /*activation=*/3, kT0 - 5 * kMs);
  const auto cov = rtc::testing::IsotropicCovariance(traj, 0.01);
  const BallPrediction view{&traj, &cov, true};
  const BallPrediction empty{};
  const PlannerRtState rt = sc.RestingRt();
  const MpcSegmentBallTarget ball = BallAt(p_c, v, -v.normalized());

  SegmentSnapshot seg{};
  SegmentRecord rec{};
  ASSERT_TRUE(on.PlanFirst(rt, plan, ball, seg, rec, view))
      << SegmentOutcomeName(rec.outcome) << " / " << rec.core_reason_name << " ball_min "
      << rec.ball_min_distance;
  EXPECT_EQ(rec.ball_inert_nodes, 0);
  EXPECT_FALSE(rec.coll_fallback);
  EXPECT_GE(rec.ball_active_nodes, 1) << "the ball never crossed the other arm — fixture";
  ASSERT_TRUE(std::isfinite(rec.ball_min_distance));
  EXPECT_GE(rec.ball_min_distance, kBallDSafe - 1e-9);
  // The control group: the arm stays where the ball passes through it.
  SegmentSnapshot seg_off{};
  SegmentRecord rec_off{};
  ASSERT_TRUE(off.PlanFirst(rt, plan, ball, seg_off, rec_off, view))
      << SegmentOutcomeName(rec_off.outcome);
  {
    // The ON planner's catch core re-measures the OFF segment: the ball
    // passes through the arm there. (The core's ball is the one the ON solve
    // posed — the same prediction.)
    Eigen::MatrixXd q(kNv, seg_off.n_nodes + 1);
    for (int k = 0; k <= seg_off.n_nodes; ++k) {
      for (int j = 0; j < kNv; ++j) {
        q(j, k) = seg_off.q[U(k * kMaxSegmentNv + j)];
      }
    }
    ASSERT_EQ(seg_off.n_pre, seg.n_pre);
    MpcSegmentCollisionCheck chk =
        const_cast<rtc::catching::MpcSegmentCore&>(on.ApproachCore(seg.n_pre))
            .RecheckCollision(q, Eigen::VectorXd(Eigen::VectorXd::Zero(0)));
    ASSERT_TRUE(chk.evaluated);
    EXPECT_LT(chk.ball_min, kBallDSafe)
        << "the control group never came close — the gate is vacuous";
  }
  // No prediction: every pre-catch ball row inert, flagged, and the solve
  // still publishes.
  SegmentSnapshot seg_empty{};
  SegmentRecord rec_empty{};
  g_now = kT0;
  ASSERT_TRUE(on.PlanFirst(rt, plan, ball, seg_empty, rec_empty, empty))
      << SegmentOutcomeName(rec_empty.outcome);
  EXPECT_EQ(rec_empty.ball_inert_nodes, seg_empty.n_pre);
  EXPECT_TRUE(rec_empty.coll_fallback);
  EXPECT_EQ(rec_empty.ball_active_nodes, 0);
  EXPECT_TRUE(std::isnan(rec_empty.ball_min_distance));
  std::cout << "[segment ball] active_nodes=" << rec.ball_active_nodes
            << " ball_min=" << rec.ball_min_distance << " n_pre=" << seg.n_pre << "\n";
}

// ── The publish gate on the real distances ───────────────────────────────────

TEST(SegmentCollision, TheGateReadsTheRealClearances) {
  MpcSegmentCollisionCheck chk;
  EXPECT_FALSE(MpcSegmentCollisionClear(chk, 1e-5)) << "not evaluated";
  chk.evaluated = true;
  EXPECT_TRUE(MpcSegmentCollisionClear(chk, 1e-5)) << "no row measured: nothing to refuse";
  chk.self_clearance = 0.004;
  EXPECT_TRUE(MpcSegmentCollisionClear(chk, 1e-5));
  chk.self_clearance = -0.5e-5;
  EXPECT_TRUE(MpcSegmentCollisionClear(chk, 1e-5)) << "inside the solver's tolerance";
  chk.self_clearance = -2e-5;
  EXPECT_FALSE(MpcSegmentCollisionClear(chk, 1e-5)) << "the linear model passed, the real one not";
  chk.self_clearance = 0.01;
  chk.ball_clearance = -0.003;
  EXPECT_FALSE(MpcSegmentCollisionClear(chk, 1e-5)) << "a ball row inside its safe distance";
  chk.ball_clearance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(MpcSegmentCollisionClear(chk, 1e-5)) << "no ball row";
  chk.self_clearance = std::numeric_limits<double>::quiet_NaN();
  chk.ball_clearance = 0.0;
  EXPECT_TRUE(MpcSegmentCollisionClear(chk, 1e-5));
  chk.self_clearance = -std::numeric_limits<double>::infinity();
  EXPECT_FALSE(MpcSegmentCollisionClear(chk, 1e-5));
}
