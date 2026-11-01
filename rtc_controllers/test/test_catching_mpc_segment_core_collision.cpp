// MpcSegmentCore with collision rows (#642 E3-F03) on the synthetic tree (a trunk and two arms,
// 17 joints, the catch frame on arm 0). The judged properties:
//   • without a spec the core carries no collision row or variable (the golden
//     suites pin the QP itself);
//   • counter-swing: the other arm's reference crosses the catch arm — the self
//     rows are active, the solver honours them on the LINEAR model (residual
//     within 10·eps_abs), the REAL distances re-measured at the solution keep
//     d_safe − s_d,k (the margin covers the linearisation error, ε_lin ≤ m),
//     and the control group (rows off) ends inside d_safe;
//   • the fixed point of re-linearising at the solution: linear = nonlinear, so
//     the real clearance holds to 10·eps_abs;
//   • the ball rows: the ball's swept capsule crosses the other arm — hard rows
//     honoured, real clearance ≥ 0, an invalid sample's node inert and counted,
//     and the swept capsule never farther than the segment's end points or
//     midpoint;
//   • the dense oracle assembles the same rows; ρ_τ = 0 leaves the rows
//     unchanged (the collision branch refreshes the FK itself);
//   • no allocation up to the QP with the rows on.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/mpc_block_grid.hpp"
#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
using rtc::catching::BlockPattern;
using rtc::catching::kCollisionActiveTol;
using rtc::catching::MpcSegmentCollisionCheck;
using rtc::catching::MpcSegmentCore;
using rtc::catching::MpcSegmentCoreInput;
using rtc::catching::MpcSegmentCoreParams;
using rtc::catching::MpcSegmentCoreReason;
using rtc::catching::MpcSegmentCoreReasonName;
using rtc::catching::MpcSegmentCoreResult;
using rtc::testing::SyntheticTree;
using rtc::testing::mpc_segment_core::ArmModel;
using rtc::testing::mpc_segment_core::LimitsFromModel;
using rtc::testing::mpc_segment_core::RestInput;
using rtc::testing::mpc_segment_core::UseAsReference;

constexpr int kTrunk = 3;
constexpr int kArm = 7;
constexpr int kNv = kTrunk + 2 * kArm;  // 17
constexpr int kNPre = 4;
constexpr double kDtPre = 0.1;
constexpr int kNStop = 6;
constexpr double kDt = 0.025;
constexpr int kN = kNPre + kNStop;

constexpr double kDSafe = 0.02;
constexpr double kMargin = 0.01;
constexpr double kRhoD = 5.0;
constexpr double kBallDSafe = 0.02;
constexpr double kBallRadius = 0.035;
constexpr double kBallMargin = 0.005;

ArmModel WithLinkFrames() {
  return rtc::testing::SyntheticTreeWithLinkFrames(kTrunk, kArm, 2);
}

// The cross pairs, the trunk pairs, the ball against the OTHER arm.
std::string SpecYaml() {
  return rtc::testing::SyntheticTreeCapsuleYaml(kTrunk, kArm, true, true);
}

MpcSegmentCoreParams BaseParams(bool collision,
                                const std::shared_ptr<const rub::CapsuleModel>& spec) {
  MpcSegmentCoreParams p;
  p.n_pre = kNPre;
  p.dt_pre = kDtPre;
  p.n_nodes = kNStop;
  p.dt = kDt;
  const BlockPattern fine = [] {
    BlockPattern b;
    for (const int s : {1, 1, 1, 1, 1, 2, 3}) {
      b.block_sizes[static_cast<std::size_t>(b.n_blocks++)] = s;
    }
    return b;
  }();
  p.n_blocks = fine.n_blocks;
  p.block_sizes = fine.block_sizes;
  p.w_delta = 5.0;
  if (collision) {
    p.collision.capsules = spec;
    p.collision.d_safe = kDSafe;
    p.collision.margin = kMargin;
    p.collision.rho_d = kRhoD;
    p.collision.ball_d_safe = kBallDSafe;
  }
  return p;
}

struct Fixture {
  ArmModel arm = WithLinkFrames();
  rtc::catching::MpcSegmentCoreLimits limits = LimitsFromModel(*arm.model);
  std::shared_ptr<const rub::CapsuleModel> spec = rub::CapsuleModel::ParseYaml(SpecYaml(), "core");
  pinocchio::Data data{*arm.model};
  rub::CapsuleWorkspace ws;  // the test's own measure, with the ball

  Fixture() {
    rub::CapsuleWorkspace::BindOptions opt;
    opt.with_ball = true;
    opt.catch_frame = arm.frame;
    EXPECT_EQ(ws.Bind(spec, *arm.model, opt), rub::CapsuleBindStatus::kOk) << ws.BindError();
  }

  // Smallest real self-pair distance at q.
  double MinSelfDistance(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(*arm.model, data, q);
    EXPECT_TRUE(ws.UpdatePlacements(*arm.model, data));
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
    double m = std::numeric_limits<double>::infinity();
    for (std::size_t p = 0; p < ws.NumSelfPairs(); ++p) {
      m = std::min(m, ws.Distance(p));
    }
    return m;
  }

  // Smallest real ball-pair distance at q with the ball at (p0, p1).
  double MinBallDistance(const Eigen::VectorXd& q, const Eigen::Vector3d& p0,
                         const Eigen::Vector3d& p1) {
    pinocchio::forwardKinematics(*arm.model, data, q);
    EXPECT_TRUE(ws.UpdatePlacements(*arm.model, data));
    ws.SetBall(p0, p1, kBallRadius);
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
    double m = std::numeric_limits<double>::infinity();
    for (std::size_t p = ws.NumSelfPairs(); p < ws.NumPairs(); ++p) {
      m = std::min(m, ws.Distance(p));
    }
    return m;
  }

  // Joint m of arm 1 (the other arm) in model order.
  static int OtherArm(int j) { return kTrunk + kArm + j; }

  // A pose of the other arm that crosses the catch arm: random steps on its
  // joints until the smallest pair distance sits just inside penetration.
  Eigen::VectorXd CrossingPose() {
    std::mt19937 rng(642);
    std::uniform_real_distribution<double> u(-0.7, 0.7);
    for (int trial = 0; trial < 50000; ++trial) {
      Eigen::VectorXd q = arm.q_nominal;
      for (int j = 0; j < kArm; ++j) {
        q[OtherArm(j)] = std::clamp(q[OtherArm(j)] + u(rng), limits.q_min[OtherArm(j)] + 0.1,
                                    limits.q_max[OtherArm(j)] - 0.1);
      }
      const double d = MinSelfDistance(q);
      if (d > -0.03 && d < -0.005) {
        return q;
      }
    }
    ADD_FAILURE() << "no crossing pose found — the fixture's geometry changed";
    return arm.q_nominal;
  }

  // The reference: the other arm reaches q_cross over the pre-catch part
  // (minimum jerk, at rest at the catch node and after), everything else
  // resting at the nominal pose. x_0 = (q_nominal, 0, 0).
  MpcSegmentCoreInput CrossingInput(const Eigen::VectorXd& q_cross) const {
    MpcSegmentCoreInput in = RestInput(arm.q_nominal);
    in.q_ref.setZero(kNv, kN + 1);
    in.qd_ref.setZero(kNv, kN + 1);
    in.qdd_ref.setZero(kNv, kN + 1);
    const double t_span = kNPre * kDtPre;
    for (int k = 0; k <= kN; ++k) {
      const double s = static_cast<double>(std::min(k, kNPre)) / kNPre;
      const double s2 = s * s;
      const double s3 = s2 * s;
      const double pos = 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s2;
      const double vel = (30.0 * s2 - 60.0 * s3 + 30.0 * s2 * s2) / t_span;
      const double acc = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (t_span * t_span);
      for (int m = 0; m < kNv; ++m) {
        const double step = q_cross[m] - arm.q_nominal[m];
        in.q_ref(m, k) = arm.q_nominal[m] + step * pos;
        in.qd_ref(m, k) = step * vel;
        in.qdd_ref(m, k) = step * acc;
      }
    }
    in.reference_valid = true;
    in.cold_start = true;
    return in;
  }
};

double ActiveTol(const MpcSegmentCoreParams& p) {
  return kCollisionActiveTol * p.solver.eps_abs;
}

}  // namespace

// ── Off: no row, no variable ─────────────────────────────────────────────────

TEST(MpcSegmentCoreCollision, OffCarriesNothing) {
  Fixture f;
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, BaseParams(false, f.spec), f.limits),
            MpcSegmentCoreReason::kNone);
  EXPECT_FALSE(core.CollisionTermActive());
  EXPECT_EQ(core.NumCollisionPairs(), 0);
  EXPECT_EQ(core.NumBallPairs(), 0);
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  EXPECT_EQ(r.coll_slack.size(), 0);
  EXPECT_EQ(r.coll_active.cols(), 0);
  MpcSegmentCoreInput in = f.CrossingInput(f.arm.q_nominal);
  ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
  EXPECT_FALSE(r.collision_evaluated);
  EXPECT_FALSE(core.RecheckCollision(r.q, r.coll_slack).evaluated);
}

TEST(MpcSegmentCoreCollision, InitRejectsBadParamsByName) {
  Fixture f;
  MpcSegmentCore core;
  MpcSegmentCoreParams p = BaseParams(true, f.spec);
  p.collision.rho_d = 0.0;
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits),
            MpcSegmentCoreReason::kCollisionInvalid);
  EXPECT_NE(core.CollisionError().find("rho_d"), std::string::npos) << core.CollisionError();
  // A ball capsule on the catch arm: the workspace's refusal, by name.
  const auto bad = rub::CapsuleModel::ParseYaml(R"(
collision_capsules:
  capsules:
    - {name: a0_5, link: arm_0_5_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.03}
    - {name: a1_5, link: arm_1_5_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.03}
  pairs: [[a0_5, a1_5]]
  ball_capsules: [a0_5])",
                                                "bad");
  p = BaseParams(true, bad);
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits),
            MpcSegmentCoreReason::kCollisionInvalid);
  EXPECT_NE(core.CollisionError().find("a0_5"), std::string::npos) << core.CollisionError();
  EXPECT_FALSE(core.IsInitialized());
  // A result sized for another core is refused, not resized.
  MpcSegmentCore ok;
  ASSERT_EQ(ok.Init(*f.arm.model, f.arm.frame, BaseParams(true, f.spec), f.limits),
            MpcSegmentCoreReason::kNone);
  EXPECT_EQ(ok.NumCollisionPairs(), 13);
  EXPECT_EQ(ok.NumBallPairs(), 3);
  MpcSegmentCoreResult r;
  ok.ResizeResult(r);
  r.coll_active.resize(0, 0);
  MpcSegmentCoreInput in = f.CrossingInput(f.arm.q_nominal);
  EXPECT_FALSE(ok.Solve(in, r));
  EXPECT_EQ(r.reason, MpcSegmentCoreReason::kDimMismatch);
}

// ── Counter-swing: the self rows ─────────────────────────────────────────────

TEST(MpcSegmentCoreCollision, SelfRowsHoldAtTheSolutionAndTheControlDoesNot) {
  Fixture f;
  const Eigen::VectorXd q_cross = f.CrossingPose();
  ASSERT_LT(f.MinSelfDistance(q_cross), 0.0);
  ASSERT_GT(f.MinSelfDistance(f.arm.q_nominal), kDSafe + kMargin);
  const MpcSegmentCoreParams on = BaseParams(true, f.spec);
  const MpcSegmentCoreParams off = BaseParams(false, f.spec);
  MpcSegmentCore core_on;
  MpcSegmentCore core_off;
  ASSERT_EQ(core_on.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone)
      << core_on.CollisionError();
  ASSERT_EQ(core_off.Init(*f.arm.model, f.arm.frame, off, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r_on;
  MpcSegmentCoreResult r_off;
  core_on.ResizeResult(r_on);
  core_off.ResizeResult(r_off);
  MpcSegmentCoreInput in = f.CrossingInput(q_cross);
  ASSERT_TRUE(core_on.Solve(in, r_on)) << MpcSegmentCoreReasonName(r_on.reason);
  ASSERT_TRUE(core_off.Solve(in, r_off)) << MpcSegmentCoreReasonName(r_off.reason);

  ASSERT_TRUE(r_on.collision_evaluated);
  EXPECT_GE(r_on.coll_active_nodes, 1) << "the other arm did not reach the catch arm";
  EXPECT_LT(r_on.coll_min_distance_ref, kDSafe) << "the reference does not cross";
  // (a) the linear model, the solver's own tolerance.
  EXPECT_GE(r_on.coll_residual_min, -ActiveTol(on));
  // (b) the real distances at the solution: the slack the solve took is
  // honoured, and the margin covered the linearisation error.
  const MpcSegmentCollisionCheck chk = core_on.RecheckCollision(r_on.q, r_on.coll_slack);
  ASSERT_TRUE(chk.evaluated);
  EXPECT_EQ(chk.invalid_pairs, 0);
  EXPECT_GE(chk.self_clearance, 0.0) << "node " << chk.worst_node << " self_min " << chk.self_min
                                     << " slack_max " << r_on.coll_slack_max;
  EXPECT_LE(chk.eps_lin, kMargin) << "ε_lin exceeds the margin m";
  // The control group: the same input without the rows ends inside d_safe,
  // measured with the ON core's own workspace on the OFF solution.
  const MpcSegmentCollisionCheck ctrl =
      core_on.RecheckCollision(r_off.q, Eigen::VectorXd(Eigen::VectorXd::Zero(0)));
  ASSERT_TRUE(ctrl.evaluated);
  EXPECT_LT(ctrl.self_min, kDSafe) << "the control group never came close — the gate is vacuous";
  EXPECT_LT(ctrl.self_min, chk.self_min);
  std::cout << "[collision self] active_nodes=" << r_on.coll_active_nodes
            << " active_rows=" << r_on.coll_active_rows << " slack_max=" << r_on.coll_slack_max
            << " residual_min=" << r_on.coll_residual_min
            << " d_ref_min=" << r_on.coll_min_distance_ref << " d_true_min=" << chk.self_min
            << " clearance=" << chk.self_clearance << " eps_lin=" << chk.eps_lin
            << " control d_true_min=" << ctrl.self_min << " iterations on/off=" << r_on.iterations
            << "/" << r_off.iterations << " linearize_us=" << r_on.linearize_us << "\n";
}

TEST(MpcSegmentCoreCollision, FixedPointOfRelinearisationHoldsTheRealClearance) {
  Fixture f;
  const Eigen::VectorXd q_cross = f.CrossingPose();
  // The map x̄ ↦ solve(x̄) is a proximal-point iteration (the pull toward x̄
  // vanishes at its fixed point), so its fixed point minimises the OTHER
  // terms under the rows. With nothing pulling the other arm it would walk
  // back to rest, far from any row; a rest term toward the crossing pose
  // keeps the rows active at the fixed point, and the iteration converges
  // linearly at the rate w_Δ sets against that term.
  MpcSegmentCoreParams on = BaseParams(true, f.spec);
  on.w_delta = 1.0;  // a lighter proximal term converges faster (rate ≈ w_Δ/(w_Δ + μ))
  on.collision.rho_d = 50.0;
  on.w_rest = Eigen::VectorXd::Zero(kNv);
  for (int j = 0; j < kArm; ++j) {
    on.w_rest[Fixture::OtherArm(j)] = 50.0;
  }
  on.q_rest = q_cross;
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  MpcSegmentCoreInput in = f.CrossingInput(q_cross);
  ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
  double step = std::numeric_limits<double>::infinity();
  int it = 0;
  constexpr int kMaxIterations = 500;
  for (; it < kMaxIterations && step > 1e-6; ++it) {
    const Eigen::MatrixXd prev = r.q;
    UseAsReference(r, in);
    in.cold_start = false;
    in.w_delta_scale = 1.0;
    ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason) << " at iteration " << it;
    step = (r.q - prev).cwiseAbs().maxCoeff();
    if (it % 25 == 0 || step < 1e-6) {
      std::cout << "[collision fixed point] it=" << it << " step=" << step
                << " slack_max=" << r.coll_slack_max << " residual_min=" << r.coll_residual_min
                << " active_rows=" << r.coll_active_rows << " d_lin_min=" << r.coll_min_distance
                << " qp_iters=" << r.iterations << "\n";
    }
  }
  std::cout << "[collision fixed point] iterations=" << it << " last step=" << step << "\n";
  ASSERT_LT(step, 1e-6) << "no fixed point within " << kMaxIterations << " re-linearisations";
  // At the fixed point the linear model IS the nonlinear one: the real
  // clearance holds to the solver's tolerance, and the margins come out as
  // extra distance.
  const MpcSegmentCollisionCheck chk = core.RecheckCollision(r.q, r.coll_slack);
  ASSERT_TRUE(chk.evaluated);
  EXPECT_GE(r.coll_residual_min, -ActiveTol(on));
  EXPECT_GE(chk.self_clearance, kMargin - ActiveTol(on));
  EXPECT_LE(std::abs(chk.eps_lin), 1e-6);
  EXPECT_GE(r.coll_active_nodes, 1);
}

// ── The ball rows ────────────────────────────────────────────────────────────

TEST(MpcSegmentCoreCollision, BallRowsHoldAndInvalidSamplesAreInert) {
  Fixture f;
  // The ball flies through the other arm's forearm (its elbow link at the
  // nominal pose) between nodes 2 and 3, along −y at 4 m/s.
  pinocchio::forwardKinematics(*f.arm.model, f.data, f.arm.q_nominal);
  const pinocchio::JointIndex elbow = f.arm.model->getJointId("arm_1_4");
  const Eigen::Vector3d p_hit = f.data.oMi[elbow].translation() + Eigen::Vector3d(0.05, 0.0, 0.0);
  const Eigen::Vector3d v(0.0, -4.0, 0.0);
  const double t_hit = 2.5 * kDtPre;
  const auto at = [&](double t) { return Eigen::Vector3d(p_hit + v * (t - t_hit)); };

  const MpcSegmentCoreParams on = BaseParams(true, f.spec);
  const MpcSegmentCoreParams off = BaseParams(false, f.spec);
  MpcSegmentCore core_on;
  MpcSegmentCore core_off;
  ASSERT_EQ(core_on.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone)
      << core_on.CollisionError();
  ASSERT_EQ(core_off.Init(*f.arm.model, f.arm.frame, off, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r_on;
  MpcSegmentCoreResult r_off;
  core_on.ResizeResult(r_on);
  core_off.ResizeResult(r_off);
  // The other arm stays: the reference is the nominal pose at rest.
  MpcSegmentCoreInput in = f.CrossingInput(f.arm.q_nominal);
  in.ball_radius = kBallRadius;
  for (int k = 1; k <= kNPre; ++k) {
    in.ball_valid[static_cast<std::size_t>(k)] = k != 4;  // node 4: no sample
    in.ball_p0.col(k) = at((k - 1) * kDtPre);
    in.ball_p1.col(k) = at(k * kDtPre);
    in.ball_margin[static_cast<std::size_t>(k)] = kBallMargin;
  }
  // The swept capsule is never farther than any point of the segment.
  for (int k = 1; k <= kNPre; ++k) {
    const Eigen::Vector3d p0 = in.ball_p0.col(k);
    const Eigen::Vector3d p1 = in.ball_p1.col(k);
    const double swept = f.MinBallDistance(f.arm.q_nominal, p0, p1);
    for (const Eigen::Vector3d& pt : {p0, p1, Eigen::Vector3d(0.5 * (p0 + p1))}) {
      EXPECT_LE(swept, f.MinBallDistance(f.arm.q_nominal, pt, pt) + 1e-12) << "node " << k;
    }
  }
  ASSERT_LT(f.MinBallDistance(f.arm.q_nominal, in.ball_p0.col(3), in.ball_p1.col(3)), 0.0)
      << "the ball does not cross the other arm — fixture";

  ASSERT_TRUE(core_on.Solve(in, r_on)) << MpcSegmentCoreReasonName(r_on.reason);
  ASSERT_TRUE(core_off.Solve(in, r_off)) << MpcSegmentCoreReasonName(r_off.reason);
  ASSERT_TRUE(r_on.collision_evaluated);
  EXPECT_EQ(r_on.ball_inert_nodes, 1);
  EXPECT_GE(r_on.ball_active_nodes, 1);
  EXPECT_GE(r_on.ball_residual_min, -ActiveTol(on));
  const MpcSegmentCollisionCheck chk = core_on.RecheckCollision(r_on.q, r_on.coll_slack);
  ASSERT_TRUE(chk.evaluated);
  EXPECT_GE(chk.ball_clearance, 0.0) << "ball_min " << chk.ball_min;
  EXPECT_LE(chk.eps_lin, std::max(kMargin, kBallMargin));
  // The control group: the arm stays and the ball passes through it.
  const MpcSegmentCollisionCheck ctrl =
      core_on.RecheckCollision(r_off.q, Eigen::VectorXd(Eigen::VectorXd::Zero(0)));
  ASSERT_TRUE(ctrl.evaluated);
  EXPECT_LT(ctrl.ball_min, kBallDSafe);
  // The inert node: no row of it is active, whatever the ball does there.
  for (int p = core_on.NumCollisionPairs();
       p < core_on.NumCollisionPairs() + core_on.NumBallPairs(); ++p) {
    EXPECT_EQ(r_on.coll_active(p, 4), 0);
  }
  std::cout << "[collision ball] active_nodes=" << r_on.ball_active_nodes
            << " inert_nodes=" << r_on.ball_inert_nodes
            << " residual_min=" << r_on.ball_residual_min << " d_true_min=" << chk.ball_min
            << " clearance=" << chk.ball_clearance << " control d_true_min=" << ctrl.ball_min
            << " eps_lin=" << chk.eps_lin << "\n";
}

TEST(MpcSegmentCoreCollision, TheVelocitySlackKeepsItsColumnBesideTheCollisionSlacks) {
  // The collision slacks are appended after s_v. A ball the catch frame
  // cannot match (3 m/s against a 0.3 m/s allowance) needs s_v > 0; the other
  // arm at rest keeps every self row inactive, so the collision slacks stay
  // zero. Read through the wrong column, s_v would show up as the last
  // node's collision slack.
  Fixture f;
  MpcSegmentCoreParams on = BaseParams(true, f.spec);
  on.catch_terms = true;
  on.rho_v = 2.0;
  on.v_rel_allow = 0.3;
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone)
      << core.CollisionError();
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  MpcSegmentCoreInput in = f.CrossingInput(f.arm.q_nominal);
  in.p_b = Eigen::Vector3d(0.3, 0.0, 1.0);
  in.v_b = Eigen::Vector3d(0.0, 0.0, -3.0);
  ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
  ASSERT_TRUE(r.collision_evaluated);
  EXPECT_GT(r.slack_v, 0.1) << "the arm cannot match the ball: s_v carries the rows";
  EXPECT_EQ(r.coll_active_nodes, 0);
  EXPECT_LT(r.coll_slack.maxCoeff(), 10.0 * ActiveTol(on)) << r.coll_slack.transpose();
  std::cout << "[collision + s_v] slack_v=" << r.slack_v
            << " coll_slack_max=" << r.coll_slack.maxCoeff() << "\n";
}

// ── Assembly oracles ─────────────────────────────────────────────────────────

TEST(MpcSegmentCoreCollision, DenseOracleAndTorqueOffAssembleTheSameRows) {
  Fixture f;
  const Eigen::VectorXd q_cross = f.CrossingPose();
  MpcSegmentCoreParams structured = BaseParams(true, f.spec);
  MpcSegmentCoreParams dense = structured;
  dense.reference_assembly = true;
  MpcSegmentCoreParams no_torque = structured;
  no_torque.rho_tau = 0.0;
  MpcSegmentCore a;
  MpcSegmentCore b;
  MpcSegmentCore c;
  ASSERT_EQ(a.Init(*f.arm.model, f.arm.frame, structured, f.limits), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(b.Init(*f.arm.model, f.arm.frame, dense, f.limits), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(c.Init(*f.arm.model, f.arm.frame, no_torque, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult ra;
  MpcSegmentCoreResult rb;
  MpcSegmentCoreResult rc;
  a.ResizeResult(ra);
  b.ResizeResult(rb);
  c.ResizeResult(rc);
  MpcSegmentCoreInput in = f.CrossingInput(q_cross);
  ASSERT_TRUE(a.Solve(in, ra)) << MpcSegmentCoreReasonName(ra.reason);
  ASSERT_TRUE(b.Solve(in, rb)) << MpcSegmentCoreReasonName(rb.reason);
  ASSERT_TRUE(c.Solve(in, rc)) << MpcSegmentCoreReasonName(rc.reason);
  const Eigen::Index rows = kN + kN * a.NumCollisionPairs() + kNPre * a.NumBallPairs();
  const Eigen::MatrixXd ca = a.MainQp().C.bottomRows(rows);
  const Eigen::MatrixXd cb = b.MainQp().C.bottomRows(rows);
  const Eigen::MatrixXd cc = c.MainQp().C.bottomRows(rows);
  EXPECT_LE((ca - cb).cwiseAbs().maxCoeff(), 1e-13 * std::max(1.0, ca.cwiseAbs().maxCoeff()));
  EXPECT_EQ((ca - cc).cwiseAbs().maxCoeff(), 0.0)
      << "ρ_τ = 0 changed the collision rows: the branch did not refresh the FK itself";
  const Eigen::VectorXd la = a.MainQp().l.tail(rows);
  const Eigen::VectorXd lc = c.MainQp().l.tail(rows);
  EXPECT_EQ((la - lc).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_GT(ca.cwiseAbs().maxCoeff(), 0.0);
  EXPECT_LE((ra.q - rb.q).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(MpcSegmentCoreCollision, NoAllocationUpToTheQp) {
  Fixture f;
  const Eigen::VectorXd q_cross = f.CrossingPose();
  const MpcSegmentCoreParams on = BaseParams(true, f.spec);
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  MpcSegmentCoreInput in = f.CrossingInput(q_cross);
  in.ball_radius = kBallRadius;
  for (int k = 1; k <= kNPre; ++k) {
    in.ball_valid[static_cast<std::size_t>(k)] = true;
    // Far from the arms: rows present, none binding.
    in.ball_p0.col(k) = Eigen::Vector3d(1.5, 1.5, 1.5);
    in.ball_p1.col(k) = Eigen::Vector3d(1.4, 1.6, 1.5);
    in.ball_margin[static_cast<std::size_t>(k)] = kBallMargin;
  }
  ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
  MpcSegmentCoreInput warm = in;
  UseAsReference(r, warm);
  warm.cold_start = false;
  ASSERT_TRUE(core.Solve(warm, r)) << MpcSegmentCoreReasonName(r.reason);
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    const bool ok = core.Solve(warm, r);
    EXPECT_TRUE(ok);
    EXPECT_EQ(new_gate.count(), 0U) << "operator new inside Solve";
    (void)malloc_gate.count();  // ProxQP's own C mallocs — counted, not asserted
  }
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    const MpcSegmentCollisionCheck chk = core.RecheckCollision(r.q, r.coll_slack);
    EXPECT_TRUE(chk.evaluated);
    EXPECT_EQ(new_gate.count(), 0U);
    EXPECT_EQ(malloc_gate.count(), 0U) << "C-level allocation in the nonlinear re-check";
  }
  // A trust-region rejection after the whole linearisation and condensing:
  // the C-level gate covers the collision branch (FK, Jacobians, coal).
  MpcSegmentCoreInput outside = warm;
  for (int k = 2; k <= kN; ++k) {
    outside.q_ref(kNv - 1, k) = f.limits.q_max[kNv - 1] + 3.0 * on.delta_tr;
  }
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    const bool ok = core.Solve(outside, r);
    EXPECT_FALSE(ok);
    EXPECT_EQ(r.reason, MpcSegmentCoreReason::kTrustRegionConflict);
    EXPECT_GT(r.linearize_us, 0.0);
    EXPECT_EQ(new_gate.count(), 0U);
    EXPECT_EQ(malloc_gate.count(), 0U) << "C-level allocation up to the QP";
  }
}
