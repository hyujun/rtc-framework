// E1-F02 (#628): the decel node payload (DecelPlanSnapshot, trajectory.hpp) and
// its RT sampler (NodeTrajectoryFollower, node_follower.hpp). Each #628
// "Done when" item maps to a named test (spec on #628, MD-27 / MD-30):
//   1 payload shape + C² sampling   PayloadFitsItsBudget, Validate*,
//                                   SampleReproducesNodesAndIsContinuousAcrossThem,
//                                   AbsoluteInstantsKeepNanosecondResolution
//   2 FK consistency (MD-30)        FkConsistencyAlongTheSegment,
//                                   DeviceOrderIsMappedBeforeFk
//                                   (+ ClikResidualIsRecorded — informational)
//   3 RT: no allocation, cost       SampleAllocatesNothing,
//                                   ContendedCopyNeverTearsAndIsRecorded
//
// Fixtures break the representation symmetries a mapping bug would hide in:
// the device order is a non-identity permutation of the model order, and the
// payload's instants are realistic absolute steady ns (~1.7e18), not 0.
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/decel_mpc.hpp"  // kMaxDecelNodes seen through the core too
#include "rtc_controllers/catching/jerk_segment.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_tsid/types/wbc_types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

using rtc::catching::DecelNodeSample;
using rtc::catching::DecelPlanSnapshot;
using rtc::catching::kMaxDecelNodes;
using rtc::catching::kMaxDecelNv;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::ValidateDecelNodes;

constexpr std::int64_t kDtNs = 25'000'000;                 // Δ_s 0.025 s (MD-24)
constexpr int kNodes = 14;                                 // N_s (MD-24)
constexpr std::int64_t kTc = 1'727'000'000'123'456'789LL;  // realistic steady ns
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// ── Fixtures ─────────────────────────────────────────────────────────────────

struct Arm {
  std::shared_ptr<const pinocchio::Model> model;
  pinocchio::FrameIndex frame{0};
  Eigen::VectorXd q_nominal;  // model order
};

Arm RealArm6() {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(RTC_TEST_ROBOT_DESCRIPTIONS_DIR) + "/ur5e/urdf/ur5e.urdf";
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  Arm a;
  a.model = builder.GetFullModel();
  a.frame = a.model->getFrameId("tool0");
  a.q_nominal.resize(6);
  a.q_nominal << 0.0, -1.2, 1.3, -1.6, -1.57, 0.0;
  return a;
}

// device_of_model[m] = device index of model joint m — a non-identity
// permutation, so a follower that skipped the mapping would run FK on a
// shuffled q and the FK test would see it.
constexpr std::array<int, 6> kDeviceOfModel{2, 0, 5, 1, 4, 3};

std::size_t Idx(int k, int j) {
  return static_cast<std::size_t>(k * kMaxDecelNv + j);
}

// Consistent nodes: integrate a random piecewise-constant jerk from x0 (model
// order), store in DEVICE order. Consistency is what makes the closed form
// exact (jerk_segment.hpp), so C² is a property the sampler must reproduce.
// Node N is then put at rest (a published segment's terminal equality, which
// ValidateDecelNodes requires): only the last segment becomes inconsistent,
// and the continuity check below stops short of it.
DecelPlanSnapshot MakePlan(const Eigen::VectorXd& q0_model, std::uint32_t seed,
                           int n_nodes = kNodes, int k0 = 0) {
  const int nv = static_cast<int>(q0_model.size());
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jerk(-40.0, 40.0);
  std::uniform_real_distribution<double> vel(-1.0, 1.0);
  DecelPlanSnapshot p{};
  p.valid = true;
  p.nv = nv;
  p.n_nodes = n_nodes;
  p.dt_ns = kDtNs;
  p.k0 = k0;
  p.t_c_ns = kTc;
  p.t0_ns = kTc + static_cast<std::int64_t>(k0) * kDtNs;
  p.plan_id = 7;
  p.decel_seq = 1;
  const double dt = static_cast<double>(kDtNs) * 1e-9;
  Eigen::VectorXd q = q0_model;
  Eigen::VectorXd qd(nv);
  Eigen::VectorXd qdd = Eigen::VectorXd::Zero(nv);
  for (int j = 0; j < nv; ++j) {
    qd[j] = vel(rng);
  }
  for (int k = 0; k <= n_nodes; ++k) {
    for (int m = 0; m < nv; ++m) {
      const int d = kDeviceOfModel[static_cast<std::size_t>(m)];
      p.q[Idx(k, d)] = q[m];
      p.qd[Idx(k, d)] = qd[m];
      p.qdd[Idx(k, d)] = qdd[m];
    }
    if (k == n_nodes) {
      for (int j = 0; j < nv; ++j) {
        p.qd[Idx(k, j)] = 0.0;
        p.qdd[Idx(k, j)] = 0.0;
      }
    }
    for (int m = 0; m < nv; ++m) {
      const double u = jerk(rng);
      q[m] += qd[m] * dt + 0.5 * qdd[m] * dt * dt + u * dt * dt * dt / 6.0;
      qd[m] += qdd[m] * dt + 0.5 * u * dt * dt;
      qdd[m] += u * dt;
    }
  }
  return p;
}

// Model-order view of a device-order sample.
Eigen::VectorXd ToModel(const std::array<double, kMaxDecelNv>& dev, int nv) {
  Eigen::VectorXd out(nv);
  for (int m = 0; m < nv; ++m) {
    out[m] = dev[static_cast<std::size_t>(kDeviceOfModel[static_cast<std::size_t>(m)])];
  }
  return out;
}

class NodeFollowerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arm_ = RealArm6();
    ASSERT_EQ(arm_.model->nv, 6);
    ASSERT_TRUE(follower_.Init(arm_.model, arm_.frame, kDeviceOfModel));
  }

  Arm arm_;
  NodeTrajectoryFollower follower_;
};

// ── 1. Payload and sampling ──────────────────────────────────────────────────

TEST(DecelPayload, PayloadFitsItsBudget) {
  // MD-27: 3 node blocks × 8 joints × 25 nodes × 8 B = 4.8 KB plus a header.
  EXPECT_LT(sizeof(DecelPlanSnapshot), 5U * 1024U);
  EXPECT_GE(sizeof(DecelPlanSnapshot), 3U * kMaxDecelNv * (kMaxDecelNodes + 1) * sizeof(double));
  // The core and the payload share one capacity (the constant moved to
  // trajectory.hpp; the core's block array is sized by it).
  EXPECT_EQ(rtc::catching::DecelMpcParams{}.block_sizes.size(),
            static_cast<std::size_t>(kMaxDecelNodes));
  RecordProperty("decel_payload_bytes", std::to_string(sizeof(DecelPlanSnapshot)));
}

TEST(DecelPayload, ValidateAcceptsAWellFormedPayload) {
  Eigen::VectorXd q0 = Eigen::VectorXd::Constant(6, 0.3);
  EXPECT_TRUE(ValidateDecelNodes(MakePlan(q0, 1)));
  EXPECT_TRUE(ValidateDecelNodes(MakePlan(q0, 2, kMaxDecelNodes)));
  EXPECT_TRUE(ValidateDecelNodes(MakePlan(q0, 3, 10, 4)));  // post-catch replan, k0 = 4
}

TEST(DecelPayload, ValidateRejectsEachMalformedField) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Constant(6, 0.3);
  const DecelPlanSnapshot good = MakePlan(q0, 1);
  auto expect_rejected = [&](auto mutate, const char* what) {
    DecelPlanSnapshot p = good;
    mutate(p);
    EXPECT_FALSE(ValidateDecelNodes(p)) << what;
  };
  expect_rejected([](DecelPlanSnapshot& p) { p.valid = false; }, "valid false");
  expect_rejected([](DecelPlanSnapshot& p) { p.nv = 0; }, "nv 0");
  expect_rejected([](DecelPlanSnapshot& p) { p.nv = kMaxDecelNv + 1; }, "nv over capacity");
  expect_rejected([](DecelPlanSnapshot& p) { p.n_nodes = 0; }, "no segment");
  expect_rejected([](DecelPlanSnapshot& p) { p.n_nodes = kMaxDecelNodes + 1; }, "nodes over cap");
  expect_rejected([](DecelPlanSnapshot& p) { p.dt_ns = 0; }, "dt 0");
  expect_rejected([](DecelPlanSnapshot& p) { p.k0 = -1; }, "negative grid index");
  expect_rejected([](DecelPlanSnapshot& p) { p.t0_ns = p.t_c_ns - 1; }, "node 0 before t_c");
  expect_rejected([](DecelPlanSnapshot& p) { p.t0_ns = p.t_c_ns + p.dt_ns; }, "t0 off k0's point");
  expect_rejected([](DecelPlanSnapshot& p) { p.t0_ns += 1; }, "t0 off the grid by 1 ns");
  // Node N must be at rest: the sampler holds it past the end.
  expect_rejected([](DecelPlanSnapshot& p) { p.qd[Idx(p.n_nodes, p.nv - 1)] = 2e-3; },
                  "node N moving");
  expect_rejected([](DecelPlanSnapshot& p) { p.qdd[Idx(p.n_nodes, 0)] = -2e-3; },
                  "node N accelerating");
  // A NaN in each block, at the LAST used node and joint — a validator that
  // stopped one short of n_nodes or nv would pass it.
  expect_rejected([](DecelPlanSnapshot& p) { p.q[Idx(p.n_nodes, p.nv - 1)] = kNan; }, "q NaN");
  expect_rejected([](DecelPlanSnapshot& p) { p.qd[Idx(p.n_nodes, p.nv - 1)] = kNan; }, "qd NaN");
  expect_rejected([](DecelPlanSnapshot& p) { p.qdd[Idx(p.n_nodes, p.nv - 1)] = kNan; }, "qdd NaN");
  // Within the rest tolerance is at rest; entries past the used shape are not
  // the validator's business.
  DecelPlanSnapshot p = good;
  p.qd[Idx(p.n_nodes, 0)] = 5e-4;
  p.q[Idx(p.n_nodes + 1, 0)] = kNan;
  p.q[Idx(0, p.nv)] = kNan;
  EXPECT_TRUE(ValidateDecelNodes(p));
}

TEST_F(NodeFollowerTest, SampleReproducesNodesAndIsContinuousAcrossThem) {
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 11);
  DecelNodeSample at{};
  DecelNodeSample lo{};
  DecelNodeSample hi{};
  double worst_node = 0.0;
  double worst_jump_q = 0.0;
  double worst_jump_qd = 0.0;
  double worst_jump_qdd = 0.0;
  for (int k = 0; k <= p.n_nodes; ++k) {
    const std::int64_t t = p.t0_ns + static_cast<std::int64_t>(k) * p.dt_ns;
    ASSERT_TRUE(follower_.Sample(p, t, at)) << "node " << k;
    for (int j = 0; j < p.nv; ++j) {
      const auto u = static_cast<std::size_t>(j);
      worst_node = std::max({worst_node, std::fabs(at.q[u] - p.q[Idx(k, j)]),
                             std::fabs(at.qd[u] - p.qd[Idx(k, j)]),
                             std::fabs(at.qdd[u] - p.qdd[Idx(k, j)])});
    }
    if (k == 0 || k == p.n_nodes) {
      continue;
    }
    // One microsecond either side: a C² trajectory moves by O(|q̈|·1e-6) in
    // q̇ and O(|jerk|·1e-6) in q̈; a wrong segment index jumps by O(1).
    ASSERT_TRUE(follower_.Sample(p, t - 1000, lo));
    ASSERT_TRUE(follower_.Sample(p, t + 1000, hi));
    for (std::size_t j = 0; j < static_cast<std::size_t>(p.nv); ++j) {
      worst_jump_q = std::max(worst_jump_q, std::fabs(hi.q[j] - lo.q[j]));
      worst_jump_qd = std::max(worst_jump_qd, std::fabs(hi.qd[j] - lo.qd[j]));
      worst_jump_qdd = std::max(worst_jump_qdd, std::fabs(hi.qdd[j] - lo.qdd[j]));
    }
  }
  EXPECT_LT(worst_node, 1e-12);
  EXPECT_LT(worst_jump_q, 1e-5);
  EXPECT_LT(worst_jump_qd, 1e-4);
  EXPECT_LT(worst_jump_qdd, 1e-3);  // |jerk| ≤ 40 rad/s³ → 8e-5 over 2 µs
}

TEST_F(NodeFollowerTest, AbsoluteInstantsKeepNanosecondResolution) {
  // t0 ~ 1.7e18 ns: converting the instant to seconds before subtracting would
  // lose ~100 ns and put an exact node query into the previous segment's end.
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 12, kNodes - 3, 3);
  DecelNodeSample s{};
  ASSERT_TRUE(follower_.Sample(p, p.t0_ns + 5 * p.dt_ns, s));
  EXPECT_DOUBLE_EQ(s.t_s, 5 * 0.025);
  for (int j = 0; j < p.nv; ++j) {
    EXPECT_NEAR(s.qdd[static_cast<std::size_t>(j)], p.qdd[Idx(5, j)], 1e-12);
  }
}

TEST_F(NodeFollowerTest, BeforeNodeZeroFailsAndLeavesTheOutputUntouched) {
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 13);
  DecelNodeSample s{};
  s.q[0] = 42.0;
  s.t_s = -7.0;
  EXPECT_FALSE(follower_.Sample(p, p.t0_ns - 1, s));
  EXPECT_EQ(s.q[0], 42.0);
  EXPECT_EQ(s.t_s, -7.0);
  std::array<double, kMaxDecelNv> q{};
  std::array<double, kMaxDecelNv> qd{};
  std::array<double, kMaxDecelNv> qdd{};
  EXPECT_FALSE(NodeTrajectoryFollower::SampleJoints(p, p.t0_ns - 1, q, qd, qdd));
}

TEST_F(NodeFollowerTest, HoldsNodeNPastTheEnd) {
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 14);
  DecelNodeSample s{};
  const std::int64_t end = p.t0_ns + static_cast<std::int64_t>(p.n_nodes) * p.dt_ns;
  ASSERT_TRUE(follower_.Sample(p, end - 1, s));
  EXPECT_FALSE(s.held);
  for (const std::int64_t t : {end, end + 1, end + 10 * p.dt_ns}) {
    ASSERT_TRUE(follower_.Sample(p, t, s));
    EXPECT_TRUE(s.held);
    for (int j = 0; j < p.nv; ++j) {
      EXPECT_EQ(s.q[static_cast<std::size_t>(j)], p.q[Idx(p.n_nodes, j)]);
      EXPECT_EQ(s.qd[static_cast<std::size_t>(j)], p.qd[Idx(p.n_nodes, j)]);
    }
  }
}

TEST_F(NodeFollowerTest, RefusesAShapeItCannotIndex) {
  const DecelPlanSnapshot good = MakePlan(arm_.q_nominal, 15);
  DecelNodeSample s{};
  const std::int64_t t = good.t0_ns + good.dt_ns;
  auto refused = [&](auto mutate) {
    DecelPlanSnapshot p = good;
    mutate(p);
    return !follower_.Sample(p, t, s);
  };
  EXPECT_TRUE(refused([](DecelPlanSnapshot& p) { p.nv = 7; }));  // not this arm
  EXPECT_TRUE(refused([](DecelPlanSnapshot& p) { p.n_nodes = 0; }));
  EXPECT_TRUE(refused([](DecelPlanSnapshot& p) { p.n_nodes = kMaxDecelNodes + 1; }));
  EXPECT_TRUE(refused([](DecelPlanSnapshot& p) { p.dt_ns = 0; }));
  NodeTrajectoryFollower uninit;
  EXPECT_FALSE(uninit.Sample(good, t, s));
}

TEST(NodeFollowerInit, RejectsUnusableBindings) {
  const Arm arm = RealArm6();
  NodeTrajectoryFollower f;
  EXPECT_FALSE(f.Init(nullptr, arm.frame, kDeviceOfModel));
  EXPECT_FALSE(
      f.Init(arm.model, static_cast<pinocchio::FrameIndex>(arm.model->nframes), kDeviceOfModel));
  const std::array<int, 6> dup{0, 1, 2, 3, 4, 4};
  EXPECT_FALSE(f.Init(arm.model, arm.frame, dup));
  const std::array<int, 6> out_of_range{0, 1, 2, 3, 4, 6};
  EXPECT_FALSE(f.Init(arm.model, arm.frame, out_of_range));
  const std::array<int, 5> short_map{0, 1, 2, 3, 4};
  EXPECT_FALSE(f.Init(arm.model, arm.frame, short_map));
  EXPECT_FALSE(f.Initialized());
  EXPECT_TRUE(f.Init(arm.model, arm.frame, kDeviceOfModel));
  EXPECT_TRUE(f.Initialized());
}

// ── 2. FK consistency (MD-30) ────────────────────────────────────────────────

TEST_F(NodeFollowerTest, FkConsistencyAlongTheSegment) {
  // T_d = T(q_ref) and V_ff = ᵂJ(q_ref)·q̇_ref, against an independent path:
  // a separate Data, placement from plain FK, and the twist from the frame
  // Jacobian (not velocity FK, which the follower uses).
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 21);
  pinocchio::Data data(*arm_.model);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, arm_.model->nv);
  DecelNodeSample s{};
  double worst_p = 0.0;
  double worst_r = 0.0;
  double worst_v = 0.0;
  const std::int64_t end = p.t0_ns + static_cast<std::int64_t>(p.n_nodes) * p.dt_ns;
  for (std::int64_t t = p.t0_ns; t <= end; t += 3'700'000) {  // off-grid instants
    ASSERT_TRUE(follower_.Sample(p, t, s));
    const Eigen::VectorXd q = ToModel(s.q, p.nv);
    const Eigen::VectorXd qd = ToModel(s.qd, p.nv);
    pinocchio::computeFrameJacobian(*arm_.model, data, q, arm_.frame,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    pinocchio::updateFramePlacement(*arm_.model, data, arm_.frame);
    const pinocchio::SE3& T = data.oMf[arm_.frame];
    worst_p = std::max(worst_p, (T.translation() - s.placement.translation()).norm());
    worst_r = std::max(worst_r, (T.rotation() - s.placement.rotation()).norm());
    worst_v = std::max(worst_v, (J * qd - s.twist).norm());
  }
  EXPECT_LT(worst_p, 1e-12);
  EXPECT_LT(worst_r, 1e-12);
  EXPECT_LT(worst_v, 1e-10);
}

TEST_F(NodeFollowerTest, NodesInsideBoxChecksEveryNodeInTheModelWorld) {
  // MD-43: the catch frame at node 0..N, against an independent FK of the
  // device-order nodes. A box around all of them passes; pulling one face in
  // past the extreme node refuses and names the FIRST node beyond it.
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 33);
  pinocchio::Data data(*arm_.model);
  std::vector<Eigen::Vector3d> pos;
  for (int k = 0; k <= p.n_nodes; ++k) {
    Eigen::VectorXd q(p.nv);
    for (int m = 0; m < p.nv; ++m) {
      q[m] = p.q[Idx(k, kDeviceOfModel[static_cast<std::size_t>(m)])];
    }
    pinocchio::framesForwardKinematics(*arm_.model, data, q);
    pos.push_back(data.oMf[arm_.frame].translation());
  }
  std::array<double, 3> lo{};
  std::array<double, 3> hi{};
  for (int a = 0; a < 3; ++a) {
    lo[static_cast<std::size_t>(a)] = std::numeric_limits<double>::infinity();
    hi[static_cast<std::size_t>(a)] = -std::numeric_limits<double>::infinity();
    for (const auto& x : pos) {
      lo[static_cast<std::size_t>(a)] = std::min(lo[static_cast<std::size_t>(a)], x[a]);
      hi[static_cast<std::size_t>(a)] = std::max(hi[static_cast<std::size_t>(a)], x[a]);
    }
  }
  int first = 99;
  EXPECT_TRUE(follower_.NodesInsideBox(p, lo, hi, nullptr, &first));
  EXPECT_EQ(first, -1);

  std::array<double, 3> tight = hi;
  tight[0] -= 1e-6;
  int expected = -1;
  for (int k = 0; k <= p.n_nodes && expected < 0; ++k) {
    if (pos[static_cast<std::size_t>(k)][0] > tight[0]) {
      expected = k;
    }
  }
  ASSERT_GE(expected, 0);
  EXPECT_FALSE(follower_.NodesInsideBox(p, lo, tight, nullptr, &first));
  EXPECT_EQ(first, expected);

  // Anchored: the same path moved so node 0 sits at `anchor`. Anchored at node
  // 0 itself it is the plain check; moved by d, the box moved by d passes and
  // the original box refuses the first node the shift pushes out.
  const std::array<double, 3> at_node0{pos[0].x(), pos[0].y(), pos[0].z()};
  EXPECT_TRUE(follower_.NodesInsideBox(p, lo, hi, &at_node0, &first));
  const Eigen::Vector3d d(0.5, -0.25, 0.125);
  const std::array<double, 3> shifted{pos[0].x() + d.x(), pos[0].y() + d.y(), pos[0].z() + d.z()};
  std::array<double, 3> lo_d{};
  std::array<double, 3> hi_d{};
  for (int a = 0; a < 3; ++a) {
    lo_d[static_cast<std::size_t>(a)] = lo[static_cast<std::size_t>(a)] + d[a];
    hi_d[static_cast<std::size_t>(a)] = hi[static_cast<std::size_t>(a)] + d[a];
  }
  EXPECT_TRUE(follower_.NodesInsideBox(p, lo_d, hi_d, &shifted, &first));
  EXPECT_EQ(first, -1);
  EXPECT_FALSE(follower_.NodesInsideBox(p, lo, hi, &shifted, &first));
  EXPECT_EQ(first, 0) << "node 0 itself is placed at the anchor, outside the unshifted box";

  DecelPlanSnapshot bad = p;
  bad.q[Idx(3, 0)] = kNan;  // node 3's FK is NaN: outside, whatever the box
  std::array<double, 3> huge_lo{-1e9, -1e9, -1e9};
  std::array<double, 3> huge_hi{1e9, 1e9, 1e9};
  EXPECT_FALSE(follower_.NodesInsideBox(bad, huge_lo, huge_hi, nullptr, &first));
  EXPECT_EQ(first, 3);

  NodeTrajectoryFollower unbound;
  EXPECT_FALSE(unbound.NodesInsideBox(p, huge_lo, huge_hi, nullptr, &first));
  EXPECT_EQ(first, -1);

  // NodePosition: the same FK, one node; out of range or unbound leaves `x`.
  for (int k : {0, 5, p.n_nodes}) {
    std::array<double, 3> x{};
    ASSERT_TRUE(follower_.NodePosition(p, k, x));
    for (int a = 0; a < 3; ++a) {
      EXPECT_NEAR(x[static_cast<std::size_t>(a)], pos[static_cast<std::size_t>(k)][a], 1e-12);
    }
  }
  std::array<double, 3> untouched{7.0, 7.0, 7.0};
  EXPECT_FALSE(follower_.NodePosition(p, p.n_nodes + 1, untouched));
  EXPECT_FALSE(follower_.NodePosition(p, -1, untouched));
  EXPECT_FALSE(unbound.NodePosition(p, 0, untouched));
  EXPECT_EQ(untouched[0], 7.0);
}

TEST_F(NodeFollowerTest, DeviceOrderIsMappedBeforeFk) {
  // Negative control for the mapping: FK of the DEVICE-order vector read as if
  // it were model order must differ — otherwise the permutation fixture would
  // not be testing anything.
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 22);
  DecelNodeSample s{};
  ASSERT_TRUE(follower_.Sample(p, p.t0_ns + 2 * p.dt_ns, s));
  pinocchio::Data data(*arm_.model);
  Eigen::VectorXd q_dev(p.nv);
  for (int j = 0; j < p.nv; ++j) {
    q_dev[j] = s.q[static_cast<std::size_t>(j)];
  }
  pinocchio::framesForwardKinematics(*arm_.model, data, q_dev);
  EXPECT_GT((data.oMf[arm_.frame].translation() - s.placement.translation()).norm(), 1e-3);
  pinocchio::framesForwardKinematics(*arm_.model, data, ToModel(s.q, p.nv));
  EXPECT_LT((data.oMf[arm_.frame].translation() - s.placement.translation()).norm(), 1e-12);
}

TEST_F(NodeFollowerTest, ClikResidualIsRecorded) {
  // Informational (MD-30): the CLIK's SE3 step at q_c = q_ref with the sampled
  // target and twist_ff. The hand task alone would return v* = q̇_ref; the
  // posture term k_a(q_des − q) has no velocity feedforward, so |v* − q̇_ref|
  // is what E1-F04's posture feedforward has to remove. Recorded, not bounded.
  const auto& model = arm_.model;
  rtc::tsid::PinocchioCache cache;
  rtc::tsid::ContactManagerConfig contact_cfg;
  contact_cfg.max_contacts = 0;
  cache.Init(model, rtc::tsid::ContactFrameIds(contact_cfg));
  const int tcp = cache.RegisterFrame("tool0", arm_.frame);
  ASSERT_GE(tcp, 0);
  rtc::tsid::ClikReferenceGenerator clik;
  rtc::tsid::ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2, 3, 4, 5};
  cfg.v_limit = 0.0;  // boxes off: the residual is the cost's, not a clamp's
  clik.Init(model->nv, cfg);
  clik.SetTaskGain(Eigen::Matrix<double, 6, 1>::Constant(5.0));
  clik.SetPostureGains(0.5, 0.0);

  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 23);
  DecelNodeSample s{};
  double worst = 0.0;
  double worst_rel = 0.0;
  int solved = 0;
  for (int k = 0; k < p.n_nodes; ++k) {
    ASSERT_TRUE(follower_.Sample(p, p.t0_ns + k * p.dt_ns + p.dt_ns / 2, s));
    const Eigen::VectorXd q = ToModel(s.q, p.nv);
    const Eigen::VectorXd qd = ToModel(s.qd, p.nv);
    cache.Update(q, qd);
    if (!clik.Compute(cache, tcp, -1, s.placement, q, 0.002, true, &s.twist)) {
      continue;
    }
    ++solved;
    const double r = (clik.VRef() - qd).norm();
    worst = std::max(worst, r);
    worst_rel = std::max(worst_rel, r / std::max(qd.norm(), 1e-9));
  }
  EXPECT_EQ(solved, p.n_nodes);
  EXPECT_TRUE(std::isfinite(worst));
  RecordProperty("clik_residual_max_radps", std::to_string(worst));
  RecordProperty("clik_residual_rel_max", std::to_string(worst_rel));
  std::printf("[ record ] CLIK |v* - qd_ref| max %.3e rad/s (relative %.3e)\n", worst, worst_rel);
}

// ── 3. RT: allocation and cost ───────────────────────────────────────────────

TEST_F(NodeFollowerTest, SampleAllocatesNothing) {
  const DecelPlanSnapshot p = MakePlan(arm_.q_nominal, 31);
  DecelNodeSample s{};
  ASSERT_TRUE(follower_.Sample(p, p.t0_ns, s));  // warm-up outside the gates
  {
    // Positive control: an allocation INSIDE pinocchio's shared object.
    rtc::testing::ScopedMallocGate gate;
    pinocchio::Data probe(*arm_.model);
    ASSERT_GT(gate.count(), 0U) << "the C-level gate does not see library allocations";
  }
  std::size_t news = 0;
  std::size_t mallocs = 0;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    for (int i = 0; i < 200; ++i) {
      const std::int64_t t = p.t0_ns + i * 2'000'000;  // 2 ms ticks, past the end too
      static_cast<void>(follower_.Sample(p, t, s));
    }
    news = new_gate.count();
    mallocs = malloc_gate.count();
  }
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(mallocs, 0U);
}

TEST_F(NodeFollowerTest, ContendedCopyNeverTearsAndIsRecorded) {
  // The RT's per-tick work once E1-F04 wires it: a whole-payload SeqLock copy
  // (D-21 — never gated on sequence()) plus one Sample, against a concurrent
  // planner store. Every stored payload is internally uniform (all q entries
  // derive from its seq), so a torn copy shows up as a non-uniform one. Two
  // writer cadences (G1-C form: the numbers are recorded, not bounded):
  //   • back-to-back stores — the strongest tearing probe. It also shows why
  //     the planner must never store in a loop: SeqLock::Load has no retry
  //     bound (L1 G1-8), and against a 4.9 KB back-to-back writer the reader
  //     completes only a few hundred copies in 300 ms on the development PC.
  //     So the read count is recorded here, and only required to be nonzero;
  //   • one store per millisecond — still 20× the planner's real rate (one per
  //     wake, ≥ 20 ms), i.e. the cost the tick actually pays with a retry.
  const DecelPlanSnapshot base = MakePlan(arm_.q_nominal, 32);
  auto run = [&](std::chrono::microseconds writer_pause, std::size_t min_reads, const char* label) {
    rtc::SeqLock<DecelPlanSnapshot> box;
    DecelPlanSnapshot first = base;
    box.Store(first);
    std::atomic<bool> stop{false};
    std::thread writer([&] {
      std::uint32_t seq = 2;
      DecelPlanSnapshot p = base;
      while (!stop.load(std::memory_order_relaxed)) {
        p.decel_seq = seq;
        for (int k = 0; k <= p.n_nodes; ++k) {
          for (int j = 0; j < p.nv; ++j) {
            p.q[Idx(k, j)] = static_cast<double>(seq % 1000) * 1e-3;
          }
        }
        box.Store(p);
        ++seq;
        if (writer_pause.count() > 0) {
          std::this_thread::sleep_for(writer_pause);
        }
      }
    });
    std::vector<std::int64_t> ns;
    ns.reserve(1U << 20);
    DecelPlanSnapshot copy{};
    DecelNodeSample s{};
    int torn = 0;
    int failed = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < deadline && ns.size() < ns.capacity()) {
      const auto t0 = std::chrono::steady_clock::now();
      box.LoadInto(copy);
      const bool ok = follower_.Sample(copy, copy.t0_ns + copy.dt_ns, s);
      const auto t1 = std::chrono::steady_clock::now();
      ns.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
      failed += ok ? 0 : 1;
      if (copy.decel_seq < 2) {
        continue;  // the initial store, before the writer's first
      }
      const double v = copy.q[0];
      bool uniform = true;
      for (int k = 0; k <= copy.n_nodes && uniform; ++k) {
        for (int j = 0; j < copy.nv; ++j) {
          uniform = uniform && copy.q[Idx(k, j)] == v;
        }
      }
      torn += uniform && v == static_cast<double>(copy.decel_seq % 1000) * 1e-3 ? 0 : 1;
    }
    stop.store(true);
    writer.join();
    EXPECT_EQ(torn, 0) << label;
    EXPECT_EQ(failed, 0) << label;
    ASSERT_GE(ns.size(), min_reads) << label << ": too few reads to mean anything";
    std::sort(ns.begin(), ns.end());
    const std::int64_t p99 = ns[ns.size() * 99 / 100];
    const std::int64_t worst = ns.back();
    RecordProperty(std::string("copy_sample_p99_ns_") + label, std::to_string(p99));
    RecordProperty(std::string("copy_sample_worst_ns_") + label, std::to_string(worst));
    RecordProperty(std::string("copy_sample_reads_") + label, std::to_string(ns.size()));
    std::printf("[ record ] %s: copy (%zu B) + Sample p99 %lld ns, worst %lld ns (%zu reads)\n",
                label, sizeof(DecelPlanSnapshot), static_cast<long long>(p99),
                static_cast<long long>(worst), ns.size());
  };
  run(std::chrono::microseconds(0), 1U, "back_to_back");
  run(std::chrono::microseconds(1000), 1000U, "writer_1khz");
}

}  // namespace
