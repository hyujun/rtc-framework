/// @file test_policy_sampler.cpp
/// @brief The manager's sampler (`SampleReference` / `ComputeReference`,
///        one kernel) on a fixed and a floating base.
///
/// - Time axis: node k of a plan sits at `observation − horizon_offset + k·dt`
///   in the consumer clock, integer nanoseconds — node times land on the
///   node bit for bit, random times on the floor interval, an observation
///   stamp below the offset does not wrap; policy age = receive − observation;
///   `max_policy_age_s`.
/// - Floating kernel: quaternion slerp against an independent formula, unit
///   norm; base position on a constant world velocity; joint rows bit equal to
///   the fixed-base path; wrench and mask held per interval and masked by the
///   node mode, the last interval at the full horizon read from row H − 1.
/// - Riccati on a floating base (public G1): the feedback against `K·Δx` with
///   an independently built log map, the nearest gain node and its clamp, the
///   per-block caps one block at a time, the fade before a rotation error of
///   π, the accel-row product against the full one; config refusals.
/// - A non-finite sample or state short-circuits; the two public paths share
///   one plan; 1000 ticks (installs included) allocate nothing.

#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/feedback/riccati_feedback.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/types/wbc_reference.hpp"
#include "test_utils/g1_fixture.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace rtc::mpc {
namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();
constexpr double kPi = 3.14159265358979323846;
constexpr int kH = 8;
constexpr double kDt = 0.035;
constexpr int64_t kDtNs = 35'000'000;
constexpr int kNj = 3;  // joints of the synthetic floating layout
constexpr uint64_t kObs = 5'000'000'000ULL;
constexpr int64_t kOffset = 12'345'678;
constexpr int64_t kOrigin = static_cast<int64_t>(kObs) - kOffset;

/// A floating layout without a model: free-flyer + kNj joints, two 6-D
/// contacts, kinodynamic input.
WbLayout FloatingLayout() {
  WbLayout l;
  l.nq = kFreeFlyerNq + kNj;
  l.nv = kFreeFlyerNv + kNj;
  l.ndx = 2 * l.nv;
  l.n_base_q = kFreeFlyerNq;
  l.n_base_v = kFreeFlyerNv;
  l.n_contacts = 2;
  l.contact_dim[0] = 6;
  l.contact_dim[1] = 6;
  l.contact_u_offset[0] = 0;
  l.contact_u_offset[1] = 6;
  l.n_contact_vars = 12;
  l.n_wrench_u = 12;
  l.acc_u_offset = 12;
  l.n_acc = kNj;
  l.nu = 12 + kNj;
  return l;
}

const Eigen::Vector3d kWorldVel(0.4, -0.1, 0.05);
const Eigen::Vector3d kAxis = Eigen::Vector3d(1.0, 2.0, -0.5).normalized();
constexpr double kTurnPerNode = 0.05;  // rad

Eigen::Quaterniond NodeOrientation(int k) {
  const Eigen::Quaterniond r0 = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
  return (r0 * Eigen::AngleAxisd(kTurnPerNode * k, kAxis)).normalized();
}

/// A floating plan: the base turns about a body axis at a constant rate and
/// moves at a constant world velocity; joints, wrenches, modes and the
/// interval accelerations are distinct per node. No gain.
MPCSolution FloatingPlan(int horizon = kH) {
  const WbLayout l = FloatingLayout();
  MPCSolution s{};
  s.horizon_length = horizon;
  s.dt_node = kDt;
  MarkSolved(s, true);
  s.nq = l.nq;
  s.nv = l.nv;
  s.nu = l.nu;
  s.nx = l.nq + l.nv;
  s.ndx = l.ndx;
  s.n_contact_vars = l.n_contact_vars;
  s.observation_stamp_ns = kObs;
  s.horizon_offset_ns = kOffset;
  for (int k = 0; k <= horizon; ++k) {
    const auto kk = static_cast<std::size_t>(k);
    const Eigen::Quaterniond r = NodeOrientation(k);
    const Eigen::Vector3d p = Eigen::Vector3d(0.1, 0.2, 0.75) + kWorldVel * (kDt * k);
    const Eigen::Vector3d v_lin = r.toRotationMatrix().transpose() * kWorldVel;
    const Eigen::Vector3d w = kAxis * (kTurnPerNode / kDt);
    for (int j = 0; j < 3; ++j) {
      s.q_traj[kk][static_cast<std::size_t>(j)] = p[j];
      s.v_traj[kk][static_cast<std::size_t>(j)] = v_lin[j];
      s.v_traj[kk][static_cast<std::size_t>(3 + j)] = w[j];
    }
    s.q_traj[kk][3] = r.x();
    s.q_traj[kk][4] = r.y();
    s.q_traj[kk][5] = r.z();
    s.q_traj[kk][6] = r.w();
    for (int i = 0; i < kNj; ++i) {
      s.q_traj[kk][static_cast<std::size_t>(7 + i)] = std::sin(0.3 * k + i);
      s.v_traj[kk][static_cast<std::size_t>(6 + i)] = std::cos(0.2 * k - i);
    }
    s.node_mode[kk] = static_cast<uint32_t>(k % 3 + 1);  // 0b01, 0b10, 0b11
  }
  for (int k = 0; k < horizon; ++k) {
    const auto kk = static_cast<std::size_t>(k);
    for (int i = 0; i < l.n_contact_vars; ++i) {
      s.lambda_traj[kk][static_cast<std::size_t>(i)] = 100.0 + 10.0 * k + i;
    }
    for (int i = 0; i < l.nv; ++i) {
      s.a_ff_traj[kk][static_cast<std::size_t>(i)] = 0.01 * (k + 1) + i;
    }
  }
  return s;
}

/// The joint rows of @p f as a fixed-base plan (same values, offsets 0).
MPCSolution FixedFromFloating(const MPCSolution& f) {
  MPCSolution s = f;
  s.nq = kNj;
  s.nv = kNj;
  s.nu = kNj;
  s.nx = 2 * kNj;
  s.ndx = 2 * kNj;
  s.n_contact_vars = 0;
  for (int k = 0; k <= f.horizon_length; ++k) {
    const auto kk = static_cast<std::size_t>(k);
    for (int i = 0; i < kNj; ++i) {
      s.q_traj[kk][static_cast<std::size_t>(i)] = f.q_traj[kk][static_cast<std::size_t>(7 + i)];
      s.v_traj[kk][static_cast<std::size_t>(i)] = f.v_traj[kk][static_cast<std::size_t>(6 + i)];
    }
  }
  return s;
}

YAML::Node Config(double max_age_s = 0.0) {
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 1 << 30;
  if (max_age_s > 0.0) {
    cfg["max_policy_age_s"] = max_age_s;
  }
  return cfg;
}

uint64_t At(int64_t t) {
  return static_cast<uint64_t>(t);
}

class SamplerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    layout_ = FloatingLayout();
    mgr_.Init(Config(), layout_);
    q_.setZero(layout_.nq);
    q_[6] = 1.0;
    v_.setZero(layout_.nv);
  }

  bool Sample(int64_t now) { return mgr_.SampleReference(q_, v_, At(now), ref_); }

  WbLayout layout_;
  MPCSolutionManager mgr_;
  Eigen::VectorXd q_, v_;
  WbcReference ref_{};
};

// ── Time axis ──────────────────────────────────────────────────────────────

TEST_F(SamplerTest, NodeTimesLandOnTheNodesBitForBit) {
  const MPCSolution plan = FloatingPlan();
  mgr_.PublishSolution(plan);
  for (int k = 0; k <= kH; ++k) {
    ASSERT_TRUE(Sample(kOrigin + k * kDtNs)) << "node " << k;
    const auto kk = static_cast<std::size_t>(k);
    for (int i = 0; i < layout_.nq; ++i) {
      EXPECT_EQ(ref_.q_ref[static_cast<std::size_t>(i)],
                plan.q_traj[kk][static_cast<std::size_t>(i)])
          << "node " << k << " q " << i;
    }
    for (int i = 0; i < layout_.nv; ++i) {
      EXPECT_EQ(ref_.v_ref[static_cast<std::size_t>(i)],
                plan.v_traj[kk][static_cast<std::size_t>(i)])
          << "node " << k << " v " << i;
    }
    EXPECT_EQ(ref_.beyond_horizon, k == kH);
  }
  for (int i = 0; i < layout_.nv; ++i) {
    EXPECT_EQ(ref_.a_ref[static_cast<std::size_t>(i)], 0.0) << "held on the last node";
  }
  EXPECT_EQ(ref_.progress, 1.0);
}

// node_mode[k] = k % 3 + 1 names the interval; 1000 seeded times plus the
// boundaries either side of every node land on ⌊(now − origin) / dt⌋.
TEST_F(SamplerTest, AnyTimeLandsOnTheFloorInterval) {
  MPCSolution plan = FloatingPlan();
  for (int k = 0; k <= kH; ++k) {
    plan.node_mode[static_cast<std::size_t>(k)] = static_cast<uint32_t>(k + 1) << 2;
  }
  mgr_.PublishSolution(plan);
  std::vector<int64_t> taus;
  std::mt19937_64 rng(20261114);
  std::uniform_int_distribution<int64_t> pick(-kDtNs, (kH + 1) * kDtNs - kOffset);
  for (int i = 0; i < 1000; ++i) {
    taus.push_back(pick(rng));
  }
  for (int k = 0; k <= kH; ++k) {
    for (const int64_t d : {int64_t{-1}, int64_t{0}, int64_t{1}}) {
      taus.push_back(k * kDtNs + d);
    }
  }
  for (const int64_t tau : taus) {
    ASSERT_TRUE(Sample(kOrigin + tau)) << "tau " << tau;
    const int64_t clamped = std::max<int64_t>(0, tau);
    const bool beyond = clamped >= kH * kDtNs;
    const int k = beyond ? kH - 1 : static_cast<int>(clamped / kDtNs);
    EXPECT_EQ(ref_.contact_mask, static_cast<uint32_t>(k + 1) << 2) << "tau " << tau;
    EXPECT_EQ(ref_.beyond_horizon, beyond) << "tau " << tau;
  }
}

// An observation stamp below the offset puts node 0 before the clock's zero:
// signed arithmetic, node 0's interval, no wrap.
TEST_F(SamplerTest, AnObservationBelowTheOffsetDoesNotWrap) {
  MPCSolution plan = FloatingPlan();
  plan.observation_stamp_ns = 55;
  plan.horizon_offset_ns = 1'000'000;
  plan.node_mode[0] = 0x40;
  mgr_.PublishSolution(plan);
  ASSERT_TRUE(Sample(60));
  EXPECT_EQ(ref_.contact_mask, 0x40u);
  EXPECT_FALSE(ref_.beyond_horizon);
}

TEST_F(SamplerTest, PolicyAgeIsTheReceiveStampMinusTheObservation) {
  mgr_.PublishSolution(FloatingPlan());
  ASSERT_TRUE(Sample(static_cast<int64_t>(kObs) + 3'141'592));
  EXPECT_EQ(ref_.receive_stamp_ns - ref_.observation_stamp_ns, 3'141'592u);
  EXPECT_EQ(mgr_.ActivePolicyAgeNs(), 3'141'592);
  ASSERT_TRUE(Sample(static_cast<int64_t>(kObs) + 9'000'000));
  EXPECT_EQ(mgr_.ActivePolicyAgeNs(), 3'141'592) << "the receive stamp is the acceptance's";
}

TEST_F(SamplerTest, AMaxPolicyAgeDropsAnOlderPlan) {
  mgr_.Init(Config(0.05), layout_);
  ASSERT_TRUE(mgr_.ConfigValid());
  mgr_.PublishSolution(FloatingPlan());
  EXPECT_TRUE(Sample(static_cast<int64_t>(kObs) + 50'000'000));
  EXPECT_FALSE(Sample(static_cast<int64_t>(kObs) + 50'000'001));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 1u);

  // Pass control: off (0), the same age samples.
  mgr_.Init(Config(), layout_);
  mgr_.PublishSolution(FloatingPlan());
  EXPECT_TRUE(Sample(static_cast<int64_t>(kObs) + 50'000'001));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 0u);
}

// ── Floating kernel ────────────────────────────────────────────────────────

/// The slerp of two unit quaternions written out (no acos): Ω from the
/// chord lengths, the two sine weights.
Eigen::Vector4d ReferenceSlerp(const Eigen::Vector4d& a, const Eigen::Vector4d& b, double t) {
  const double omega = 2.0 * std::atan2((a - b).norm(), (a + b).norm());
  const double s = std::sin(omega);
  return (std::sin((1.0 - t) * omega) / s) * a + (std::sin(t * omega) / s) * b;
}

TEST_F(SamplerTest, TheBaseQuaternionIsTheSlerpOfItsNodes) {
  const MPCSolution plan = FloatingPlan();
  mgr_.PublishSolution(plan);
  std::mt19937_64 rng(20261114);
  std::uniform_int_distribution<int64_t> pick(1, kH * kDtNs - 1);
  for (int i = 0; i < 200; ++i) {
    const int64_t tau = pick(rng);
    if (tau % kDtNs == 0) {
      continue;
    }
    ASSERT_TRUE(Sample(kOrigin + tau));
    const int k = static_cast<int>(tau / kDtNs);
    const double alpha = static_cast<double>(tau % kDtNs) / static_cast<double>(kDtNs);
    const auto kk = static_cast<std::size_t>(k);
    const Eigen::Vector4d a(plan.q_traj[kk][3], plan.q_traj[kk][4], plan.q_traj[kk][5],
                            plan.q_traj[kk][6]);
    const Eigen::Vector4d b(plan.q_traj[kk + 1][3], plan.q_traj[kk + 1][4], plan.q_traj[kk + 1][5],
                            plan.q_traj[kk + 1][6]);
    const Eigen::Vector4d want = ReferenceSlerp(a, b, alpha);
    const Eigen::Vector4d got(ref_.q_ref[3], ref_.q_ref[4], ref_.q_ref[5], ref_.q_ref[6]);
    EXPECT_LE((got - want).norm(), 1e-12) << "tau " << tau;
    EXPECT_LE(std::abs(got.norm() - 1.0), 1e-12) << "tau " << tau;
    EXPECT_GE(got.dot(a), 0.0) << "the short way";
  }
}

// The base moves at a constant world velocity while it turns: the node LOCAL
// linear velocities rotated to world are that velocity, so the Hermite
// position is the straight line.
TEST_F(SamplerTest, TheBasePositionFollowsAConstantWorldVelocity) {
  const MPCSolution plan = FloatingPlan();
  mgr_.PublishSolution(plan);
  for (int64_t tau = 0; tau < kH * kDtNs; tau += 1'234'567) {
    ASSERT_TRUE(Sample(kOrigin + tau));
    const double t = static_cast<double>(tau) * 1e-9;
    const Eigen::Vector3d want = Eigen::Vector3d(0.1, 0.2, 0.75) + kWorldVel * t;
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(ref_.q_ref[static_cast<std::size_t>(j)], want[j], 1e-12) << "tau " << tau;
    }
  }
}

// A base whose LOCAL linear velocity and angular rate change node to node:
// the position is the cubic Hermite of the node positions with the world
// slopes R_k · v_lin_k, written out here.
TEST_F(SamplerTest, TheBasePositionIsTheHermiteOfTheWorldSlopes) {
  MPCSolution plan = FloatingPlan();
  for (int k = 0; k <= kH; ++k) {
    const auto kk = static_cast<std::size_t>(k);
    for (int j = 0; j < 6; ++j) {
      plan.v_traj[kk][static_cast<std::size_t>(j)] += 0.1 * std::sin(0.7 * k + j);
    }
  }
  mgr_.PublishSolution(plan);
  for (int64_t tau = 1; tau < kH * kDtNs; tau += 2'345'671) {
    ASSERT_TRUE(Sample(kOrigin + tau));
    const int k = static_cast<int>(tau / kDtNs);
    const double s = static_cast<double>(tau % kDtNs) / static_cast<double>(kDtNs);
    const auto kk = static_cast<std::size_t>(k);
    const auto slope = [&](std::size_t n) {
      const Eigen::Quaterniond r(plan.q_traj[n][6], plan.q_traj[n][3], plan.q_traj[n][4],
                                 plan.q_traj[n][5]);
      return Eigen::Vector3d(r.toRotationMatrix() * Eigen::Vector3d(plan.v_traj[n][0],
                                                                    plan.v_traj[n][1],
                                                                    plan.v_traj[n][2]));
    };
    const Eigen::Vector3d m0 = slope(kk);
    const Eigen::Vector3d m1 = slope(kk + 1);
    for (int j = 0; j < 3; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      const double want = (2 * s * s * s - 3 * s * s + 1) * plan.q_traj[kk][jj] +
                          (s * s * s - 2 * s * s + s) * kDt * m0[j] +
                          (-2 * s * s * s + 3 * s * s) * plan.q_traj[kk + 1][jj] +
                          (s * s * s - s * s) * kDt * m1[j];
      EXPECT_NEAR(ref_.q_ref[jj], want, 1e-12) << "tau " << tau;
    }
  }
}

// Consecutive node quaternions on opposite hemispheres (a producer that did
// not canonicalise): the slerp still takes the short way and stays unit.
TEST_F(SamplerTest, AnOppositeHemispherePairStillTakesTheShortWay) {
  MPCSolution plan = FloatingPlan();
  for (std::size_t j = 3; j < 7; ++j) {
    plan.q_traj[3][j] = -plan.q_traj[3][j];
  }
  mgr_.PublishSolution(plan);
  ASSERT_TRUE(Sample(kOrigin + 2 * kDtNs + kDtNs / 2));
  const Eigen::Vector4d a(plan.q_traj[2][3], plan.q_traj[2][4], plan.q_traj[2][5],
                          plan.q_traj[2][6]);
  const Eigen::Vector4d b(-plan.q_traj[3][3], -plan.q_traj[3][4], -plan.q_traj[3][5],
                          -plan.q_traj[3][6]);  // the stored node, back on a's hemisphere
  const Eigen::Vector4d want = ReferenceSlerp(a, b, 0.5);
  const Eigen::Vector4d got(ref_.q_ref[3], ref_.q_ref[4], ref_.q_ref[5], ref_.q_ref[6]);
  EXPECT_LE(std::abs(got.norm() - 1.0), 1e-12);
  EXPECT_LE(std::min((got - want).norm(), (got + want).norm()), 1e-12) << "the short-way rotation";
  EXPECT_GE(got.dot(a), 0.0);
}

// A node spacing whose horizon would not fit an int64 of nanoseconds is
// refused, not sampled with an overflowed horizon.
TEST_F(SamplerTest, ANodeSpacingTooLargeForTheClockIsRefused) {
  MPCSolution plan = FloatingPlan();
  plan.dt_node = 2e8;  // s
  mgr_.PublishSolution(plan);
  EXPECT_FALSE(Sample(kOrigin));
}

// The joint rows of a floating plan come out of the same arithmetic as a
// fixed-base plan with the same joint values.
TEST_F(SamplerTest, JointRowsAreBitEqualToTheFixedBasePath) {
  const MPCSolution floating = FloatingPlan();
  MPCSolutionManager fixed;
  fixed.Init(Config(), WbLayout::FixedBase(kNj, kNj, 0));
  mgr_.PublishSolution(floating);
  fixed.PublishSolution(FixedFromFloating(floating));
  const Eigen::VectorXd qf = Eigen::VectorXd::Zero(kNj);
  const Eigen::VectorXd vf = Eigen::VectorXd::Zero(kNj);
  WbcReference rf{};
  for (int64_t tau = -kDtNs / 2; tau < (kH + 1) * kDtNs - kOffset; tau += 3'333'333) {
    ASSERT_TRUE(Sample(kOrigin + tau));
    ASSERT_TRUE(fixed.SampleReference(qf, vf, At(kOrigin + tau), rf));
    for (int i = 0; i < kNj; ++i) {
      const auto iq = static_cast<std::size_t>(7 + i);
      const auto iv = static_cast<std::size_t>(6 + i);
      const auto f = static_cast<std::size_t>(i);
      EXPECT_EQ(ref_.q_ref[iq], rf.q_ref[f]) << "tau " << tau;
      EXPECT_EQ(ref_.v_ref[iv], rf.v_ref[f]) << "tau " << tau;
      EXPECT_EQ(ref_.a_ref[iv], rf.a_ref[f]) << "tau " << tau;
    }
  }
}

// Base rate consistency (report): the central difference of the sampled
// base position against R · v_lin of the sample — they differ by O(dt · a)
// (the twist is interpolated linearly, the position by Hermite).
TEST_F(SamplerTest, ReportsTheBaseRateConsistency) {
  mgr_.PublishSolution(FloatingPlan());
  constexpr int64_t kH_ns = 100'000;
  double worst = 0.0;
  for (int64_t tau = kDtNs / 3; tau < (kH - 1) * kDtNs; tau += 7'777'777) {
    ASSERT_TRUE(Sample(kOrigin + tau + kH_ns));
    const Eigen::Vector3d p1(ref_.q_ref[0], ref_.q_ref[1], ref_.q_ref[2]);
    ASSERT_TRUE(Sample(kOrigin + tau - kH_ns));
    const Eigen::Vector3d p0(ref_.q_ref[0], ref_.q_ref[1], ref_.q_ref[2]);
    ASSERT_TRUE(Sample(kOrigin + tau));
    const Eigen::Quaterniond r(ref_.q_ref[6], ref_.q_ref[3], ref_.q_ref[4], ref_.q_ref[5]);
    const Eigen::Vector3d rv = r * Eigen::Vector3d(ref_.v_ref[0], ref_.v_ref[1], ref_.v_ref[2]);
    worst = std::max(worst, ((p1 - p0) / (2e-9 * kH_ns) - rv).norm());
  }
  RecordProperty("base_rate_mismatch_mps", std::to_string(worst));
  std::printf("[sampler] base rate: max |dp/dt - R v_lin| = %.3e m/s\n", worst);
  EXPECT_LT(worst, 1e-3);
}

// ── Wrench and contact mask ────────────────────────────────────────────────

TEST_F(SamplerTest, TheWrenchIsHeldAndMaskedByTheIntervalsMode) {
  const MPCSolution plan = FloatingPlan();
  mgr_.PublishSolution(plan);
  for (int k = 0; k <= kH; ++k) {
    for (const int64_t d : {int64_t{0}, kDtNs / 2, kDtNs - 1}) {
      if (k == kH && d != 0) {
        continue;
      }
      ASSERT_TRUE(Sample(kOrigin + k * kDtNs + d));
      const int kw = std::min(k, kH - 1);
      const auto kk = static_cast<std::size_t>(kw);
      EXPECT_EQ(ref_.contact_mask, plan.node_mode[kk]) << "node " << k;
      for (int c = 0; c < 2; ++c) {
        const bool active = ((plan.node_mode[kk] >> c) & 1U) != 0U;
        for (int i = 6 * c; i < 6 * c + 6; ++i) {
          const auto ii = static_cast<std::size_t>(i);
          EXPECT_EQ(ref_.wrench_ref[ii], active ? plan.lambda_traj[kk][ii] : 0.0)
              << "node " << k << " slot " << i;
        }
      }
      // The base acceleration is the interval's mean, held.
      for (int i = 0; i < 6; ++i) {
        const auto ii = static_cast<std::size_t>(i);
        EXPECT_EQ(ref_.a_ref[ii], k == kH ? 0.0 : plan.a_ff_traj[kk][ii]) << "node " << k;
      }
    }
  }
}

// At the full horizon the last interval reads row H − 1 of the H-row arrays
// (the old kernel read row H, past the end); the fixed-base Interpolate path
// too.
TEST_F(SamplerTest, TheLastIntervalAtTheFullHorizonReadsRowHMinusOne) {
  MPCSolution plan = FloatingPlan(kMaxHorizon);
  mgr_.PublishSolution(plan);
  ASSERT_TRUE(Sample(kOrigin + (kMaxHorizon - 1) * kDtNs + kDtNs / 2));
  const auto last = static_cast<std::size_t>(kMaxHorizon - 1);
  const uint32_t mode = plan.node_mode[last];
  EXPECT_EQ(ref_.contact_mask, mode);
  for (int i = 0; i < 12; ++i) {
    const auto ii = static_cast<std::size_t>(i);
    const bool active = ((mode >> (i / 6)) & 1U) != 0U;
    EXPECT_EQ(ref_.wrench_ref[ii], active ? plan.lambda_traj[last][ii] : 0.0) << "slot " << i;
  }

  TrajectoryInterpolator interp;
  interp.Init(kNj, kNj, 12);
  MPCSolution fixed = FixedFromFloating(plan);
  fixed.n_contact_vars = 12;
  ASSERT_TRUE(interp.SetSolution(fixed, 0));
  Eigen::VectorXd q(kNj), v(kNj), a(kNj), lambda(12);
  InterpMeta meta{};
  interp.Interpolate(At((kMaxHorizon - 1) * kDtNs + kDtNs / 2), q, v, a, lambda, meta);
  ASSERT_TRUE(meta.valid);
  for (int i = 0; i < 12; ++i) {
    EXPECT_EQ(lambda[i], plan.lambda_traj[last][static_cast<std::size_t>(i)]) << "slot " << i;
  }
}

// ── Short circuits and the two public paths ───────────────────────────────

TEST_F(SamplerTest, ANonFiniteSampleIsRefusedAndTheRestStillSamples) {
  MPCSolution plan = FloatingPlan();
  plan.q_traj[3][8] = std::numeric_limits<double>::quiet_NaN();
  mgr_.PublishSolution(plan);
  EXPECT_FALSE(Sample(kOrigin + 2 * kDtNs + kDtNs / 2)) << "interval 2 reads node 3";
  EXPECT_FALSE(ref_.valid);
  EXPECT_TRUE(Sample(kOrigin + 5 * kDtNs + kDtNs / 2)) << "pass control: interval 5";
}

TEST_F(SamplerTest, BothPublicPathsSampleTheOnePlan) {
  MPCSolution plan = FloatingPlan();
  mgr_.PublishSolution(plan);
  Eigen::VectorXd q_ref(layout_.nq), v_ref(layout_.nv), a_ff(layout_.nv), lambda(12),
      u_fb(layout_.nv);
  InterpMeta meta{};
  ASSERT_TRUE(
      mgr_.ComputeReference(q_, v_, At(kOrigin + kDtNs), q_ref, v_ref, a_ff, lambda, u_fb, meta));
  ASSERT_TRUE(Sample(kOrigin + kDtNs)) << "the plan ComputeReference took is held for both";
  EXPECT_EQ(ref_.plan_id, mgr_.ActivePlanId());
  for (int i = 0; i < layout_.nq; ++i) {
    EXPECT_EQ(ref_.q_ref[static_cast<std::size_t>(i)], q_ref[i]);
  }
  mgr_.PublishSolution(plan);
  ASSERT_TRUE(Sample(kOrigin + 2 * kDtNs));
  const uint64_t taken = ref_.plan_id;
  ASSERT_TRUE(mgr_.ComputeReference(q_, v_, At(kOrigin + 2 * kDtNs), q_ref, v_ref, a_ff, lambda,
                                    u_fb, meta));
  EXPECT_EQ(mgr_.ActivePlanId(), taken) << "a plan taken by one path is the other's";
}

// A plan that claims more gain nodes, or a larger gain, than the payload
// holds is refused as a dimension mismatch.
TEST_F(SamplerTest, APlanBeyondTheGainCapacityIsRefused) {
  MPCSolution plan = FloatingPlan();
  plan.gain_nodes = kMaxGainNodes + 1;
  mgr_.PublishSolution(plan);
  EXPECT_FALSE(Sample(kOrigin));
  EXPECT_EQ(mgr_.DimensionRejections(), 1u);
  plan.gain_nodes = 0;  // pass control
  mgr_.PublishSolution(plan);
  EXPECT_TRUE(Sample(kOrigin));
}

// A configuration Init refused stays off: SetEnabled(true) does not turn it on.
TEST(SamplerConfig, ARefusedConfigCannotBeReEnabled) {
  MPCSolutionManager mgr;
  mgr.Init(YAML::Load("{enabled: true, max_policy_age_s: -1.0}"), FloatingLayout());
  ASSERT_FALSE(mgr.ConfigValid());
  mgr.SetEnabled(true);
  EXPECT_FALSE(mgr.Enabled());
  mgr.Init(YAML::Load("{enabled: false}"), FloatingLayout());  // pass control
  mgr.SetEnabled(true);
  EXPECT_TRUE(mgr.Enabled());
}

// A re-Init clears what the earlier one set: the block caps of a floating
// layout do not stay on the Riccati of a fixed one, and an interpolator
// whose re-Init was refused samples nothing.
TEST(SamplerReInit, ReInitClearsTheBlockCapsAndARefusedInterpolatorSamplesNothing) {
  RiccatiFeedback fb;
  fb.Init(9, 15, 18);
  RiccatiBlockCaps caps{};
  caps.fill(1e-3);
  ASSERT_TRUE(fb.SetBlockCaps(caps, 9, true));
  fb.Init(9, 9, 18);
  fb.SetMaxDeltaXNorm(10.0);
  std::vector<double> K(9 * 18, 1.0);
  fb.SetGain(K.data(), 9, 18);
  Eigen::VectorXd dx = Eigen::VectorXd::Constant(18, 0.01);
  Eigen::VectorXd u(9);
  ASSERT_TRUE(fb.ComputeFromDelta(dx, u));
  EXPECT_EQ(fb.LastScale(), 1.0) << "the single norm cap of the new Init, not the old block caps";

  TrajectoryInterpolator interp;
  interp.Init(kNj, kNj, 0);
  interp.Init(kMaxNq + 1, kNj, 0);
  (void)interp.SetSolution(FixedFromFloating(FloatingPlan()), 0);
  double q[kNj] = {7.0, 7.0, 7.0};
  double v[kNj] = {};
  double a[kNj] = {};
  uint32_t mask = 0;
  EXPECT_FALSE(interp.Sample(0, WbLayout::FixedBase(kNj, kNj, 0), q, v, a, nullptr, mask).valid);
  EXPECT_EQ(q[0], 7.0) << "nothing written";
}

TEST(SamplerProvenance, TheReferenceCarriesThePlansProvenance) {
  MPCSolutionManager mgr;
  const WbLayout l = FloatingLayout();
  mgr.Init(Config(), l);
  MPCSolution plan = FloatingPlan();
  plan.observation_sequence = 41;
  plan.observation_source_sequence = 0xABCDEF0123ULL;
  plan.model_epoch = 0;
  plan.state_epoch = 0;
  plan.schedule_id = 0x5555AAAA5555AAAAULL;
  plan.schedule_epoch = 9;
  mgr.PublishSolution(plan);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(l.nq), v = Eigen::VectorXd::Zero(l.nv);
  WbcReference ref{};
  ASSERT_TRUE(mgr.SampleReference(q, v, At(kOrigin), ref));
  EXPECT_EQ(ref.observation_stamp_ns, kObs);
  EXPECT_EQ(ref.observation_sequence, 41u);
  EXPECT_EQ(ref.observation_source_sequence, 0xABCDEF0123ULL);
  EXPECT_EQ(ref.schedule_id, 0x5555AAAA5555AAAAULL);
  EXPECT_EQ(ref.schedule_epoch, 9u);
  EXPECT_EQ(ref.plan_id, 1u);
  EXPECT_EQ(ref.receive_stamp_ns, At(kOrigin));
  EXPECT_EQ(ref.nq, l.nq);
  EXPECT_EQ(ref.nv, l.nv);
  EXPECT_EQ(ref.n_contact_vars, 12);
  EXPECT_TRUE(ref.valid);
}

// ── Riccati on a floating base (public G1) ─────────────────────────────────

class FloatingRiccatiTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(test_utils::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << test_utils::kG1Urdf;
    }
    model_ = test_utils::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(test_utils::kG1SolesYaml)),
              RobotModelInitError::kNoError);
    layout_ = WbLayout::FromHandler(handler_);
    q_ref_ = pinocchio::neutral(model_);
    q_ref_[2] = 0.75;
    const Eigen::Quaterniond r = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
    q_ref_.segment<4>(3) = r.coeffs();  // xyzw
    v_ref_ = Eigen::VectorXd::Zero(model_.nv);
    std::mt19937_64 rng(20261114);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int g = 0; g < kMaxGainNodes; ++g) {
      K_[static_cast<std::size_t>(g)] = Eigen::MatrixXd(layout_.nu, layout_.ndx);
      for (int i = 0; i < layout_.nu; ++i) {
        for (int j = 0; j < layout_.ndx; ++j) {
          K_[static_cast<std::size_t>(g)](i, j) = u(rng);
        }
      }
    }
  }

  /// A plan at rest at q_ref_ with kMaxGainNodes distinct gains.
  MPCSolution Plan() const {
    MPCSolution s{};
    s.horizon_length = kH;
    s.dt_node = kDt;
    MarkSolved(s, true);
    s.nq = layout_.nq;
    s.nv = layout_.nv;
    s.nu = layout_.nu;
    s.nx = layout_.nq + layout_.nv;
    s.ndx = layout_.ndx;
    s.n_contact_vars = layout_.n_contact_vars;
    s.observation_stamp_ns = kObs;
    s.gain_nodes = kMaxGainNodes;
    for (int k = 0; k <= kH; ++k) {
      for (int i = 0; i < layout_.nq; ++i) {
        s.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = q_ref_[i];
      }
    }
    for (int g = 0; g < kMaxGainNodes; ++g) {
      EXPECT_TRUE(PackRiccatiGainRowMajor(K_[static_cast<std::size_t>(g)],
                                          s.K_riccati[static_cast<std::size_t>(g)].data(),
                                          s.K_riccati[static_cast<std::size_t>(g)].size()));
    }
    return s;
  }

  void Init(const std::string& riccati_yaml = "") {
    YAML::Node cfg = Config();
    cfg["riccati"] =
        riccati_yaml.empty() ? YAML::Load("{max_delta_x_norm: 1.0e6}") : YAML::Load(riccati_yaml);
    mgr_.Init(cfg, layout_, &model_);
    ASSERT_TRUE(mgr_.ConfigValid());
    mgr_.PublishSolution(Plan());
  }

  /// `Δx` built by hand: the free-flyer block from the SE(3) log of
  /// M_ref⁻¹·M_cur (pinocchio's log6 on placements built here), the joints
  /// and velocities by subtraction.
  Eigen::VectorXd OracleDx(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
    const auto placement = [](const Eigen::VectorXd& x) {
      const Eigen::Quaterniond r(x[6], x[3], x[4], x[5]);
      return pinocchio::SE3(r.normalized().toRotationMatrix(), x.head<3>());
    };
    const int nv = model_.nv;
    Eigen::VectorXd dx(2 * nv);
    dx.head<6>() = pinocchio::log6(placement(q_ref_).actInv(placement(q))).toVector();
    dx.segment(6, nv - 6) = q.tail(model_.nq - 7) - q_ref_.tail(model_.nq - 7);
    dx.tail(nv) = v - v_ref_;
    return dx;
  }

  /// The accel rows of K_g · Δx · scale, and the bound on its rounding.
  void Expect(int g, const Eigen::VectorXd& dx, double scale, const WbcReference& ref,
              const char* what) const {
    const Eigen::MatrixXd& K = K_[static_cast<std::size_t>(g)];
    for (int i = 0; i < layout_.n_acc; ++i) {
      double want = 0.0;
      double mag = 0.0;
      for (int j = 0; j < layout_.ndx; ++j) {
        want += K(layout_.acc_u_offset + i, j) * dx[j];
        mag += std::abs(K(layout_.acc_u_offset + i, j) * dx[j]);
      }
      want *= scale;
      const double got = ref.u_fb_acc[static_cast<std::size_t>(layout_.n_base_v + i)];
      EXPECT_LE(std::abs(got - want), 1e-12 * std::max(1.0, mag) + 4.0 * layout_.ndx * kEps * mag)
          << what << " row " << i;
    }
    for (int i = 0; i < layout_.n_base_v; ++i) {
      EXPECT_EQ(ref.u_fb_acc[static_cast<std::size_t>(i)], 0.0) << what << " base row " << i;
    }
  }

  pinocchio::Model model_;
  RobotModelHandler handler_;
  WbLayout layout_;
  Eigen::VectorXd q_ref_, v_ref_;
  std::array<Eigen::MatrixXd, kMaxGainNodes> K_;
  MPCSolutionManager mgr_;
  WbcReference ref_{};
};

/// @p q_ref_ perturbed in every block.
Eigen::VectorXd Perturbed(const Eigen::VectorXd& q_ref, const pinocchio::Model& model) {
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv);
  for (int i = 0; i < model.nv; ++i) {
    dq[i] = 0.01 * std::sin(1.0 + i);
  }
  Eigen::VectorXd q(model.nq);
  pinocchio::integrate(model, q_ref, dq, q);
  return q;
}

TEST_F(FloatingRiccatiTest, TheFeedbackIsTheGainTimesTheTangentDifference) {
  Init();
  const Eigen::VectorXd q = Perturbed(q_ref_, model_);
  Eigen::VectorXd v(model_.nv);
  for (int i = 0; i < model_.nv; ++i) {
    v[i] = 0.02 * std::cos(2.0 * i);
  }
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
  Expect(0, OracleDx(q, v), 1.0, ref_, "node 0");
}

// The gain of the node nearest the sample time; past the last shipped gain
// node the last one, counted.
TEST_F(FloatingRiccatiTest, TheNearestGainNodeIsUsedAndAClampIsCounted) {
  Init();
  const Eigen::VectorXd q = Perturbed(q_ref_, model_);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  const Eigen::VectorXd dx = OracleDx(q, v);
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(static_cast<int64_t>(kObs) + 2 * kDtNs - 1), ref_));
  Expect(2, dx, 1.0, ref_, "nearest 2");
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(static_cast<int64_t>(kObs) + kDtNs / 2 - 1), ref_));
  Expect(0, dx, 1.0, ref_, "nearest 0");
  EXPECT_EQ(mgr_.GainNodeClamps(), 0u);
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(static_cast<int64_t>(kObs) + 4 * kDtNs), ref_));
  Expect(kMaxGainNodes - 1, dx, 1.0, ref_, "clamped");
  EXPECT_EQ(mgr_.GainNodeClamps(), 1u);
}

// One block perturbed at a time, its cap set below the perturbation: the
// whole feedback scales by cap / ‖Δx_b‖; with the cap above it, by 1.
TEST_F(FloatingRiccatiTest, EachBlockCapScalesTheWholeFeedback) {
  const char* names[] = {"base_position",        "base_orientation",      "joint_position",
                         "base_linear_velocity", "base_angular_velocity", "joint_velocity"};
  const int nv = model_.nv;
  for (int b = 0; b < 6; ++b) {
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(nv);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
    const int entry = b == 0 ? 1 : b == 1 ? 4 : b == 2 ? 9 : b == 3 ? 0 : b == 4 ? 5 : 12;
    (b < 3 ? dq : v)[entry] = 0.02;
    Eigen::VectorXd q(model_.nq);
    pinocchio::integrate(model_, q_ref_, dq, q);
    const Eigen::VectorXd dx = OracleDx(q, v);
    const auto block = [&](int bb) {
      const int s[] = {0, 3, 6, nv, nv + 3, nv + 6, 2 * nv};
      return dx.segment(s[bb], s[bb + 1] - s[bb]).norm();
    };
    for (const double cap : {0.005, 1.0}) {
      const std::string yaml = std::string("{max_delta_x_norm: 1.0e6, max_delta_blocks: {") +
                               names[b] + ": " + std::to_string(cap) + "}}";
      Init(yaml);
      ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
      const double nb = block(b);
      const double scale = nb > cap ? cap / nb : 1.0;
      Expect(0, dx, scale, ref_, names[b]);
    }
  }
}

// A base rotation error close to π: the feedback fades to 0 over the last
// 0.2 rad, so the log map's axis flip at π cannot chatter.
TEST_F(FloatingRiccatiTest, ARotationErrorNearPiFadesTheFeedbackOut) {
  Init();
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -1.0, 0.4).normalized();
  const auto rotated = [&](double theta) {
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
    dq.segment<3>(3) = theta * axis;
    Eigen::VectorXd q(model_.nq);
    pinocchio::integrate(model_, q_ref_, dq, q);
    return q;
  };
  for (const double theta : {kPi - 0.1, kPi - 1e-3}) {
    const Eigen::VectorXd q = rotated(theta);
    const Eigen::VectorXd dx = OracleDx(q, v);
    const double th = dx.segment<3>(3).norm();
    ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
    Expect(0, dx, std::max(0.0, (kPi - th) / kRotationFadeRad), ref_, "near pi");
  }
  // Either side of π the log flips its axis; each side is faded by its own
  // factor, so the jump across π is bounded by the two faded magnitudes.
  const Eigen::VectorXd q_below = rotated(kPi - 1e-3);
  const Eigen::VectorXd q_above = rotated(kPi + 1e-3);
  const Eigen::VectorXd dx_below = OracleDx(q_below, v);
  const Eigen::VectorXd dx_above = OracleDx(q_above, v);
  const double f_below = (kPi - dx_below.segment<3>(3).norm()) / kRotationFadeRad;
  const double f_above = (kPi - dx_above.segment<3>(3).norm()) / kRotationFadeRad;
  ASSERT_TRUE(mgr_.SampleReference(q_below, v, At(kObs), ref_));
  Expect(0, dx_below, f_below, ref_, "below pi");
  const WbcReference below = ref_;
  ASSERT_TRUE(mgr_.SampleReference(q_above, v, At(kObs), ref_));
  Expect(0, dx_above, f_above, ref_, "above pi");
  for (int i = 0; i < layout_.n_acc; ++i) {
    const auto ii = static_cast<std::size_t>(layout_.n_base_v + i);
    const int row = layout_.acc_u_offset + i;
    const double bound = f_below * std::abs((K_[0].row(row) * dx_below)(0)) +
                         f_above * std::abs((K_[0].row(row) * dx_above)(0));
    EXPECT_LE(std::abs(below.u_fb_acc[ii] - ref_.u_fb_acc[ii]), bound + 1e-12) << "row " << i;
  }
}

// Two blocks over their caps: the smaller of the two ratios wins.
TEST_F(FloatingRiccatiTest, TwoBlocksOverTheirCapsTakeTheSmallerRatio) {
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
  dq[1] = 0.02;   // base position
  dq[10] = 0.05;  // a joint
  Eigen::VectorXd q(model_.nq);
  pinocchio::integrate(model_, q_ref_, dq, q);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  const Eigen::VectorXd dx = OracleDx(q, v);
  Init("{max_delta_x_norm: 1.0e6, max_delta_blocks: {base_position: 0.01, joint_position: 0.01}}");
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
  const double s_pos = 0.01 / dx.segment<3>(0).norm();
  const double s_joint = 0.01 / dx.segment(6, model_.nv - 6).norm();
  ASSERT_LT(s_joint, s_pos);
  Expect(0, dx, s_joint, ref_, "two blocks");
}

// The floating ComputeReference puts the joint feedback at the joints'
// tangent entries, as SampleReference does — not on the base rows.
TEST_F(FloatingRiccatiTest, ComputeReferencePutsTheJointFeedbackOnTheJointRows) {
  Init();
  const Eigen::VectorXd q = Perturbed(q_ref_, model_);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd q_ref(model_.nq), v_ref(model_.nv), a(model_.nv), lambda(12), u_fb(model_.nv);
  InterpMeta meta{};
  ASSERT_TRUE(mgr_.ComputeReference(q, v, At(kObs), q_ref, v_ref, a, lambda, u_fb, meta));
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
  for (int i = 0; i < model_.nv; ++i) {
    EXPECT_EQ(u_fb[i], ref_.u_fb_acc[static_cast<std::size_t>(i)]) << "row " << i;
  }
}

// A node quaternion off unit length (beyond the layout's tolerance): the
// reference samples, the feedback is skipped and counted.
TEST_F(FloatingRiccatiTest, ANonUnitNodeQuaternionSkipsTheFeedbackCounted) {
  Init();
  MPCSolution plan = Plan();
  for (std::size_t j = 3; j < 7; ++j) {
    plan.q_traj[0][j] *= 1.001;
  }
  mgr_.PublishSolution(plan);
  const Eigen::VectorXd q = Perturbed(q_ref_, model_);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
  EXPECT_EQ(mgr_.FeedbackSkips(), 1u);
  for (double u : ref_.u_fb_acc) {
    EXPECT_EQ(u, 0.0);
  }
  mgr_.PublishSolution(Plan());  // pass control
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));
  EXPECT_EQ(mgr_.FeedbackSkips(), 1u);
}

TEST_F(FloatingRiccatiTest, AZeroGainScaleGivesNoFeedback) {
  Init();
  mgr_.SetRiccatiGainScale(0.0);
  ASSERT_TRUE(mgr_.SampleReference(Perturbed(q_ref_, model_), Eigen::VectorXd::Zero(model_.nv),
                                   At(kObs), ref_));
  for (double u : ref_.u_fb_acc) {
    EXPECT_EQ(u, 0.0);
  }
}

// A non-finite current state: the reference still samples, the feedback is 0.
TEST_F(FloatingRiccatiTest, ANonFiniteStateGivesNoFeedbackButAReference) {
  Init();
  Eigen::VectorXd q = Perturbed(q_ref_, model_);
  q[10] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(mgr_.SampleReference(q, Eigen::VectorXd::Zero(model_.nv), At(kObs), ref_));
  EXPECT_TRUE(ref_.valid);
  for (double u : ref_.u_fb_acc) {
    EXPECT_EQ(u, 0.0);
  }
  // Pass control: the same state without the NaN feeds back.
  q[10] = q_ref_[10] + 0.01;
  ASSERT_TRUE(mgr_.SampleReference(q, Eigen::VectorXd::Zero(model_.nv), At(kObs), ref_));
  double n = 0.0;
  for (double u : ref_.u_fb_acc) {
    n += std::abs(u);
  }
  EXPECT_GT(n, 0.0);
}

// 1000 ticks with a fresh plan every 10th: no heap allocation anywhere in
// the binary while they run (pinocchio's difference included).
TEST_F(FloatingRiccatiTest, AThousandTicksAllocateNothing) {
  Init();
  const MPCSolution plan = Plan();
  const Eigen::VectorXd q = Perturbed(q_ref_, model_);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  ASSERT_TRUE(mgr_.SampleReference(q, v, At(kObs), ref_));  // warm
  std::size_t seen = 0;
  int valid = 0;
  {
    const rtc::testing::ScopedMallocGate gate;
    for (int t = 0; t < 1000; ++t) {
      if (t % 10 == 0) {
        mgr_.PublishSolution(plan);
      }
      valid +=
          mgr_.SampleReference(q, v, At(static_cast<int64_t>(kObs) + t * 100'000), ref_) ? 1 : 0;
    }
    seen = gate.count();
  }
  EXPECT_EQ(seen, 0u);
  EXPECT_EQ(valid, 1000);
}

// ── Configuration refusals ────────────────────────────────────────────────

class SamplerConfigRefusal : public ::testing::TestWithParam<const char*> {};

TEST_P(SamplerConfigRefusal, LeavesTheManagerDisabled) {
  YAML::Node cfg = YAML::Load(GetParam());
  cfg["enabled"] = true;
  MPCSolutionManager mgr;
  const bool fixed_case = cfg["fixed"] && cfg["fixed"].as<bool>();
  mgr.Init(cfg, fixed_case ? WbLayout::FixedBase(kNj, kNj, 0) : FloatingLayout());
  EXPECT_FALSE(mgr.ConfigValid()) << GetParam();
  EXPECT_FALSE(mgr.Enabled()) << GetParam();
}

INSTANTIATE_TEST_SUITE_P(
    Values, SamplerConfigRefusal,
    ::testing::Values("{max_policy_age_s: -0.1}", "{max_policy_age_s: .nan}",
                      "{max_policy_age_s: .inf}",
                      "{riccati: {max_delta_blocks: {joint_position: 0.0}}}",
                      "{riccati: {max_delta_blocks: {joint_position: -1.0}}}",
                      "{riccati: {max_delta_blocks: {joint_position: .nan}}}",
                      "{riccati: {max_delta_blocks: {elbow: 1.0}}}",
                      "{riccati: {max_delta_blocks: [1.0]}}",
                      "{fixed: true, riccati: {max_delta_blocks: {base_position: 1.0}}}"));

TEST(SamplerConfig, AValidConfigIsTaken) {
  MPCSolutionManager mgr;
  mgr.Init(YAML::Load("{enabled: true, max_policy_age_s: 0.1, riccati: {max_delta_blocks: "
                      "{base_position: 0.1, joint_velocity: 2.0}}}"),
           FloatingLayout());
  EXPECT_TRUE(mgr.ConfigValid());
  EXPECT_TRUE(mgr.Enabled());
  MPCSolutionManager fixed;
  fixed.Init(YAML::Load("{enabled: true, riccati: {max_delta_blocks: {joint_position: 0.1}}}"),
             WbLayout::FixedBase(kNj, kNj, 0));
  EXPECT_TRUE(fixed.ConfigValid());
}

// ── The accel-row product ─────────────────────────────────────────────────

// The accel rows alone against the same rows of the full product: equal up
// to the rounding of a sum of ndx terms.
TEST(RiccatiRows, TheAccelRowProductMatchesTheFullOnesRows) {
  constexpr int kNu = 20;
  constexpr int kNdx = 30;
  constexpr int kRow0 = 8;
  constexpr int kRows = 12;
  std::mt19937_64 rng(20261114);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  Eigen::MatrixXd K(kNu, kNdx);
  Eigen::VectorXd dx(kNdx);
  for (int i = 0; i < kNu; ++i) {
    for (int j = 0; j < kNdx; ++j) {
      K(i, j) = u(rng);
    }
  }
  for (int j = 0; j < kNdx; ++j) {
    dx[j] = 0.1 * u(rng);
  }
  std::vector<double> flat(static_cast<std::size_t>(kNu * kNdx));
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  RiccatiFeedback rows;
  rows.Init(kNdx / 2, kNu, kNdx);
  ASSERT_TRUE(rows.SetAccelRows(kRow0, kRows));
  rows.SetAccelOnly(true);
  rows.UseGain(flat.data(), kNu, kNdx);
  RiccatiFeedback full;
  full.Init(kNdx / 2, kNu, kNdx);
  full.SetAccelOnly(false);
  full.SetGain(flat.data(), kNu, kNdx);
  Eigen::VectorXd a(kRows), f(kNu);
  ASSERT_TRUE(rows.ComputeFromDelta(dx, a));
  ASSERT_TRUE(full.ComputeFromDelta(dx, f));
  for (int i = 0; i < kRows; ++i) {
    double mag = 0.0;
    for (int j = 0; j < kNdx; ++j) {
      mag += std::abs(K(kRow0 + i, j) * dx[j]);
    }
    EXPECT_LE(std::abs(a[i] - f[kRow0 + i]), 4.0 * kNdx * kEps * mag) << "row " << i;
  }
}

}  // namespace
}  // namespace rtc::mpc
