// MPCSolutionManager facade tests.
//
// Covers:
//   * Disabled manager returns false from ComputeReference.
//   * YAML Init correctly reads enabled / max_stale_solutions / riccati.*.
//   * Publish → Consume round-trip through TripleBuffer yields valid ref.
//   * RT-state SeqLock round-trip.
//   * Stale counter increments when no new publish arrives; resets on
//     fresh publish; fallback engages at the configured threshold.

#include "rtc_mpc/manager/mpc_solution_manager.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace rtc::mpc {
namespace {

constexpr int kNq = 3;
constexpr int kNv = 3;
constexpr int kNc = 0;
constexpr int kHorizon = 10;
constexpr double kDtNode = 0.01;

/// Build a trivially valid solution whose q(t) is linear, identity Riccati.
MPCSolution MakeSolution(uint64_t timestamp = 1) {
  MPCSolution sol{};
  sol.timestamp_ns = timestamp;
  sol.horizon_length = kHorizon;
  sol.dt_node = kDtNode;
  sol.converged = true;
  sol.nq = kNq;
  sol.nv = kNv;
  sol.nu = kNv;
  sol.nx = kNq + kNv;
  sol.n_contact_vars = kNc;

  for (int k = 0; k <= kHorizon; ++k) {
    const double t = static_cast<double>(k) * kDtNode;
    for (int i = 0; i < kNq; ++i) {
      sol.q_traj[static_cast<std::size_t>(k)]
                [static_cast<std::size_t>(i)] = t;
    }
    for (int i = 0; i < kNv; ++i) {
      sol.v_traj[static_cast<std::size_t>(k)]
                [static_cast<std::size_t>(i)] = 1.0;
    }
  }
  // Identity Riccati gain (nu x nx) at node 0.
  for (int i = 0; i < kNv; ++i) {
    sol.K_riccati[0][static_cast<std::size_t>(
        i * (kNq + kNv) + i)] = 1.0;
  }
  return sol;
}

YAML::Node MakeConfig(bool enabled, int max_stale = 5) {
  YAML::Node cfg;
  cfg["enabled"] = enabled;
  cfg["max_stale_solutions"] = max_stale;
  YAML::Node riccati;
  riccati["enabled"] = true;
  riccati["gain_scale"] = 1.0;
  riccati["accel_only"] = true;
  cfg["riccati"] = riccati;
  return cfg;
}

class MpcSolutionManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    q_curr_.setZero(kNq);
    v_curr_.setZero(kNv);
    q_ref_.setZero(kNq);
    v_ref_.setZero(kNv);
    a_ff_.setZero(kNv);
    lambda_ref_.setZero(std::max(1, kNc));
    u_fb_.setZero(kNv);
  }

  MPCSolutionManager mgr_;
  Eigen::VectorXd q_curr_, v_curr_;
  Eigen::VectorXd q_ref_, v_ref_, a_ff_;
  Eigen::VectorXd lambda_ref_, u_fb_;
  InterpMeta meta_{};
};

TEST_F(MpcSolutionManagerTest, DisabledShortCircuits) {
  mgr_.Init(MakeConfig(false), kNq, kNv, kNc);
  EXPECT_FALSE(mgr_.Enabled());

  const bool ok = mgr_.ComputeReference(
      q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(meta_.valid);
  EXPECT_DOUBLE_EQ(u_fb_.norm(), 0.0);
}

TEST_F(MpcSolutionManagerTest, YamlLoadsMaxStaleAndRiccati) {
  mgr_.Init(MakeConfig(true, 7), kNq, kNv, kNc);
  EXPECT_TRUE(mgr_.Enabled());
  EXPECT_EQ(mgr_.MaxStaleSolutions(), 7);
  EXPECT_DOUBLE_EQ(mgr_.RiccatiGainScale(), 1.0);
}

TEST_F(MpcSolutionManagerTest, PublishConsumeRoundTrip) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);

  const uint64_t now = 1'000'000'000;
  mgr_.PublishSolution(MakeSolution(now));

  const bool ok = mgr_.ComputeReference(
      q_curr_, v_curr_, now, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_);
  ASSERT_TRUE(ok);
  EXPECT_TRUE(meta_.valid);
  EXPECT_FALSE(meta_.beyond_horizon);
  // At t=0 the reference position is zero (linear trajectory).
  for (int i = 0; i < kNq; ++i) {
    EXPECT_NEAR(q_ref_(i), 0.0, 1e-12);
  }
  for (int i = 0; i < kNv; ++i) {
    EXPECT_NEAR(v_ref_(i), 1.0, 1e-12);
  }
}

TEST_F(MpcSolutionManagerTest, StateSeqLockRoundTrip) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);

  q_curr_ << 0.5, -0.2, 1.3;
  v_curr_ << 0.1, 0.2, 0.3;
  mgr_.WriteState(q_curr_, v_curr_, 42);

  const MPCStateSnapshot snap = mgr_.ReadState();
  EXPECT_EQ(snap.timestamp_ns, 42U);
  EXPECT_EQ(snap.nq, kNq);
  EXPECT_EQ(snap.nv, kNv);
  EXPECT_DOUBLE_EQ(snap.q[0], 0.5);
  EXPECT_DOUBLE_EQ(snap.q[1], -0.2);
  EXPECT_DOUBLE_EQ(snap.q[2], 1.3);
  EXPECT_DOUBLE_EQ(snap.v[2], 0.3);
}

TEST_F(MpcSolutionManagerTest, StaleCounterIncrementsAndResets) {
  mgr_.Init(MakeConfig(true, 3), kNq, kNv, kNc);

  const uint64_t t0 = 1'000'000'000;
  mgr_.PublishSolution(MakeSolution(t0));

  // First call consumes the fresh solution — stale stays 0.
  EXPECT_TRUE(mgr_.ComputeReference(
      q_curr_, v_curr_, t0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StaleCount(), 0);

  // Subsequent calls without a new publish: counter climbs.
  EXPECT_TRUE(mgr_.ComputeReference(
      q_curr_, v_curr_, t0 + 1, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
      meta_));
  EXPECT_EQ(mgr_.StaleCount(), 1);
  EXPECT_TRUE(mgr_.ComputeReference(
      q_curr_, v_curr_, t0 + 2, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
      meta_));
  EXPECT_EQ(mgr_.StaleCount(), 2);

  // Threshold 3: next stale increment triggers fallback.
  EXPECT_FALSE(mgr_.ComputeReference(
      q_curr_, v_curr_, t0 + 3, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
      meta_));
  EXPECT_EQ(mgr_.StaleCount(), 3);

  // Publish a fresh solution → counter resets, ComputeReference succeeds.
  mgr_.PublishSolution(MakeSolution(t0 + 1000));
  EXPECT_TRUE(mgr_.ComputeReference(
      q_curr_, v_curr_, t0 + 1000, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
      meta_));
  EXPECT_EQ(mgr_.StaleCount(), 0);
}

TEST_F(MpcSolutionManagerTest, NoSolutionYieldsFallback) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  EXPECT_FALSE(mgr_.HasEverReceivedSolution());
  EXPECT_FALSE(mgr_.ComputeReference(
      q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
}

}  // namespace
}  // namespace rtc::mpc
