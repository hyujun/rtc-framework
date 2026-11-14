// MPCSolutionManager facade tests.
//
// Covers:
//   * Disabled manager returns false from ComputeReference.
//   * YAML Init correctly reads enabled / max_stale_solutions / riccati.*.
//   * Publish → Consume round-trip through TripleBuffer yields valid ref.
//   * RT-state SeqLock round-trip.
//   * Stale counter increments when no new publish arrives; resets on
//     fresh publish; fallback engages at the configured threshold.

#include "rtc_mpc/feedback/riccati_feedback.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

namespace rtc::mpc {
namespace {

constexpr int kNq = 3;
constexpr int kNv = 3;
constexpr int kNc = 0;
constexpr int kHorizon = 10;
constexpr double kDtNode = 0.01;

/// Build a trivially valid solution whose q(t) is linear, identity Riccati,
/// observed at @p timestamp (the consumer clock the tests sample with).
MPCSolution MakeSolution(uint64_t timestamp = 1) {
  MPCSolution sol{};
  sol.timestamp_ns = timestamp;
  sol.observation_stamp_ns = timestamp;
  sol.horizon_length = kHorizon;
  sol.dt_node = kDtNode;
  sol.converged = true;
  sol.nq = kNq;
  sol.nv = kNv;
  sol.nu = kNv;
  sol.nx = kNq + kNv;
  sol.ndx = 2 * kNv;
  sol.n_contact_vars = kNc;
  sol.gain_nodes = 1;  // node 0 carries the identity gain below

  for (int k = 0; k <= kHorizon; ++k) {
    const double t = static_cast<double>(k) * kDtNode;
    for (int i = 0; i < kNq; ++i) {
      sol.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = t;
    }
    for (int i = 0; i < kNv; ++i) {
      sol.v_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = 1.0;
    }
  }
  // Identity Riccati gain (nu × ndx) at node 0, in the shared flat layout.
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(kNv, 2 * kNv);
  K.topLeftCorner(kNv, kNv).setIdentity();
  EXPECT_TRUE(PackRiccatiGainRowMajor(K, sol.K_riccati[0].data(), sol.K_riccati[0].size()));
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

// The manager (TripleBuffer slots, interpolator copy) must stay small enough
// to live on a stack or as a controller member: the payloads are on the heap.
static_assert(sizeof(MPCSolutionManager) < 64 * 1024,
              "MPCSolutionManager must not embed MPCSolution payloads inline");

TEST_F(MpcSolutionManagerTest, WriteStateStampsSequenceEpochMaskAndValid) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  EXPECT_FALSE(mgr_.ReadState().valid) << "nothing stored yet";

  mgr_.SetModelEpoch(3);
  EXPECT_EQ(mgr_.ModelEpoch(), 3u);
  mgr_.SetContactMask(0b10u);
  mgr_.WriteState(q_curr_, v_curr_, 100);
  const MPCStateSnapshot first = mgr_.ReadState();
  EXPECT_TRUE(first.valid);
  EXPECT_EQ(first.sequence, 1u);
  EXPECT_EQ(first.model_epoch, 3u);
  EXPECT_EQ(first.contact_mask, 0b10u);
  EXPECT_EQ(first.timestamp_ns, 100u);

  mgr_.SetContactMask(0b11u);
  mgr_.WriteState(q_curr_, v_curr_, 200);
  const MPCStateSnapshot second = mgr_.ReadState();
  EXPECT_EQ(second.sequence, 2u) << "sequence increments per store";
  EXPECT_EQ(second.contact_mask, 0b11u);
  EXPECT_EQ(second.model_epoch, 3u);
}

TEST_F(MpcSolutionManagerTest, PublishStampsMonotonicPlanIdWithoutTouchingTheSource) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  MPCSolution sol = MakeSolution();
  sol.observation_stamp_ns = 55;
  sol.model_epoch = 4;
  ASSERT_EQ(sol.plan_id, 0u);
  EXPECT_EQ(mgr_.ActivePlanId(), 0u) << "nothing acquired yet";

  mgr_.SetModelEpoch(4);
  mgr_.PublishSolution(sol);
  EXPECT_EQ(sol.plan_id, 0u) << "the caller's copy is left untouched";
  ASSERT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, 60, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
                                    meta_));
  EXPECT_EQ(mgr_.ActivePlanId(), 1u);
  EXPECT_EQ(mgr_.ActiveReceiveStampNs(), 60u) << "stamped with the consumer's own clock";
  EXPECT_EQ(mgr_.ActiveObservationStampNs(), 55u);
  EXPECT_EQ(mgr_.ActiveModelEpoch(), 4u);

  // Two more publishes before the consumer looks again: it sees the newest.
  mgr_.PublishSolution(sol);
  mgr_.PublishSolution(sol);
  ASSERT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, 61, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
                                    meta_));
  EXPECT_EQ(mgr_.ActivePlanId(), 3u);
  EXPECT_EQ(mgr_.ActiveReceiveStampNs(), 61u);
}

TEST_F(MpcSolutionManagerTest, PlanFromAnotherModelEpochIsNotInstalled) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  MPCSolution sol = MakeSolution();
  sol.model_epoch = 1;

  // Current epoch is 0: the plan is refused and counted, nothing is sampled.
  mgr_.PublishSolution(sol);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.EpochRejections(), 1u);
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);
  EXPECT_FALSE(mgr_.HasEverReceivedSolution());

  // The same plan under the matching epoch installs.
  mgr_.SetModelEpoch(1);
  mgr_.PublishSolution(sol);
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.EpochRejections(), 1u);
  EXPECT_EQ(mgr_.ActiveModelEpoch(), 1u);

  // Re-Init forgets the active plan and the counters.
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);
  EXPECT_EQ(mgr_.EpochRejections(), 0u);
  EXPECT_EQ(mgr_.DimensionRejections(), 0u);
}

TEST_F(MpcSolutionManagerTest, UnusablePlanIsNotInstalledAndDoesNotResetStaleness) {
  mgr_.Init(MakeConfig(true, 2), kNq, kNv, kNc);
  MPCSolution good = MakeSolution();
  good.observation_stamp_ns = 5;
  mgr_.PublishSolution(good);
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.ActivePlanId(), 1u);

  // A diverged plan arrives: it must not replace the good one, nor count as
  // a fresh solution — the consumer keeps sampling and ageing the old plan.
  MPCSolution bad = MakeSolution();
  bad.status = SolverStatus::kDiverged;
  bad.observation_stamp_ns = 6;
  mgr_.PublishSolution(bad);
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StaleCount(), 1) << "a rejected plan is a tick without a fresh one";
  EXPECT_EQ(mgr_.ActivePlanId(), 1u);
  EXPECT_EQ(mgr_.ActiveObservationStampNs(), 5u);
}

TEST_F(MpcSolutionManagerTest, LayoutInitMatchesTheFixedBaseConvenienceOverload) {
  MPCSolutionManager by_layout;
  by_layout.Init(MakeConfig(true), WbLayout::FixedBase(kNq, kNv, kNc));
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  EXPECT_FALSE(mgr_.Layout().floating_base());
  EXPECT_EQ(mgr_.Layout().nu, kNv);

  const MPCSolution sol = MakeSolution();
  by_layout.PublishSolution(sol);
  mgr_.PublishSolution(sol);
  Eigen::VectorXd q2(kNq), v2(kNv), a2(kNv), l2(std::max(1, kNc)), u2(kNv);
  InterpMeta m2{};
  q_curr_.setConstant(0.2);
  ASSERT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, 5'000'000, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                    u_fb_, meta_));
  ASSERT_TRUE(by_layout.ComputeReference(q_curr_, v_curr_, 5'000'000, q2, v2, a2, l2, u2, m2));
  EXPECT_EQ(q2, q_ref_) << "same layout, same reference, bit-exact";
  EXPECT_EQ(u2, u_fb_);
}

TEST_F(MpcSolutionManagerTest, InvalidLayoutLeavesTheManagerDisabled) {
  // A factory that refused its inputs returns an empty layout; the YAML
  // switch alone must not turn such a manager on.
  mgr_.Init(MakeConfig(true), WbLayout::FixedBase(0, 0, 0));
  EXPECT_FALSE(mgr_.Layout().valid());
  EXPECT_FALSE(mgr_.Enabled());
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
}

TEST_F(MpcSolutionManagerTest, FloatingLayoutIsRefusedBeforeInterpolation) {
  WbLayout floating = WbLayout::FixedBase(kNq + 7, kNv + 6, kNc);
  floating.n_base_q = 7;
  floating.n_base_v = 6;
  floating.n_acc = kNv;
  floating.n_wrench_u = kNc;
  floating.acc_u_offset = kNc;
  floating.nu = kNc + kNv;
  mgr_.Init(MakeConfig(true), floating);
  EXPECT_TRUE(mgr_.Layout().floating_base());

  MPCSolution sol = MakeSolution();
  sol.nq = kNq + 7;
  sol.nv = kNv + 6;
  mgr_.PublishSolution(sol);
  Eigen::VectorXd q(kNq + 7), v(kNv + 6), q_ref(kNq + 7), v_ref(kNv + 6), a_ff(kNv + 6),
      u_fb(kNv + 6);
  q.setZero();
  v.setZero();
  EXPECT_FALSE(mgr_.ComputeReference(q, v, 0, q_ref, v_ref, a_ff, lambda_ref_, u_fb, meta_));
  EXPECT_TRUE(meta_.floating_unsupported);
  EXPECT_FALSE(meta_.valid);
  EXPECT_FALSE(mgr_.HasEverReceivedSolution()) << "refused before any plan is acquired";
}

TEST_F(MpcSolutionManagerTest, PlanWhoseDimensionsDisagreeWithTheLayoutIsRejected) {
  // The Riccati workspace is sized by the layout; a plan with another input
  // dimension is refused and counted instead of being silently dropped by
  // SetGain (or worse, installed against the wrong layout).
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  MPCSolution sol = MakeSolution();
  sol.nu = kNv + 2;
  mgr_.PublishSolution(sol);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.DimensionRejections(), 1u);
  EXPECT_EQ(mgr_.EpochRejections(), 0u);
  EXPECT_FALSE(mgr_.HasEverReceivedSolution());

  // The matching plan installs and its gain reaches u_fb.
  sol.nu = kNv;
  mgr_.PublishSolution(sol);
  q_curr_(0) = 0.25;  // position error on joint 0 at t = 0 (q_ref = 0 there)
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_DOUBLE_EQ(u_fb_(0), 0.25);
  EXPECT_EQ(mgr_.DimensionRejections(), 1u);
}

TEST_F(MpcSolutionManagerTest, DisabledShortCircuits) {
  mgr_.Init(MakeConfig(false), kNq, kNv, kNc);
  EXPECT_FALSE(mgr_.Enabled());

  const bool ok =
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_);
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

  const bool ok = mgr_.ComputeReference(q_curr_, v_curr_, now, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                        u_fb_, meta_);
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
  EXPECT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, t0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_,
                                    meta_));
  EXPECT_EQ(mgr_.StaleCount(), 0);

  // Subsequent calls without a new publish: counter climbs.
  EXPECT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, t0 + 1, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                    u_fb_, meta_));
  EXPECT_EQ(mgr_.StaleCount(), 1);
  EXPECT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, t0 + 2, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                    u_fb_, meta_));
  EXPECT_EQ(mgr_.StaleCount(), 2);

  // Threshold 3: next stale increment triggers fallback.
  EXPECT_FALSE(mgr_.ComputeReference(q_curr_, v_curr_, t0 + 3, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                     u_fb_, meta_));
  EXPECT_EQ(mgr_.StaleCount(), 3);

  // Publish a fresh solution → counter resets, ComputeReference succeeds.
  mgr_.PublishSolution(MakeSolution(t0 + 1000));
  EXPECT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, t0 + 1000, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                    u_fb_, meta_));
  EXPECT_EQ(mgr_.StaleCount(), 0);
}

TEST_F(MpcSolutionManagerTest, NoSolutionYieldsFallback) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  EXPECT_FALSE(mgr_.HasEverReceivedSolution());
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
}

// ── Solve-timing probe (Phase 7c perf follow-up) ────────────────────────────

TEST_F(MpcSolutionManagerTest, SolveStatsEmptyBeforeFirstPublish) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  const auto s = mgr_.GetSolveStats();
  EXPECT_EQ(s.count, 0u);
  EXPECT_EQ(s.window, 0u);
  EXPECT_EQ(s.last_ns, 0u);
  EXPECT_EQ(s.min_ns, 0u);
  EXPECT_EQ(s.max_ns, 0u);
  EXPECT_EQ(s.p50_ns, 0u);
  EXPECT_EQ(s.p99_ns, 0u);
  EXPECT_DOUBLE_EQ(s.mean_ns, 0.0);
}

TEST_F(MpcSolutionManagerTest, SolveStatsAggregatesPercentiles) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);

  // Feed a 100-sample window where sample i has solve_duration_ns = (i+1)*1e6
  // (1 ms, 2 ms, …, 100 ms). p50 should land near 50 ms, p99 near 99 ms.
  for (int i = 0; i < 100; ++i) {
    MPCSolution sol = MakeSolution(static_cast<uint64_t>(i + 1));
    sol.solve_duration_ns = static_cast<uint64_t>(i + 1) * uint64_t{1'000'000};
    mgr_.PublishSolution(sol);
  }

  const auto s = mgr_.GetSolveStats();
  EXPECT_EQ(s.count, 100u);
  EXPECT_EQ(s.window, 100u);
  EXPECT_EQ(s.last_ns, 100'000'000u);
  EXPECT_EQ(s.min_ns, 1'000'000u);
  EXPECT_EQ(s.max_ns, 100'000'000u);
  // Rank-based p50 of 100 sorted samples → index round(0.5 * 99) = 50 →
  // samples[50] = 51 ms (0-indexed). p99 → round(0.99 * 99) = 98 → 99 ms.
  EXPECT_EQ(s.p50_ns, 51'000'000u);
  EXPECT_EQ(s.p99_ns, 99'000'000u);
  // Mean of 1..100 ms = 50.5 ms.
  EXPECT_NEAR(s.mean_ns, 50'500'000.0, 1.0);
}

TEST_F(MpcSolutionManagerTest, SolveStatsRingBufferOverwrites) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);

  // Fill beyond the window so the oldest samples fall off. Window size is
  // MPCSolutionManager::kSolveStatsWindow (256). Publish 300 samples where
  // the last 256 all have duration == 5 ms. Statistics must reflect only
  // that tail; the earlier 1 ms samples should be gone.
  for (int i = 0; i < 300; ++i) {
    MPCSolution sol = MakeSolution(static_cast<uint64_t>(i + 1));
    sol.solve_duration_ns = (i < 44) ? 1'000'000ULL : 5'000'000ULL;
    mgr_.PublishSolution(sol);
  }

  const auto s = mgr_.GetSolveStats();
  EXPECT_EQ(s.count, 300u);
  EXPECT_EQ(s.window, MPCSolutionManager::kSolveStatsWindow);
  EXPECT_EQ(s.last_ns, 5'000'000u);
  EXPECT_EQ(s.min_ns, 5'000'000u);
  EXPECT_EQ(s.max_ns, 5'000'000u);
  EXPECT_EQ(s.p50_ns, 5'000'000u);
  EXPECT_EQ(s.p99_ns, 5'000'000u);
  EXPECT_DOUBLE_EQ(s.mean_ns, 5'000'000.0);
}

TEST_F(MpcSolutionManagerTest, ResetSolveStatsClearsAllFields) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  for (int i = 0; i < 50; ++i) {
    MPCSolution sol = MakeSolution(static_cast<uint64_t>(i + 1));
    sol.solve_duration_ns = 1'000'000ULL;
    mgr_.PublishSolution(sol);
  }
  ASSERT_GT(mgr_.GetSolveStats().count, 0u);

  mgr_.ResetSolveStats();

  const auto s = mgr_.GetSolveStats();
  EXPECT_EQ(s.count, 0u);
  EXPECT_EQ(s.window, 0u);
  EXPECT_EQ(s.last_ns, 0u);
}

// The stats producer (MPC thread, RT) publishes through a SeqLock and the
// non-RT reader must never see a torn window. Sample k (1-based) carries
// solve_duration_ns == k, so every consistent snapshot satisfies
// last == max == count and min == count - window + 1; a mix of two publishes
// breaks at least one of them.
TEST_F(MpcSolutionManagerTest, SolveStatsSnapshotIsConsistentUnderConcurrentPublish) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  constexpr std::uint64_t kPublishes = 20'000;
  std::atomic<bool> done{false};
  std::thread producer([&] {
    MPCSolution sol = MakeSolution(1);
    for (std::uint64_t k = 1; k <= kPublishes; ++k) {
      sol.timestamp_ns = k;
      sol.solve_duration_ns = k;
      mgr_.PublishSolution(sol);
    }
    done.store(true, std::memory_order_release);
  });

  std::uint64_t reads = 0;
  std::uint64_t inconsistent = 0;
  std::uint64_t prev_count = 0;
  while (!done.load(std::memory_order_acquire) || reads == 0) {
    const auto s = mgr_.GetSolveStats();
    ++reads;
    if (s.count == 0) {
      continue;
    }
    const std::uint64_t expect_window =
        std::min<std::uint64_t>(s.count, MPCSolutionManager::kSolveStatsWindow);
    const bool ok = s.window == expect_window && s.last_ns == s.count && s.max_ns == s.count &&
                    s.min_ns == s.count - s.window + 1 && s.count >= prev_count;
    if (!ok) {
      ++inconsistent;
    }
    prev_count = s.count;
  }
  producer.join();

  EXPECT_EQ(inconsistent, 0u) << "torn snapshots out of " << reads << " reads";
  EXPECT_EQ(mgr_.GetSolveStats().count, kPublishes);
}

// ── State epoch, eviction, the age cap and re-enable ───────────────────────

TEST_F(MpcSolutionManagerTest, WriteStateCarriesTheSourceSequenceAndTheStateEpoch) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  mgr_.SetStateEpoch(9);
  EXPECT_EQ(mgr_.StateEpoch(), 9u);
  mgr_.WriteState(q_curr_, v_curr_, 100, 0xABCDEF0123456789ULL);
  const MPCStateSnapshot a = mgr_.ReadState();
  EXPECT_EQ(a.source_sequence, 0xABCDEF0123456789ULL);
  EXPECT_EQ(a.state_epoch, 9u);
  EXPECT_EQ(a.sequence, 1u) << "the manager's own count is independent of the source's";

  mgr_.WriteState(q_curr_, v_curr_, 200);  // the default: no source sequence
  const MPCStateSnapshot b = mgr_.ReadState();
  EXPECT_EQ(b.source_sequence, 0u);
  EXPECT_EQ(b.sequence, 2u);
  EXPECT_EQ(b.state_epoch, 9u);
}

TEST_F(MpcSolutionManagerTest, PlanFromAnotherStateEpochIsNotInstalled) {
  mgr_.Init(MakeConfig(true), kNq, kNv, kNc);
  mgr_.SetStateEpoch(2);
  MPCSolution sol = MakeSolution();
  sol.state_epoch = 1;
  mgr_.PublishSolution(sol);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StateEpochRejections(), 1u);
  EXPECT_EQ(mgr_.EpochRejections(), 0u) << "a state epoch is not a model epoch";
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);
  EXPECT_FALSE(mgr_.HasEverReceivedSolution()) << "refused at arrival, never installed";

  // The same plan under the current state epoch installs.
  sol.state_epoch = 2;
  mgr_.PublishSolution(sol);
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 0, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StateEpochRejections(), 1u);
}

// The plan being sampled is dropped at the first sample after a bump, with
// no newer plan arriving — the gate does not wait for one.
TEST_F(MpcSolutionManagerTest, AStateEpochBumpEvictsTheActivePlan) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  mgr_.PublishSolution(MakeSolution());
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 1, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  // Pass control: without a bump the held plan keeps sampling.
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 2, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));

  mgr_.SetStateEpoch(1);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 3, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_FALSE(meta_.valid);
  EXPECT_EQ(mgr_.StateEpochRejections(), 1u);
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 4, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StateEpochRejections(), 1u) << "counted once per evicted plan";

  // A plan of the new epoch is accepted again.
  MPCSolution next = MakeSolution();
  next.state_epoch = 1;
  mgr_.PublishSolution(next);
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 5, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
}

TEST_F(MpcSolutionManagerTest, AModelEpochBumpEvictsTheActivePlan) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  mgr_.PublishSolution(MakeSolution());
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 1, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  mgr_.SetModelEpoch(5);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 2, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.EpochRejections(), 1u);
  EXPECT_EQ(mgr_.StateEpochRejections(), 0u);
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);
}

// Age = now − observation stamp, cap = (horizon + 1) node intervals: at the
// cap the plan still samples (held on its last node), one nanosecond past it
// the plan is dropped and counted once.
TEST_F(MpcSolutionManagerTest, APlanOlderThanItsHorizonPlusOneNodeIsDropped) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  const uint64_t obs = 7'000'000'000ULL;
  const uint64_t cap = static_cast<uint64_t>(kHorizon + 1) * 10'000'000ULL;  // kDtNode = 10 ms
  MPCSolution sol = MakeSolution(obs);
  sol.observation_stamp_ns = obs;
  mgr_.PublishSolution(sol);
  ASSERT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, obs, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                    u_fb_, meta_));
  EXPECT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, obs + cap, q_ref_, v_ref_, a_ff_, lambda_ref_,
                                    u_fb_, meta_));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 0u);

  EXPECT_FALSE(mgr_.ComputeReference(q_curr_, v_curr_, obs + cap + 1, q_ref_, v_ref_, a_ff_,
                                     lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 1u);
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);
  EXPECT_FALSE(mgr_.ComputeReference(q_curr_, v_curr_, obs + cap + 2, q_ref_, v_ref_, a_ff_,
                                     lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 1u);
}

// A plan that arrives already past the cap is installed and dropped in the
// same call; an observation stamp ahead of `now` is a negative age, not a
// wrapped one.
TEST_F(MpcSolutionManagerTest, TheAgeCapAppliesOnArrivalAndNeverWraps) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  MPCSolution old = MakeSolution();
  old.observation_stamp_ns = 0;
  mgr_.PublishSolution(old);
  EXPECT_FALSE(mgr_.ComputeReference(q_curr_, v_curr_, 1'000'000'000ULL, q_ref_, v_ref_, a_ff_,
                                     lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 1u);

  MPCSolution ahead = MakeSolution();
  ahead.observation_stamp_ns = 1'000'000'055ULL;
  mgr_.PublishSolution(ahead);
  EXPECT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, 1'000'000'000ULL, q_ref_, v_ref_, a_ff_,
                                    lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 1u);
}

// Disable → enable: the first sample after the re-enable has no plan, even
// though the held one is young; a plan published after it is taken.
TEST_F(MpcSolutionManagerTest, ReEnablingDropsThePlanHeldFromBefore) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  mgr_.PublishSolution(MakeSolution());
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 1, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));

  mgr_.SetEnabled(false);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 2, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  mgr_.SetEnabled(true);
  mgr_.SetEnabled(true);  // a repeated `true` is not another re-enable
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 3, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.ActivePlanId(), 0u);

  mgr_.PublishSolution(MakeSolution());
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 4, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 5, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_))
      << "the plan taken after the re-enable is kept";
}

// After a re-enable, a plan computed from a state written before it is not
// taken — one published while disabled (the triple buffer keeps it) nor one
// the MPC side publishes afterwards from the frozen snapshot; a plan of the
// first state written after the re-enable is.
TEST_F(MpcSolutionManagerTest, APlanOfAStateFromBeforeTheReEnableIsNotTaken) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  mgr_.WriteState(q_curr_, v_curr_, 1);
  const uint64_t before = mgr_.ReadState().sequence;
  MPCSolution old_plan = MakeSolution(1);
  old_plan.observation_sequence = before;
  mgr_.SetEnabled(false);
  mgr_.PublishSolution(old_plan);  // published while disabled
  mgr_.SetEnabled(true);
  mgr_.WriteState(q_curr_, v_curr_, 2);  // the consumer's order: state, then sample
  const uint64_t after = mgr_.ReadState().sequence;
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 2, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StateEpochRejections(), 1u);
  mgr_.PublishSolution(old_plan);  // the frozen snapshot, solved again after the re-enable
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 3, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.StateEpochRejections(), 2u);

  MPCSolution fresh = MakeSolution(2);
  fresh.observation_sequence = after;
  mgr_.PublishSolution(fresh);
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 4, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
}

// A plan observed far ahead of `now` — the consumer's clock went back (a
// reset that restarts its tick count) — is dropped, not held at node 0.
TEST_F(MpcSolutionManagerTest, APlanFromAheadOfAClockThatWentBackIsDropped) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  const uint64_t cap = static_cast<uint64_t>(kHorizon + 1) * 10'000'000ULL;
  mgr_.PublishSolution(MakeSolution(5'000'000'000ULL));
  ASSERT_TRUE(mgr_.ComputeReference(q_curr_, v_curr_, 5'000'000'000ULL - cap, q_ref_, v_ref_, a_ff_,
                                    lambda_ref_, u_fb_, meta_))
      << "at the cap ahead the plan still samples";
  EXPECT_FALSE(mgr_.ComputeReference(q_curr_, v_curr_, 5'000'000'000ULL - cap - 1, q_ref_, v_ref_,
                                     a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.PolicyAgeRejections(), 1u);
}

// An evicted plan's gain is not applied to the next plan when that one
// ships none; pass control: a plan with a gain feeds the error back.
TEST_F(MpcSolutionManagerTest, AnEvictedPlansGainIsNotAppliedToTheNextPlan) {
  mgr_.Init(MakeConfig(true, 1000), kNq, kNv, kNc);
  q_curr_.setConstant(0.5);  // a position error the identity gain would feed back
  mgr_.PublishSolution(MakeSolution());
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 1, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_NE(u_fb_.norm(), 0.0);

  mgr_.SetStateEpoch(1);
  EXPECT_FALSE(
      mgr_.ComputeReference(q_curr_, v_curr_, 2, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(mgr_.ActiveReceiveStampNs(), 0u) << "the evicted plan's provenance goes with it";
  MPCSolution no_gain = MakeSolution();
  no_gain.state_epoch = 1;
  no_gain.gain_nodes = 0;
  mgr_.PublishSolution(no_gain);
  ASSERT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 3, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
  EXPECT_EQ(u_fb_.norm(), 0.0);
}

// Pass control for the re-enable rule: enabling a manager that was
// initialised disabled, before any plan, takes the first plan published.
TEST_F(MpcSolutionManagerTest, EnablingBeforeTheFirstPlanTakesIt) {
  mgr_.Init(MakeConfig(false), kNq, kNv, kNc);
  mgr_.SetEnabled(true);
  mgr_.PublishSolution(MakeSolution());
  EXPECT_TRUE(
      mgr_.ComputeReference(q_curr_, v_curr_, 1, q_ref_, v_ref_, a_ff_, lambda_ref_, u_fb_, meta_));
}

// Per-tick raw-sample stream is no longer owned by MPCSolutionManager — it
// lives on MPCThread::TimingProducer() (see test_mpc_timing_logger.cpp +
// test_mpc_thread_skeleton.cpp). The aggregate windowed ring driven by
// PublishSolution is covered by the GetSolveStats tests above.

}  // namespace
}  // namespace rtc::mpc
