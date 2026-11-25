/// @file test_clik_scenarios.cpp
/// @brief The shared CLIK scenarios (clik_golden_scenarios.hpp) reach every branch of
///   ClikReferenceGenerator::Compute().
///
/// test_clik_multiframe replays these scenarios through the multi-frame overload and
/// compares in the same run; this file is what keeps that replay from being vacuous.
/// It asserts properties of the current run only — nothing is recorded from a
/// host (a recorded table pins the numerics of one build, not the behaviour; PROC-6).
///
/// Inputs are open-loop: every tick's measured q comes from a fixed sequence,
/// not from the previous q_ref. Four scenarios together reach every branch of
/// Compute(): velocity clamp, position box, box collapse (both directions, with
/// and without the velocity re-clamp), carry-forward anchor, measured reseed,
/// anchor drift clamp, both failure branches (QP failure on a non-finite target;
/// non-finite output on dt = +inf) and their forced re-anchor, hand posture,
/// universe and registered (non-identity) base frames.

#include "clik_golden_scenarios.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iterator>
#include <vector>

namespace rtc::tsid {
namespace {

using test::clik_golden::kNv;
using test::clik_golden::kRecordWidth;
using test::clik_golden::kScenarios;
// ProxQP eps_abs (QPSolverConfig default): the box holds only to this tolerance.
constexpr double kSolverEps = 1e-6;

class ClikGoldenTest : public test::clik_golden::ClikGoldenScenarioTest {
 protected:
  [[nodiscard]] std::vector<double> RunAll() {
    std::vector<double> all;
    for (int s = 0; s < static_cast<int>(std::size(kScenarios)); ++s) {
      const std::vector<double> r =
          RunScenario(s, [](ClikReferenceGenerator& gen, const test::clik_golden::Tick& t) {
            return gen.Compute(t.cache, t.tcp_idx, t.base_idx, t.des, t.q_posture, t.dt, t.reseed);
          });
      all.insert(all.end(), r.begin(), r.end());
    }
    return all;
  }
};

// Every branch the scenarios claim to cover actually fires, checked on the current run.
TEST_F(ClikGoldenTest, ScenariosReachEveryBranch) {
  const std::vector<double> got = RunAll();
  auto at = [&](int s, int tick, int field) {
    size_t base = 0;
    for (int i = 0; i < s; ++i) {
      base += static_cast<size_t>(kScenarios[i].ticks * kRecordWidth);
    }
    return got[base + static_cast<size_t>(tick * kRecordWidth + field)];
  };
  auto v_ref = [&](int s, int tick, int j) { return at(s, tick, 1 + kNv + j); };

  // Velocity clamp active somewhere in scenario 0 (v_limit 0.5).
  bool clamp_hit = false;
  for (int k = 0; k < kScenarios[0].ticks; ++k) {
    for (int j = 0; j < 7; ++j) {
      clamp_hit = clamp_hit || std::abs(std::abs(v_ref(0, k, j)) - 0.5) < 1e-6;
    }
  }
  EXPECT_TRUE(clamp_hit) << "velocity clamp never active";

  // Collapse above q_max (joint 3, ticks 10–14): only motion back, bounded by v_limit.
  for (int k = 10; k < 15; ++k) {
    EXPECT_LE(v_ref(0, k, 3), 0.0) << k;
    EXPECT_GE(v_ref(0, k, 3), -0.5 - kSolverEps) << k;
  }
  // Collapse below q_min (joint 5, ticks 20–24).
  for (int k = 20; k < 25; ++k) {
    EXPECT_GE(v_ref(0, k, 5), 0.0) << k;
    EXPECT_LE(v_ref(0, k, 5), 0.5 + kSolverEps) << k;
  }
  // Velocity box off: the raw collapse exceeds any sane limit above q_max.
  bool raw_collapse = false;
  for (int k = 10; k < 15; ++k) {
    raw_collapse = raw_collapse || v_ref(2, k, 3) < -0.5;
  }
  EXPECT_TRUE(raw_collapse) << "unbounded collapse branch not reached";

  // Failure branches: scenario 1 tick 25 (QP), scenario 3 tick 12 (non-finite
  // output); success on every other tick, including right after each.
  EXPECT_EQ(at(1, 25, 0), 0.0);
  EXPECT_EQ(at(3, 12, 0), 0.0);
  for (int k = 0; k < kScenarios[3].ticks; ++k) {
    if (k != 12) {
      EXPECT_EQ(at(3, k, 0), 1.0) << k;
    }
  }
  for (int k = 0; k < kScenarios[1].ticks; ++k) {
    if (k != 25) {
      EXPECT_EQ(at(1, k, 0), 1.0) << k;
    }
  }
  // Drift clamp active in scenario 1: some |q_ref − q_meas| sits at 0.02.
  bool drift_hit = false;
  for (int k = 1; k < kScenarios[1].ticks; ++k) {
    const Eigen::VectorXd q = MeasuredQ(1, k);
    for (int j = 0; j < kNv; ++j) {
      drift_hit = drift_hit || std::abs(std::abs(at(1, k, 1 + j) - q(j)) - 0.02) < 1e-12;
    }
  }
  EXPECT_TRUE(drift_hit) << "anchor drift clamp never active";

  // Hand posture drives the fingers (hand v_ref nonzero).
  EXPECT_GT(std::abs(v_ref(3, 5, 7)) + std::abs(v_ref(3, 5, 8)), 1e-6);
}

}  // namespace
}  // namespace rtc::tsid
