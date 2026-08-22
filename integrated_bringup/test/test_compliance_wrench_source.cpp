// ── test_compliance_wrench_source.cpp ────────────────────────────────────────
// #469 S1: the pull → compliance-wrench adapter, driven end to end from a known
// external load through the SHIPPED iiwa7_leap profile.
//
// WHAT THIS ADDS OVER THE S0 TESTS. Those pin `PullEstimate` itself: a known
// load L on the fingertip lane comes back as P∥L. This file continues the same
// chain one step further — into the tuple the compliance core actually consumes
// — and pins the three things the adapter decides and the estimator does not:
// which components of the estimate become the force half (and that the torque
// half is zero rather than synthesised), when a tick is WITHHELD from the
// pipeline, and when `ComplianceFaults::quality_low` is raised. That last one
// has had no producer at all until now (compliance-conventions.md §5: "자리만
// 있고 현재 아무도 세우지 않는다"), so every branch of it is new behaviour and
// none of it is covered anywhere else.
//
// The load, the pinch axis, the fingertip rotations and why none of them are
// axis-aligned are documented in pull_known_load_fixture.hpp.
// ─────────────────────────────────────────────────────────────────────────────
#include "integrated_bringup/support/compliance_wrench_source.hpp"
#include "pull_known_load_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::ComplianceWrenchSource;
using integrated_bringup::ConfigurePullEstimatorWiring;
using integrated_bringup::FromPullEstimate;
using integrated_bringup::PullEstimatorWiring;
using integrated_bringup::VirtualTcpResult;
using integrated_bringup::WrenchSourceVerdict;
namespace fx = integrated_bringup::testfx;

constexpr int kSettleTicks = 1500;

// Where the virtual TCP sits. Deliberately unlike the wrench in every
// component, so a verdict that crossed the two fields would be visible.
const Eigen::Vector3d kApplyPoint(0.34, 0.05, 0.19);

VirtualTcpResult MakeVtcp(const Eigen::Vector3d& p, bool valid = true) {
  VirtualTcpResult v;
  v.world_pose = pinocchio::SE3::Identity();
  v.world_pose.translation() = p;
  v.valid = valid;
  return v;
}

class PullWrenchSource : public ::testing::Test {
 protected:
  void SetUp() override {
    const integrated_bringup::DemoSharedConfig cfg =
        fx::LoadShippedPullProfile(RTC_DEMO_SHARED_CONFIG_DIR);
    ASSERT_NO_THROW(ConfigurePullEstimatorWiring(
        cfg, fx::kPullRateHz, std::span<const std::string>(fx::kPullTipLinks), w_));
    ASSERT_TRUE(w_.enabled());
    ASSERT_EQ(w_.thumb_contact, 0);
  }

  PullEstimatorWiring w_;
};

// ── 1. The load reaches the pipeline as a wrench, unrotated and un-negated ──
//
// The acceptance core of S1. The load is diagonal with three distinct
// magnitudes, so a swapped or duplicated component cannot reproduce it, and the
// answer is compared against the same hand-derived P∥L the S0 tests use.
TEST_F(PullWrenchSource, KnownLoadBecomesTheWrenchHandedToThePipeline) {
  const rtc::grasp::PullEstimate& est =
      fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks);
  ASSERT_TRUE(est.valid) << "invalid_reason = " << static_cast<int>(est.invalid_reason);

  const WrenchSourceVerdict v = FromPullEstimate(w_, MakeVtcp(kApplyPoint));

  EXPECT_TRUE(v.publish);
  EXPECT_FALSE(v.quality_low);
  EXPECT_EQ(v.reason, 0);

  EXPECT_NEAR(v.wrench[0], fx::kExpectedInPlane.x(), 1e-3);
  EXPECT_NEAR(v.wrench[1], fx::kExpectedInPlane.y(), 1e-3);
  EXPECT_NEAR(v.wrench[2], fx::kExpectedInPlane.z(), 1e-3);

  // The torque half is a measurement of zero, not an unset field: the estimator
  // projects onto the pinch plane and produces no moment. A lever arm exists
  // here (the apply point is 0.4 m from the origin) so anything that
  // synthesised r × f would land far from zero.
  EXPECT_DOUBLE_EQ(v.wrench[3], 0.0);
  EXPECT_DOUBLE_EQ(v.wrench[4], 0.0);
  EXPECT_DOUBLE_EQ(v.wrench[5], 0.0);
  EXPECT_GT(kApplyPoint.norm() * fx::kExpectedInPlane.norm(), 1.0)
      << "the apply point is too close to the origin for the zero-torque check to bite";

  // The application point is the virtual TCP, carried through untouched.
  EXPECT_NEAR((v.p_apply - kApplyPoint).norm(), 0.0, 1e-12);
}

// ── 2. An invalid estimate is WITHHELD, and says why ────────────────────────
//
// D-A7: withholding is the staleness mechanism. Publishing a zero wrench here
// would be a *fresh* sample of zero — the pipeline's age would never grow, the
// §10.6 fade would never start, and `wrench_timeout` would never be reported.
TEST_F(PullWrenchSource, InvalidEstimateIsWithheldFromThePipeline) {
  fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks);
  ASSERT_TRUE(FromPullEstimate(w_, MakeVtcp(kApplyPoint)).publish) << "precondition";

  // Drop the thumb — a `required_roles` tip — through the real staging path.
  std::array<fx::FtSample, 4> samples = fx::PullLaneSamples(fx::kLoad);
  samples[0].valid = false;
  const rtc::grasp::PullEstimate& est = fx::RunPullTicks(w_, samples, /*grasp_detected=*/false, 10);
  ASSERT_FALSE(est.valid);

  const WrenchSourceVerdict v = FromPullEstimate(w_, MakeVtcp(kApplyPoint));
  EXPECT_FALSE(v.publish);
  EXPECT_EQ(v.reason,
            static_cast<std::uint8_t>(rtc::grasp::PullInvalidReason::kRequiredContactMissing));
  // The timeout lane owns this tick's fault, not the quality lane: the source
  // is behaving correctly by declining to speak.
  EXPECT_FALSE(v.quality_low);
}

// ── 3. A usable estimate with nowhere to put it degrades NOW ────────────────
//
// An invalid virtual TCP resolves to the identity, whose origin is finite and
// completely wrong as a place to hang a force. Withholding is right; staying
// silent about it is not, because the consumer would not hear about it until
// `wrench_timeout` seconds later.
TEST_F(PullWrenchSource, InvalidApplicationPointWithholdsAndDegradesImmediately) {
  const rtc::grasp::PullEstimate& est =
      fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks);
  ASSERT_TRUE(est.valid);

  const WrenchSourceVerdict v = FromPullEstimate(w_, MakeVtcp(kApplyPoint, /*valid=*/false));
  EXPECT_FALSE(v.publish);
  EXPECT_TRUE(v.quality_low);
  // reason stays 0 because the PULL was fine — that pair (quality_low with
  // reason 0) is the signature no ordinary invalid tick can produce.
  EXPECT_EQ(v.reason, 0);
}

// ── 4. A non-finite application point is caught here, not downstream ────────
//
// NUM-7: every magnitude comparison is false for NaN, so nothing else on this
// path would stop it. `WrenchInput::Set` would drop the sample and leave a
// counter as the only trace while the arm ran on a fading stale wrench.
//
// Not reachable from `ComputeVirtualTcp` today — #316 made that function fail
// to identity on a non-finite pose — so this gate is a boundary defence rather
// than a fix, and the verdict is constructed directly to reach it. The class of
// bug is not hypothetical: that same function shipped returning `valid=true`
// with a NaN pose, and its consumer latched it as a control point.
TEST_F(PullWrenchSource, NonFiniteApplicationPointWithholdsAndDegradesImmediately) {
  const rtc::grasp::PullEstimate& est =
      fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks);
  ASSERT_TRUE(est.valid);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const WrenchSourceVerdict v =
      FromPullEstimate(w_, MakeVtcp(Eigen::Vector3d(kApplyPoint.x(), nan, kApplyPoint.z())));
  EXPECT_FALSE(v.publish);
  EXPECT_TRUE(v.quality_low);
}

// ── 5. A controller configured without this source is silent, not faulted ───
TEST(PullWrenchSourceDisabled, DisabledWiringPublishesNothingAndFaultsNothing) {
  PullEstimatorWiring w;  // never configured → no estimator
  ASSERT_FALSE(w.enabled());

  const WrenchSourceVerdict v = FromPullEstimate(w, MakeVtcp(kApplyPoint));
  EXPECT_FALSE(v.publish);
  EXPECT_FALSE(v.quality_low) << "a source that was never configured must not hold the "
                                 "controller in DEGRADED for its whole life";
  EXPECT_EQ(v.reason, 0);
  for (std::size_t i = 0; i < v.wrench.size(); ++i) {
    EXPECT_DOUBLE_EQ(v.wrench[i], 0.0) << "component " << i;
  }
}

// ── 6. Slip risk degrades the estimate without silencing it ─────────────────
//
// The wrench is still the best available reading, so it keeps flowing; what
// changes is that the state machine is told to stop trusting it fully. Grip
// dropped to 2 N against the same load puts friction utilisation past 1.0.
TEST_F(PullWrenchSource, SlipRiskRaisesQualityLowButStillPublishes) {
  const rtc::grasp::PullEstimate& est =
      fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks, /*squeeze=*/2.0);
  ASSERT_TRUE(est.valid) << "invalid_reason = " << static_cast<int>(est.invalid_reason);
  ASSERT_TRUE(est.slip_risk) << "utilisation = " << est.max_friction_utilization;

  const WrenchSourceVerdict v = FromPullEstimate(w_, MakeVtcp(kApplyPoint));
  EXPECT_TRUE(v.publish) << "a slipping grasp still reports the best force it has";
  EXPECT_TRUE(v.quality_low);
}

// ── 7. A saturated tip degrades the estimate too ────────────────────────────
TEST_F(PullWrenchSource, SaturatedTipRaisesQualityLow) {
  fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks);

  // Push one non-required tip past the profile's 25 N per-axis saturation gate.
  // Thumb + index still clear `min_valid_contacts: 2`, so the tick stays valid
  // and the flag is what changes.
  std::array<fx::FtSample, 4> samples = fx::PullLaneSamples(fx::kLoad);
  samples[2].force = {30.0F, 0.0F, 0.0F};
  const rtc::grasp::PullEstimate& est = fx::RunPullTicks(w_, samples, /*grasp_detected=*/false, 50);
  ASSERT_TRUE(est.valid) << "invalid_reason = " << static_cast<int>(est.invalid_reason);
  ASSERT_TRUE(est.any_saturated);

  const WrenchSourceVerdict v = FromPullEstimate(w_, MakeVtcp(kApplyPoint));
  EXPECT_TRUE(v.publish);
  EXPECT_TRUE(v.quality_low);
}

// ── 8. Leakage that could account for the whole signal degrades it ──────────
//
// `leakage_bound` is Σ|f_n,i|·sin(δθ): an upper bound on how much of the grip
// could be masquerading as pull. The adapter compares it to the estimate rather
// than to a tuned threshold, so the condition it states is "this reading could
// be entirely leakage" — at which point not even its direction survives. Here a
// firm 10 N grip carries a 0.3 N in-plane load, and the bound is ~0.7 N.
TEST_F(PullWrenchSource, LeakageBoundThatSwallowsTheEstimateRaisesQualityLow) {
  // In-plane unit vector: (2,1,2)·(1,-2,0) = 0.
  const Eigen::Vector3d in_plane = Eigen::Vector3d(1.0, -2.0, 0.0).normalized();
  const Eigen::Vector3d faint_load = 0.8 * fx::kPinchNormal + 0.3 * in_plane;

  const rtc::grasp::PullEstimate& est =
      fx::RunPullLoad(w_, faint_load, /*grasp_detected=*/false, kSettleTicks);
  ASSERT_TRUE(est.valid) << "invalid_reason = " << static_cast<int>(est.invalid_reason);
  // Isolate the predicate: neither of the other two quality inputs is set, so a
  // pass here is the leakage comparison and nothing else.
  ASSERT_FALSE(est.slip_risk);
  ASSERT_FALSE(est.any_saturated);
  ASSERT_GT(est.leakage_bound, est.magnitude)
      << "fixture no longer puts the estimate under its own leakage bound";

  const WrenchSourceVerdict v = FromPullEstimate(w_, MakeVtcp(kApplyPoint));
  EXPECT_TRUE(v.publish);
  EXPECT_TRUE(v.quality_low);
}

// ── 9. The enum carries no value it cannot serve ────────────────────────────
//
// `kMomentumObserver` is absent on purpose (#469 S6 adds it with its adapter):
// a declared-but-unimplemented value turns a config typo into a runtime throw
// that reads as a controller bug, where a value that simply does not exist is
// rejected by the same path as any other misspelling.
TEST(PullWrenchSourceEnum, OnlyTheImplementedSourceIsDeclared) {
  EXPECT_EQ(static_cast<std::uint8_t>(ComplianceWrenchSource::kPullEstimator), 0);
}

}  // namespace
