// ── test_pull_estimator_known_wrench.cpp ─────────────────────────────────────
// #177 crit#4, consumer half: what the SHIPPED iiwa7_leap profile does with a
// fingertip force lane that is carrying a known external load.
//
// WHY THIS EXISTS SEPARATELY FROM test_pull_estimator_wiring.cpp. That file
// builds its own inline YAML with `force_sign: 1.0`, a fixed plane normal and
// identity fingertip rotations. It is a good unit test of the wiring and a poor
// test of the deployed system: the value that actually decides which way the
// arm will be pushed — `force_sign: -1.0`, four times over in
// config/iiwa7_leap/controllers/demo_shared.yaml — was read by no test in this
// repo, so deleting it shipped green (measured: 217 tests across five suites
// stay passing with it flipped). That number exists because the sim fingertip
// lane is deliberately env-on-link (rtc_mujoco_sim negates MuJoCo's
// geom1-on-environment netforce to the ROS wrench convention) while the
// estimator sums finger-on-object. The two conventions meet in that YAML and
// nowhere else, which makes the YAML the thing to pin.
//
// The physics, the fixture and how the simulator half composes with this one
// are documented in pull_known_load_fixture.hpp.
// ─────────────────────────────────────────────────────────────────────────────
#include "pull_known_load_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::ConfigurePullEstimatorWiring;
using integrated_bringup::DemoSharedConfig;
using integrated_bringup::PullEstimatorWiring;
namespace fx = integrated_bringup::testfx;

// Ticks to settle: past the estimator's one-tick axis lag and well past the
// profile's 5 Hz output filter at 500 Hz.
constexpr int kSettleTicks = 1500;

class ShippedPullEstimator : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_ = fx::LoadShippedPullProfile(RTC_DEMO_SHARED_CONFIG_DIR);
    ASSERT_NO_THROW(ConfigurePullEstimatorWiring(
        cfg_, fx::kPullRateHz, std::span<const std::string>(fx::kPullTipLinks), w_));
    ASSERT_TRUE(w_.enabled()) << "the shipped " << fx::kPullProfile
                              << " profile disabled the estimator";
    ASSERT_EQ(w_.num_contacts, 4);
    // Everything below indexes tips by this order, so it is asserted rather
    // than assumed: a reordered tip_names block must fail loudly here instead
    // of quietly re-labelling which tip is the thumb.
    const std::vector<std::string> expected_roles = {"thumb", "index", "middle", "ring"};
    ASSERT_EQ(w_.roles, expected_roles);
    for (int k = 0; k < w_.num_contacts; ++k) {
      ASSERT_EQ(w_.slot[static_cast<std::size_t>(k)], k) << "contact " << k;
    }
    ASSERT_EQ(w_.thumb_contact, 0);
    ASSERT_EQ(w_.normal_source, integrated_bringup::PullPlaneNormalSource::kPinchGeometry);
  }

  DemoSharedConfig cfg_;
  PullEstimatorWiring w_;
};

// ── 1. The shipped sign flip, stated where a reader can find it ─────────────
//
// The two behavioural tests below already fail if this value moves, but they
// fail as a sign error in a vector. This one names the file and the field.
TEST_F(ShippedPullEstimator, EveryTipInvertsTheEnvOnLinkSimLane) {
  ASSERT_EQ(cfg_.num_pull_contacts, 4);
  for (std::size_t i = 0; i < static_cast<std::size_t>(cfg_.num_pull_contacts); ++i) {
    EXPECT_DOUBLE_EQ(cfg_.pull_contacts[i].force_sign, -1.0)
        << fx::kPullProfile << " tip " << w_.roles[i]
        << ": the simulator publishes env-on-link contact wrenches and the estimator "
           "sums finger-on-object, so this profile must invert. Dropping it inverts "
           "every f_n gate — the 2026-07-22 p1b failure, where a solid grasp published "
           "an all-zero estimate.";
  }
  // The gravity model must stay off, or the identity this file tests acquires a
  // second, silently-configured term (the profile keeps it zero because
  // use_baseline_subtraction already removes in-plane gravity).
  EXPECT_TRUE(cfg_.pull_estimator_params.gravity_force.isZero(0.0));
}

// ── 2. Known load → P∥L, sign, frame and magnitude at once ─────────────────
TEST_F(ShippedPullEstimator, KnownLoadOnTheLaneBecomesItsInPlanePart) {
  const rtc::grasp::PullEstimate& est =
      fx::RunPullLoad(w_, fx::kLoad, /*grasp_detected=*/false, kSettleTicks);

  ASSERT_TRUE(est.valid) << "invalid_reason = " << static_cast<int>(est.invalid_reason);
  EXPECT_EQ(est.valid_contact_count, 3);
  EXPECT_FALSE(est.slip_risk);
  EXPECT_FALSE(est.any_saturated);
  EXPECT_FALSE(est.baseline_applied) << "no grasp edge was raised, so nothing may be subtracted";

  // The observed pinch axis is the one the fixture built.
  EXPECT_NEAR((est.plane_normal - fx::kPinchNormal).norm(), 0.0, 1e-6);

  // force_raw is the law's output before the output filter — exact up to the
  // float32 the fingertip lane is carried in.
  EXPECT_NEAR(est.force_raw.x(), fx::kExpectedInPlane.x(), 1e-4);
  EXPECT_NEAR(est.force_raw.y(), fx::kExpectedInPlane.y(), 1e-4);
  EXPECT_NEAR(est.force_raw.z(), fx::kExpectedInPlane.z(), 1e-4);

  // force_filtered is what a consumer reads (#469 D-A2 feeds exactly this into
  // the admittance law), so it is checked too, after the 5 Hz Bessel settles.
  EXPECT_NEAR(est.force_filtered.x(), fx::kExpectedInPlane.x(), 1e-3);
  EXPECT_NEAR(est.force_filtered.y(), fx::kExpectedInPlane.y(), 1e-3);
  EXPECT_NEAR(est.force_filtered.z(), fx::kExpectedInPlane.z(), 1e-3);

  // The out-of-plane part really was removed, and really was there to remove —
  // otherwise P∥ would be untested and the estimate would just be L.
  EXPECT_NEAR(est.force_filtered.dot(fx::kPinchNormal), 0.0, 1e-3);
  EXPECT_GT((fx::kLoad - fx::kExpectedInPlane).norm(), 0.5)
      << "the load is nearly in-plane, so this fixture does not exercise the projection";
}

// ── 3. With the profile's baseline armed, the estimate is the CHANGE in load ─
//
// The shipped profile runs `use_baseline_subtraction: true`, so the absolute
// reading above is not the path the robot takes: the snapshot arms on the grasp
// rising edge and everything after it is a delta. Pinning only the absolute
// case would leave the deployed path uncovered.
TEST_F(ShippedPullEstimator, BaselineArmedOnTheGraspEdgeMakesTheEstimateADelta) {
  ASSERT_TRUE(w_.use_baseline) << "the shipped profile no longer subtracts a baseline — "
                                  "this test is asserting the wrong contract";

  const Eigen::Vector3d preload(1.0, 1.0, -2.0);
  const Eigen::Vector3d loaded = preload + fx::kLoad;

  // Settle on the preload with no grasp edge, so nothing is captured yet.
  const rtc::grasp::PullEstimate& before =
      fx::RunPullLoad(w_, preload, /*grasp_detected=*/false, kSettleTicks);
  ASSERT_TRUE(before.valid);
  ASSERT_FALSE(before.baseline_applied);

  // Rising edge: the next valid tick snapshots the preload.
  fx::RunPullLoad(w_, preload, /*grasp_detected=*/true, 1);
  EXPECT_TRUE(w_.estimator->estimate().baseline_applied);

  // Now change the load. What survives the subtraction is the change, and the
  // change was chosen to be the same kLoad as the test above — so the expected
  // answer is the same vector, reached down the deployed path.
  const rtc::grasp::PullEstimate& after =
      fx::RunPullLoad(w_, loaded, /*grasp_detected=*/true, kSettleTicks);
  ASSERT_TRUE(after.valid);
  EXPECT_TRUE(after.baseline_applied);

  EXPECT_NEAR(after.force_raw.x(), fx::kExpectedInPlane.x(), 1e-4);
  EXPECT_NEAR(after.force_raw.y(), fx::kExpectedInPlane.y(), 1e-4);
  EXPECT_NEAR(after.force_raw.z(), fx::kExpectedInPlane.z(), 1e-4);
  EXPECT_NEAR(after.force_filtered.x(), fx::kExpectedInPlane.x(), 1e-3);
  EXPECT_NEAR(after.force_filtered.y(), fx::kExpectedInPlane.y(), 1e-3);
  EXPECT_NEAR(after.force_filtered.z(), fx::kExpectedInPlane.z(), 1e-3);
}

}  // namespace
