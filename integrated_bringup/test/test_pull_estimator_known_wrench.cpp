// ── test_pull_estimator_known_wrench.cpp ─────────────────────────────────────
// #177 crit#4, consumer half: what the SHIPPED iiwa7_leap profile does with a
// fingertip force lane that is carrying a known external load.
//
// WHY THIS EXISTS SEPARATELY FROM test_pull_estimator_wiring.cpp. That file
// builds its own inline YAML with `force_sign: 1.0`, a fixed plane normal and
// identity fingertip rotations. It is a good unit test of the wiring and a poor
// test of the deployed system: the value that actually decides which way the
// arm will be pushed — `force_sign: -1.0`, four times over in
// config/iiwa7_leap/controllers/demo_shared.yaml — is not read by any test in
// this repo, so deleting it ships green. That number exists because the sim
// fingertip lane is deliberately env-on-link (rtc_mujoco_sim negates MuJoCo's
// geom1-on-environment netforce to the ROS wrench convention) while the
// estimator sums finger-on-object. The two conventions meet in that YAML and
// nowhere else, which makes the YAML the thing to pin.
//
// THE OTHER HALF. rtc_mujoco_sim's test_contact_wrench_known_load.cpp hangs a
// known wrench on a pinched free body and proves the lane's world-frame sum is
// the total non-contact load L on the object:
//
//     Sum_i R_i f_i^link = L        (L = applied wrench + m g)
//
// This file starts from that L. Newton on the object gives the finger-on-object
// forces the estimator is defined to sum, Sum_i c_i = -L, and the estimator's
// law F = -P|| (Sum_i c_i + m g) with the profile's `gravity_force: [0,0,0]`
// therefore has to produce
//
//     F = -P||(-L) = +P|| L
//
// so the answer is the in-plane part of the load, pointing the way the
// environment pulls. Sign, frame and magnitude fall out of that one comparison.
//
// The two files together cover both directions of the seam: flip the sim's
// negation and the other file goes red; flip the shipped force_sign and this
// one does.
// ─────────────────────────────────────────────────────────────────────────────
#include "integrated_bringup/support/pull_estimator_wiring.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::ConfigurePullEstimatorWiring;
using integrated_bringup::DemoSharedConfig;
using integrated_bringup::PullEstimatorWiring;

constexpr double kRateHz = 500.0;
constexpr double kDt = 1.0 / kRateHz;

// The sim profile. It is the one whose pull_estimator block is fed by the
// simulator's contact-wrench lane, so it is the one that carries the sign flip.
constexpr const char* kProfile = "iiwa7_leap";

// tree-model `leap` tip_links order (config/iiwa7_leap sim.yaml), which is the
// fingertip-slot order the demo controllers hand to the wiring at configure.
const std::vector<std::string> kTipLinks = {"thumb_tip_head", "index_tip_head", "middle_tip_head",
                                            "ring_tip_head"};

// Pinch axis. Deliberately off every coordinate axis and with no repeated or
// zero component, so a permuted or partially-identity plane projection cannot
// reproduce it. Exactly unit: |(2,1,2)| = 3.
const Eigen::Vector3d kPinchNormal = Eigen::Vector3d(2.0, 1.0, 2.0) / 3.0;

// The known load: the total non-contact force the environment puts on the
// grasped object, i.e. the L that the simulator half measures.
const Eigen::Vector3d kLoad(2.0, -5.0, 3.0);

// Expected estimate = P|| L = L - n (n.L), worked out by hand so the oracle is
// not a second copy of the implementation's expression:
//   n.L    = (2*2 + 1*(-5) + 2*3) / 3 = 5/3
//   P|| L  = (2,-5,3) - (5/3)(2,1,2)/3 = (2 - 10/9, -5 - 5/9, 3 - 10/9)
const Eigen::Vector3d kExpectedInPlane(8.0 / 9.0, -50.0 / 9.0, 17.0 / 9.0);

// Grip force along the pinch axis [N]. Large enough to clear the profile's
// 0.5 N contact hysteresis with margin at every tip and to keep friction
// utilisation under the slip threshold, small enough to stay far below the
// profile's 25 N saturation gate.
constexpr double kSqueezeN = 10.0;

// The wiring reads only `.valid` and `.force[0..2]` off each controller's
// private FingertipSensorData, so a local stand-in is the whole contract.
// float, like the real one — which is why the tolerances below are 1e-4 and
// not 1e-12.
struct FtSample {
  std::array<float, 3> force{};
  bool valid{false};
};

// Three distinct, non-trivial fingertip orientations. Identity rotations here
// would let a transposed or dropped R_i pass unnoticed, which is the same hole
// the simulator-half fixture closes with its rotated pad frames.
const std::array<Eigen::Matrix3d, 4> kTipRotations = {
    Eigen::Matrix3d(
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, -2.0).normalized()).toRotationMatrix()),
    Eigen::Matrix3d(
        Eigen::AngleAxisd(-1.1, Eigen::Vector3d(-2.0, 1.0, 2.0).normalized()).toRotationMatrix()),
    Eigen::Matrix3d(
        Eigen::AngleAxisd(2.0, Eigen::Vector3d(1.0, -1.0, 3.0).normalized()).toRotationMatrix()),
    Eigen::Matrix3d(Eigen::Matrix3d::Identity())};

// Fingertip positions that make the observed pinch axis exactly kPinchNormal:
// the wiring derives n from (centroid of the touching non-thumb tips - p_thumb),
// so index and middle straddle a point 60 mm from the thumb along it. `w` is any
// unit vector orthogonal to the axis — (1,-2,0)/sqrt(5) works since
// (2,1,2).(1,-2,0) = 0 — so the straddle cancels out of the centroid exactly.
std::array<Eigen::Vector3d, 4> TipPositions() {
  const Eigen::Vector3d thumb(0.30, 0.02, 0.15);
  const Eigen::Vector3d centroid = thumb + 0.06 * kPinchNormal;
  const Eigen::Vector3d straddle = 0.02 * Eigen::Vector3d(1.0, -2.0, 0.0).normalized();
  return {thumb, centroid + straddle, centroid - straddle,
          Eigen::Vector3d(0.0, 0.0, 0.0)};  // ring: never touching, never read
}

DemoSharedConfig LoadShippedProfile() {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + kProfile + "/controllers/demo_shared.yaml";
  const YAML::Node root = YAML::LoadFile(path);
  DemoSharedConfig cfg;
  integrated_bringup::ApplyDemoSharedConfig(root["demo_shared"], cfg);
  return cfg;
}

// The finger-on-object forces of a three-finger pinch that is carrying `load`.
//
// Newton on the object fixes only their SUM (= -load); the split into a grip
// part and a share of the load is this fixture's choice, and any split that
// clears the gates would do. The thumb pushes along +n, the two opposing tips
// share -n, so the grip cancels out of the sum and only the load survives.
std::array<Eigen::Vector3d, 3> PinchForces(const Eigen::Vector3d& load) {
  const Eigen::Vector3d share = load / 3.0;
  return {kSqueezeN * kPinchNormal - share, -0.5 * kSqueezeN * kPinchNormal - share,
          -0.5 * kSqueezeN * kPinchNormal - share};
}

// Turn those into what the SIM LANE publishes for them: env-on-link (i.e. the
// negative of finger-on-object) expressed in each fingertip's own frame. This
// is the step the shipped `force_sign: -1.0` has to undo.
std::array<FtSample, 4> LaneSamplesFor(const Eigen::Vector3d& load) {
  const auto contact = PinchForces(load);
  std::array<FtSample, 4> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    const Eigen::Vector3d link = kTipRotations[i].transpose() * (-contact[i]);
    out[i].force = {static_cast<float>(link.x()), static_cast<float>(link.y()),
                    static_cast<float>(link.z())};
    out[i].valid = true;
  }
  out[3].valid = false;  // ring is not in this grasp
  return out;
}

// Run `ticks` updates through the production staging path with the lane
// carrying `load`. Several ticks are needed whatever the assertion: the pinch
// axis comes from the previous tick's touch hysteresis (one-tick lag by
// design), and force_filtered is behind a 5 Hz Bessel.
const rtc::grasp::PullEstimate& RunLoaded(PullEstimatorWiring& w, const Eigen::Vector3d& load,
                                          bool grasp_detected, int ticks) {
  const auto samples = LaneSamplesFor(load);
  const auto positions = TipPositions();
  const std::array<bool, 4> pose_valid = {true, true, true, false};
  rtc::grasp::PullEstimateData out{};
  for (int t = 0; t < ticks; ++t) {
    integrated_bringup::StageFkPullTickAndPublish<FtSample>(
        w, std::span<const FtSample>(samples), std::span<const Eigen::Matrix3d>(kTipRotations),
        std::span<const Eigen::Vector3d>(positions), std::span<const bool>(pose_valid),
        /*num_active_fingertips=*/4, grasp_detected, kDt, out);
  }
  return w.estimator->estimate();
}

class ShippedPullEstimator : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_ = LoadShippedProfile();
    ASSERT_NO_THROW(
        ConfigurePullEstimatorWiring(cfg_, kRateHz, std::span<const std::string>(kTipLinks), w_));
    ASSERT_TRUE(w_.enabled()) << "the shipped " << kProfile << " profile disabled the estimator";
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
        << kProfile << " tip " << w_.roles[i]
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

// ── 2. Known load → P|| L, sign, frame and magnitude at once ────────────────
TEST_F(ShippedPullEstimator, KnownLoadOnTheLaneBecomesItsInPlanePart) {
  const rtc::grasp::PullEstimate& est = RunLoaded(w_, kLoad, /*grasp_detected=*/false, 1500);

  ASSERT_TRUE(est.valid) << "invalid_reason = " << static_cast<int>(est.invalid_reason);
  EXPECT_EQ(est.valid_contact_count, 3);
  EXPECT_FALSE(est.slip_risk);
  EXPECT_FALSE(est.any_saturated);
  EXPECT_FALSE(est.baseline_applied) << "no grasp edge was raised, so nothing may be subtracted";

  // The observed pinch axis is the one the fixture built.
  EXPECT_NEAR((est.plane_normal - kPinchNormal).norm(), 0.0, 1e-6);

  // force_raw is the law's output before the output filter — exact up to the
  // float32 the fingertip lane is carried in.
  EXPECT_NEAR(est.force_raw.x(), kExpectedInPlane.x(), 1e-4);
  EXPECT_NEAR(est.force_raw.y(), kExpectedInPlane.y(), 1e-4);
  EXPECT_NEAR(est.force_raw.z(), kExpectedInPlane.z(), 1e-4);

  // force_filtered is what a consumer reads (#469 D-A2 feeds exactly this into
  // the admittance law), so it is checked too, after the 5 Hz Bessel settles.
  EXPECT_NEAR(est.force_filtered.x(), kExpectedInPlane.x(), 1e-3);
  EXPECT_NEAR(est.force_filtered.y(), kExpectedInPlane.y(), 1e-3);
  EXPECT_NEAR(est.force_filtered.z(), kExpectedInPlane.z(), 1e-3);

  // The out-of-plane part really was removed, and really was there to remove —
  // otherwise P|| would be untested and the estimate would just be L.
  EXPECT_NEAR(est.force_filtered.dot(kPinchNormal), 0.0, 1e-3);
  EXPECT_GT((kLoad - kExpectedInPlane).norm(), 0.5)
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
  const Eigen::Vector3d loaded = preload + kLoad;

  // Settle on the preload with no grasp edge, so nothing is captured yet.
  const rtc::grasp::PullEstimate& before = RunLoaded(w_, preload, /*grasp_detected=*/false, 1500);
  ASSERT_TRUE(before.valid);
  ASSERT_FALSE(before.baseline_applied);

  // Rising edge: the next valid tick snapshots the preload.
  RunLoaded(w_, preload, /*grasp_detected=*/true, 1);
  EXPECT_TRUE(w_.estimator->estimate().baseline_applied);

  // Now change the load. What survives the subtraction is the change, and the
  // change was chosen to be the same kLoad as the test above — so the expected
  // answer is the same vector, reached down the deployed path.
  const rtc::grasp::PullEstimate& after = RunLoaded(w_, loaded, /*grasp_detected=*/true, 1500);
  ASSERT_TRUE(after.valid);
  EXPECT_TRUE(after.baseline_applied);

  EXPECT_NEAR(after.force_raw.x(), kExpectedInPlane.x(), 1e-4);
  EXPECT_NEAR(after.force_raw.y(), kExpectedInPlane.y(), 1e-4);
  EXPECT_NEAR(after.force_raw.z(), kExpectedInPlane.z(), 1e-4);
  EXPECT_NEAR(after.force_filtered.x(), kExpectedInPlane.x(), 1e-3);
  EXPECT_NEAR(after.force_filtered.y(), kExpectedInPlane.y(), 1e-3);
  EXPECT_NEAR(after.force_filtered.z(), kExpectedInPlane.z(), 1e-3);
}

}  // namespace
