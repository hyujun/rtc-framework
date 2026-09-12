// ── Reach gate: the trained formula, pinned as numbers ───────────────────────
//
// The gate is computed outside the network but the network was trained on it,
// so "roughly right" is wrong: a debounce that latches on step 4 instead of 5,
// or a ramp that uses d instead of d², produces a finite scalar in [0, 1] that
// the policy has never seen the likes of. Every boundary below is the trained
// one, and the contact-point case uses the shipped policy's own constants.

#include "rtc_controllers/inference/reach_gate.hpp"
#include "rtc_controllers/params/reach_gate_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using rtc::inference::IsGraspedByForce;
using rtc::inference::MeanTipDistance;
using rtc::inference::ReachGate;
using rtc::inference::ReachHoldState;
using rtc::inference::ReachProximity;
using rtc::inference::RotateByQuatXyzw;
using rtc::inference::UpdateReachHold;
using rtc::params::ParseReachGateParams;

constexpr double kThr = 0.5;
constexpr int kMinFingers = 2;
constexpr int kOnSteps = 5;
constexpr int kOffSteps = 100;

// thumb, index, middle, ring — the opposing digit first.
constexpr std::array<double, 4> kGrasp{1.0, 1.0, 1.0, 0.0};
constexpr std::array<double, 4> kNoContact{0.0, 0.0, 0.0, 0.0};

void ExpectRejectMentioning(const std::string& yaml, std::string_view needle) {
  try {
    static_cast<void>(ParseReachGateParams(YAML::Load(yaml)));
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(needle), std::string::npos)
        << "rejected, but for a different reason.\n  expected: " << needle
        << "\n  actual: " << what;
    return;
  }
  ADD_FAILURE() << "expected the reach_gate block to be refused, but it parsed";
}

constexpr const char* kTipsYaml = R"(
tips:
  - { link: "l_thumb_tip_link",  force_group: "thumb",  contact_obj: [-0.013565, 0.037512, 0.005946] }
  - { link: "l_index_tip_link",  force_group: "index",  contact_obj: [-0.032927, -0.022462, 0.011726] }
  - { link: "l_middle_tip_link", force_group: "middle", contact_obj: [-0.010342, -0.038602, 0.017704] }
  - { link: "l_ring_tip_link",   force_group: "ring",   contact_obj: [0.01168, -0.038178, -0.002864] }
)";

}  // namespace

// ── Grasped: asymmetric, strict ──────────────────────────────────────────────

TEST(ReachGateCore, ThumbPlusTwoFingersIsAGrasp) {
  EXPECT_TRUE(IsGraspedByForce(kGrasp, kThr, kMinFingers));
}

TEST(ReachGateCore, FingersWithoutTheThumbAreNotAGrasp) {
  const std::array<double, 4> f{0.0, 1.0, 1.0, 1.0};
  EXPECT_FALSE(IsGraspedByForce(f, kThr, kMinFingers));
}

TEST(ReachGateCore, ThumbWithOneFingerIsNotAGrasp) {
  const std::array<double, 4> f{1.0, 1.0, 0.0, 0.0};
  EXPECT_FALSE(IsGraspedByForce(f, kThr, kMinFingers));
}

TEST(ReachGateCore, TheThresholdIsStrict) {
  const std::array<double, 4> f{kThr, kThr, kThr, kThr};
  EXPECT_FALSE(IsGraspedByForce(f, kThr, kMinFingers)) << "trained as `>`, not `>=`";
}

TEST(ReachGateCore, ANonFiniteForceCannotVouchForAGrasp) {
  const std::array<double, 4> f{std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0, 1.0};
  EXPECT_FALSE(IsGraspedByForce(f, kThr, kMinFingers));
}

// ── Hold: 5 steps on, 100 steps off, one contrary step restarts the count ────

TEST(ReachGateCore, HoldLatchesOnTheFifthConsecutiveGraspedStep) {
  ReachHoldState s;
  for (int i = 1; i < kOnSteps; ++i) {
    EXPECT_FALSE(UpdateReachHold(s, true, kOnSteps, kOffSteps)) << "step " << i;
  }
  EXPECT_TRUE(UpdateReachHold(s, true, kOnSteps, kOffSteps));
}

TEST(ReachGateCore, AContraryStepRestartsTheOnCount) {
  ReachHoldState s;
  for (int i = 0; i < kOnSteps - 1; ++i) {
    static_cast<void>(UpdateReachHold(s, true, kOnSteps, kOffSteps));
  }
  static_cast<void>(UpdateReachHold(s, false, kOnSteps, kOffSteps));
  for (int i = 1; i < kOnSteps; ++i) {
    EXPECT_FALSE(UpdateReachHold(s, true, kOnSteps, kOffSteps)) << "restarted count, step " << i;
  }
  EXPECT_TRUE(UpdateReachHold(s, true, kOnSteps, kOffSteps));
}

TEST(ReachGateCore, HoldReleasesOnlyAfterTheHundredthConsecutiveEmptyStep) {
  ReachHoldState s;
  for (int i = 0; i < kOnSteps; ++i) {
    static_cast<void>(UpdateReachHold(s, true, kOnSteps, kOffSteps));
  }
  ASSERT_TRUE(s.hold);
  for (int i = 1; i < kOffSteps; ++i) {
    EXPECT_TRUE(UpdateReachHold(s, false, kOnSteps, kOffSteps)) << "released early at " << i;
  }
  EXPECT_FALSE(UpdateReachHold(s, false, kOnSteps, kOffSteps));
}

TEST(ReachGateCore, ABriefRegraspRestartsTheOffCount) {
  ReachHoldState s;
  for (int i = 0; i < kOnSteps; ++i) {
    static_cast<void>(UpdateReachHold(s, true, kOnSteps, kOffSteps));
  }
  for (int i = 0; i < kOffSteps - 1; ++i) {
    static_cast<void>(UpdateReachHold(s, false, kOnSteps, kOffSteps));
  }
  static_cast<void>(UpdateReachHold(s, true, kOnSteps, kOffSteps));  // one grasped step
  for (int i = 1; i < kOffSteps; ++i) {
    EXPECT_TRUE(UpdateReachHold(s, false, kOnSteps, kOffSteps)) << "off count not restarted";
  }
  EXPECT_FALSE(UpdateReachHold(s, false, kOnSteps, kOffSteps));
}

TEST(ReachGateCore, AValueInitialisedStateIsTheResetState) {
  const ReachHoldState s{};
  EXPECT_FALSE(s.hold);
  EXPECT_EQ(s.on_count, 0);
  EXPECT_EQ(s.off_count, 0);
}

// ── Geometry ─────────────────────────────────────────────────────────────────

TEST(ReachGateCore, RotationReadsTheQuaternionAsXyzw) {
  // +90° about z, serialised x, y, z, w. Read as w, x, y, z the same four
  // numbers are a 180° turn about (0, 1, 1)/√2, which sends x̂ to −x̂ — so this
  // case cannot pass under the other serialisation.
  const double h = std::sqrt(0.5);
  const std::array<double, 4> q{0.0, 0.0, h, h};
  const std::array<double, 3> v{1.0, 0.0, 0.0};
  std::array<double, 3> out{};
  RotateByQuatXyzw(q, v, out);
  EXPECT_NEAR(out[0], 0.0, 1e-12);
  EXPECT_NEAR(out[1], 1.0, 1e-12);
  EXPECT_NEAR(out[2], 0.0, 1e-12);
}

TEST(ReachGateCore, RotationNormalisesAScaledQuaternion) {
  const double h = 3.0 * std::sqrt(0.5);
  const std::array<double, 4> q{0.0, 0.0, h, h};
  const std::array<double, 3> v{1.0, 0.0, 0.0};
  std::array<double, 3> out{};
  RotateByQuatXyzw(q, v, out);
  EXPECT_NEAR(out[1], 1.0, 1e-12) << "a scaled quaternion must not stretch the target";
}

TEST(ReachGateCore, AZeroQuaternionYieldsNaNNotARotation) {
  const std::array<double, 4> q{0.0, 0.0, 0.0, 0.0};
  const std::array<double, 3> v{1.0, 2.0, 3.0};
  std::array<double, 3> out{};
  RotateByQuatXyzw(q, v, out);
  EXPECT_TRUE(std::isnan(out[0]) && std::isnan(out[1]) && std::isnan(out[2]));
}

TEST(ReachGateCore, TargetsRideAlongWithTheObjectPose) {
  // The shipped policy's contact points (object frame) with the object in its
  // trained nominal pose: origin (0.5, 0.05, 0.075), flipped about y — Ry(π),
  // x,y,z,w = (0, 1, 0, 0) — so a point (cx, cy, cz) lands at
  // (0.5 − cx, 0.05 + cy, 0.075 − cz).
  const std::array<double, 12> c{-0.013565, 0.037512,  0.005946, -0.032927, -0.022462, 0.011726,
                                 -0.010342, -0.038602, 0.017704, 0.01168,   -0.038178, -0.002864};
  const std::array<double, 3> p{0.5, 0.05, 0.075};
  const std::array<double, 4> q{0.0, 1.0, 0.0, 0.0};
  std::array<double, 12> tips{};
  for (std::size_t i = 0; i < 4; ++i) {
    tips[3 * i] = p[0] - c[3 * i];
    tips[(3 * i) + 1] = p[1] + c[(3 * i) + 1];
    tips[(3 * i) + 2] = p[2] - c[(3 * i) + 2];
  }
  EXPECT_NEAR(MeanTipDistance(tips, p, q, c), 0.0, 1e-12);

  // Every tip 3 cm off along a different axis: the mean is the per-tip distance.
  tips[0] += 0.03;
  tips[4] -= 0.03;
  tips[8] += 0.03;
  tips[9] -= 0.03;
  EXPECT_NEAR(MeanTipDistance(tips, p, q, c), 0.03, 1e-12);
}

TEST(ReachGateCore, MeanTipDistanceRefusesMismatchedSpans) {
  const std::array<double, 6> tips{};
  const std::array<double, 3> c{};
  const std::array<double, 3> p{};
  const std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};
  EXPECT_TRUE(std::isnan(MeanTipDistance(tips, p, q, c)));
  EXPECT_TRUE(
      std::isnan(MeanTipDistance(std::span<const double>{}, p, q, std::span<const double>{})));
}

TEST(ReachGateCore, MeanTipDistancePropagatesANonFiniteTip) {
  std::array<double, 3> tips{0.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
  const std::array<double, 3> c{};
  const std::array<double, 3> p{};
  const std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};
  EXPECT_TRUE(std::isnan(MeanTipDistance(tips, p, q, c)));
}

// ── Ramp and gate ────────────────────────────────────────────────────────────

TEST(ReachGateCore, ProximityIsAGaussianInTheMeanDistance) {
  EXPECT_DOUBLE_EQ(ReachProximity(0.0, 0.030), 1.0);
  EXPECT_NEAR(ReachProximity(0.030, 0.030), std::exp(-1.0), 1e-15);
  EXPECT_NEAR(ReachProximity(0.060, 0.030), std::exp(-4.0), 1e-15) << "squared, not linear";
}

TEST(ReachGateCore, GateIsTheLargerOfProximityAndHold) {
  EXPECT_DOUBLE_EQ(ReachGate(0.3, false), 0.3);
  EXPECT_DOUBLE_EQ(ReachGate(0.3, true), 1.0);
  EXPECT_DOUBLE_EQ(ReachGate(0.0, false), 0.0);
}

TEST(ReachGateCore, ANaNProximityIsNotLaunderedByAHold) {
  EXPECT_TRUE(std::isnan(ReachGate(std::numeric_limits<double>::quiet_NaN(), true)));
}

TEST(ReachGateCore, TheGateHasNoRatchet) {
  // Released hold → the gate falls back to proximity; nothing remembers the 1.
  ReachHoldState s;
  for (int i = 0; i < kOnSteps; ++i) {
    static_cast<void>(
        UpdateReachHold(s, IsGraspedByForce(kGrasp, kThr, kMinFingers), kOnSteps, kOffSteps));
  }
  EXPECT_DOUBLE_EQ(ReachGate(0.2, s.hold), 1.0);
  for (int i = 0; i < kOffSteps; ++i) {
    static_cast<void>(
        UpdateReachHold(s, IsGraspedByForce(kNoContact, kThr, kMinFingers), kOnSteps, kOffSteps));
  }
  EXPECT_DOUBLE_EQ(ReachGate(0.2, s.hold), 0.2);
}

// ── Schema ───────────────────────────────────────────────────────────────────

TEST(ReachGateParams, DefaultsAreTheTrainedConstants) {
  const auto p = ParseReachGateParams(YAML::Load(kTipsYaml));
  ASSERT_EQ(p.tips.size(), 4U);
  EXPECT_EQ(p.tips[0].link, "l_thumb_tip_link");
  EXPECT_EQ(p.tips[0].force_group, "thumb");
  EXPECT_DOUBLE_EQ(p.tips[3].contact_obj[2], -0.002864);
  EXPECT_DOUBLE_EQ(p.tip_std, 0.030);
  EXPECT_DOUBLE_EQ(p.force_threshold, 0.5);
  EXPECT_EQ(p.min_fingers, 2);
  EXPECT_EQ(p.hold_on_steps, 5);
  EXPECT_EQ(p.hold_off_steps, 100);
}

TEST(ReachGateParams, ExplicitConstantsOverrideTheDefaults) {
  std::string yaml = kTipsYaml;
  yaml +=
      "tip_std: 0.05\nforce_threshold: 1.0\nmin_fingers: 3\nhold_on_steps: 2\nhold_off_steps: 7\n";
  const auto p = ParseReachGateParams(YAML::Load(yaml));
  EXPECT_DOUBLE_EQ(p.tip_std, 0.05);
  EXPECT_DOUBLE_EQ(p.force_threshold, 1.0);
  EXPECT_EQ(p.min_fingers, 3);
  EXPECT_EQ(p.hold_on_steps, 2);
  EXPECT_EQ(p.hold_off_steps, 7);
}

TEST(ReachGateParams, RejectsAMissingBlock) {
  EXPECT_THROW(ParseReachGateParams(YAML::Node{}), std::invalid_argument);
}

TEST(ReachGateParams, RejectsFewerThanTwoTips) {
  ExpectRejectMentioning(R"(
tips:
  - { link: "a", force_group: "g", contact_obj: [0, 0, 0] }
)",
                         "at least two tips");
}

TEST(ReachGateParams, RejectsATipWithoutAForceGroup) {
  ExpectRejectMentioning(R"(
tips:
  - { link: "a", contact_obj: [0, 0, 0] }
  - { link: "b", force_group: "g", contact_obj: [0, 0, 0] }
)",
                         "missing `force_group`");
}

TEST(ReachGateParams, RejectsAContactPointThatIsNotThreeNumbers) {
  ExpectRejectMentioning(R"(
tips:
  - { link: "a", force_group: "f", contact_obj: [0, 0] }
  - { link: "b", force_group: "g", contact_obj: [0, 0, 0] }
)",
                         "contact_obj: [x, y, z]");
  ExpectRejectMentioning(R"(
tips:
  - { link: "a", force_group: "f", contact_obj: [0, .nan, 0] }
  - { link: "b", force_group: "g", contact_obj: [0, 0, 0] }
)",
                         "contact_obj[1] must be finite");
}

TEST(ReachGateParams, RejectsARepeatedForceGroup) {
  ExpectRejectMentioning(R"(
tips:
  - { link: "a", force_group: "g", contact_obj: [0, 0, 0] }
  - { link: "b", force_group: "g", contact_obj: [0, 0, 0] }
)",
                         "repeats force_group 'g'");
}

TEST(ReachGateParams, RejectsARepeatedLink) {
  ExpectRejectMentioning(R"(
tips:
  - { link: "a", force_group: "f", contact_obj: [0, 0, 0] }
  - { link: "a", force_group: "g", contact_obj: [0, 0, 0] }
)",
                         "repeats link 'a'");
}

TEST(ReachGateParams, RejectsANonPositiveTipStd) {
  std::string yaml = kTipsYaml;
  yaml += "tip_std: 0.0\n";
  ExpectRejectMentioning(yaml, "tip_std must be");
}

TEST(ReachGateParams, RejectsMinFingersBeyondTheOpposedTips) {
  std::string yaml = kTipsYaml;
  yaml += "min_fingers: 4\n";
  ExpectRejectMentioning(yaml, "min_fingers must be in [1, 3]");
}

TEST(ReachGateParams, RejectsAZeroStepCount) {
  std::string yaml = kTipsYaml;
  yaml += "hold_off_steps: 0\n";
  ExpectRejectMentioning(yaml, "must both be >= 1");
}

TEST(ReachGateParams, APresentButUnparseableConstantIsRefusedNotDefaulted) {
  std::string yaml = kTipsYaml;
  yaml += "hold_on_steps: five\n";
  ExpectRejectMentioning(yaml, "'hold_on_steps' is present but does not parse");
}
