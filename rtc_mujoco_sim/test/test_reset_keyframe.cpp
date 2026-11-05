// ── test_reset_keyframe.cpp ──────────────────────────────────────────────────
// `reset_keyframe: <name>` — the scene's start and reset pose for the WHOLE
// qpos, free joints included (W0-F02, #976).
//
// WHY THIS EXISTS. HandleReset was `mj_resetData` plus each robot group's
// `initial_qpos` on its command joints. For a fixed-base arm that is the
// keyframe pose, because the keyframe has nothing but command joints in it.
// For a floating-base robot it is not: mj_resetData puts the free joint back
// at qpos0 — the <body pos> of the XML, usually the origin with the identity
// quaternion — and nothing ever wrote the keyframe's base pose. The robot
// started, and was reset to, a stance keyframe with its base somewhere else.
//
// WHAT MAKES THESE ASSERTIONS BITE. Every comparison against the keyframe is
// bit-exact (`==` on the full qpos vector read back from mjData against
// `key_qpos` of the compiled model), the selected keyframe is the SECOND one
// in the file so index-0 resolution fails, and each reset case first steps
// the scene until the base has visibly fallen — a reset that does nothing
// cannot pass. The control case (key unset) pins the historical behaviour
// the key leaves in place: base at qpos0, command joints from the FIRST
// keyframe.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#ifndef FLOATING_KEYFRAME_MJCF_PATH
#error "FLOATING_KEYFRAME_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

// nq = 7 (free joint) + 1 (hinge).
constexpr std::size_t kNq = 8;
constexpr std::size_t kHingeSlot = 7;
// Fixture values, mirrored so a change to the MJCF fails loudly.
constexpr double kStandHinge = 0.40;
constexpr double kOtherHinge = -0.90;

MuJoCoSimulator::Config MakeFloatingConfig(const std::string& reset_keyframe) {
  auto cfg = test::HeadlessConfig(FLOATING_KEYFRAME_MJCF_PATH, 10.0);
  cfg.groups.push_back(test::RobotGroup("arm", {"j1"}));
  cfg.reset_keyframe = reset_keyframe;
  return cfg;
}

// Steps until the base has moved, so a later "reset restored it" is not
// vacuous. No floor in the fixture: it falls under gravity.
void StepUntilBaseMoved(MuJoCoSimulator& sim, const std::vector<double>& start) {
  for (int i = 0; i < 400; ++i) {
    sim.StepForTest();
  }
  const auto moved = sim.GetQposForTest();
  ASSERT_EQ(moved.size(), kNq);
  ASSERT_GT(std::abs(moved[2] - start[2]), 0.05)
      << "positive control: the base never left its start pose";
}

// ── Keyframe lane ───────────────────────────────────────────────────────────

TEST(ResetKeyframe, StartIsTheNamedKeyframeWholeQposBitExact) {
  MuJoCoSimulator sim(MakeFloatingConfig("stand"));
  ASSERT_TRUE(sim.Initialize());

  const auto key = sim.GetKeyQposForTest("stand");
  ASSERT_EQ(key.size(), kNq) << "fixture keyframe not found";
  const auto q = sim.GetQposForTest();
  EXPECT_EQ(q, key) << "start pose is not the keyframe, bit for bit";
  // Named, not index 0: the decoy keyframe must NOT be what we are at.
  EXPECT_NE(q, sim.GetKeyQposForTest("other"));
  // And the command-joint fallback reads the SAME keyframe.
  EXPECT_DOUBLE_EQ(sim.GetPositions(0)[0], kStandHinge);
}

TEST(ResetKeyframe, ResetRestoresTheNamedKeyframeWholeQposBitExact) {
  MuJoCoSimulator sim(MakeFloatingConfig("stand"));
  ASSERT_TRUE(sim.Initialize());
  const auto key = sim.GetKeyQposForTest("stand");
  ASSERT_EQ(key.size(), kNq);

  StepUntilBaseMoved(sim, key);

  sim.ResetForTest();
  EXPECT_EQ(sim.GetQposForTest(), key) << "reset did not return the whole qpos to the keyframe";
  EXPECT_DOUBLE_EQ(sim.GetPositions(0)[0], kStandHinge);
}

TEST(ResetKeyframe, ResetIsRepeatable) {
  MuJoCoSimulator sim(MakeFloatingConfig("stand"));
  ASSERT_TRUE(sim.Initialize());
  const auto key = sim.GetKeyQposForTest("stand");
  for (int round = 0; round < 3; ++round) {
    StepUntilBaseMoved(sim, key);
    sim.ResetForTest();
    ASSERT_EQ(sim.GetQposForTest(), key) << "round " << round;
  }
}

// ── YAML initial_qpos on top of the keyframe ────────────────────────────────

// The precedence `initial_qpos` already has (YAML > keyframe) is unchanged:
// it overrides the command joints and ONLY them — the free joint stays on
// the keyframe. Both at start and after a reset.
TEST(ResetKeyframe, YamlInitialQposOverridesCommandJointsOnTopOfKeyframe) {
  auto cfg = MakeFloatingConfig("stand");
  cfg.groups[0].initial_qpos = {0.90};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  const auto key = sim.GetKeyQposForTest("stand");
  ASSERT_EQ(key.size(), kNq);

  const auto check = [&](const char* when) {
    const auto q = sim.GetQposForTest();
    ASSERT_EQ(q.size(), kNq) << when;
    for (std::size_t i = 0; i < kHingeSlot; ++i) {
      EXPECT_EQ(q[i], key[i]) << when << ": free-joint slot " << i << " left the keyframe";
    }
    EXPECT_DOUBLE_EQ(q[kHingeSlot], 0.90) << when << ": YAML must outrank the keyframe";
  };
  check("start");

  StepUntilBaseMoved(sim, key);
  sim.ResetForTest();
  check("after reset");
}

// ── Control group: key unset = historical behaviour ─────────────────────────

// Without the key, the base goes to qpos0 (XML body pose — here the origin
// and the identity quaternion, which no keyframe of the fixture has) and the
// command joints come from the FIRST keyframe. This is the behaviour every
// shipped fixed-base bringup relies on, and the one the key leaves alone.
TEST(ResetKeyframe, UnsetKeyKeepsQpos0PlusFirstKeyframeCommandJoints) {
  MuJoCoSimulator sim(MakeFloatingConfig(""));
  ASSERT_TRUE(sim.Initialize());

  const auto qpos0 = sim.GetQpos0ForTest();
  const auto key_other = sim.GetKeyQposForTest("other");
  const auto key_stand = sim.GetKeyQposForTest("stand");
  ASSERT_EQ(qpos0.size(), kNq);

  const auto check = [&](const char* when) {
    const auto q = sim.GetQposForTest();
    ASSERT_EQ(q.size(), kNq) << when;
    for (std::size_t i = 0; i < kHingeSlot; ++i) {
      EXPECT_EQ(q[i], qpos0[i]) << when << ": free-joint slot " << i << " is not qpos0";
    }
    EXPECT_DOUBLE_EQ(q[kHingeSlot], kOtherHinge) << when << ": command joint not from key 0";
    // The defect, stated as a fact about the control group: the base is at
    // NEITHER keyframe.
    EXPECT_NE(q, key_other) << when;
    EXPECT_NE(q, key_stand) << when;
  };
  check("start");

  StepUntilBaseMoved(sim, qpos0);
  sim.ResetForTest();
  check("after reset");
}

// ── Rejection lane ──────────────────────────────────────────────────────────

TEST(ResetKeyframe, UnknownKeyframeNameIsRejected) {
  MuJoCoSimulator sim(MakeFloatingConfig("no_such_key"));
  EXPECT_FALSE(sim.Initialize())
      << "an unknown reset_keyframe must fail Initialize, not fall back to qpos0";
}

TEST(ResetKeyframe, KeyframeOnSceneWithoutOneIsRejected) {
  auto cfg = test::MakeMinimalConfig();  // MINIMAL_MJCF_PATH has no <keyframe>
  cfg.reset_keyframe = "home";
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

}  // namespace
}  // namespace rtc
