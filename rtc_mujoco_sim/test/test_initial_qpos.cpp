// ── test_initial_qpos.cpp ────────────────────────────────────────────────────
// `robot_response.<group>.initial_qpos` — the startup and reset pose.
//
// WHY THIS EXISTS. Until this key, the only source of a group's initial pose
// was the MJCF's first keyframe. A scene without one therefore started at
// qpos0, which for an arm is the fully extended pose rather than any home
// somebody chose — and a bringup that must not edit the MJCF it loads had no
// way to fix that. The key is also read by HandleReset (the viewer's R), so
// getting it wrong moves both startup and every reset after it.
//
// WHAT MAKES THESE ASSERTIONS BITE. The precedence is carried by an explicit
// `has_yaml_initial_qpos` flag rather than by testing whether the vector is
// non-zero, and the difference is only observable in one case: an all-zero
// YAML pose on a scene whose keyframe is non-zero. `YamlAllZerosStillBeats-
// Keyframe` is that case. Drop the flag for a "is it set?" emptiness test on
// the *values* and every other test here still passes, which is precisely how
// the hand groups in this repo (all-zero home, ten joints) would have silently
// inherited a keyframe instead.
//
// The fixture keyframe is 0.30 / -0.70 — different magnitude and sign — so a
// transposed or reversed copy cannot land on a plausible pose.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#ifndef KEYFRAME_MJCF_PATH
#error "KEYFRAME_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

constexpr double kTol = 1e-9;

// The fixture's keyframe, mirrored here so a change to the MJCF fails loudly
// instead of quietly rewriting what "keyframe wins" means.
constexpr double kKeyJ1 = 0.30;
constexpr double kKeyJ2 = -0.70;

MuJoCoSimulator::Config MakeKeyframeConfig() {
  auto cfg = test::MakeMinimalConfig();
  cfg.model_path = KEYFRAME_MJCF_PATH;
  return cfg;
}

void ExpectPose(const MuJoCoSimulator& sim, double j1, double j2, const char* what) {
  const auto q = sim.GetPositions(0);
  ASSERT_EQ(q.size(), 2u) << what;
  EXPECT_NEAR(q[0], j1, kTol) << what;
  EXPECT_NEAR(q[1], j2, kTol) << what;
}

// ── Fallback lane: no YAML value ────────────────────────────────────────────

TEST(InitialQpos, KeyframeIsUsedWhenYamlIsAbsent) {
  MuJoCoSimulator sim(MakeKeyframeConfig());
  ASSERT_TRUE(sim.Initialize());
  ExpectPose(sim, kKeyJ1, kKeyJ2, "keyframe should supply the pose");
}

TEST(InitialQpos, ZerosWhenNeitherKeyframeNorYamlExists) {
  // MINIMAL_MJCF_PATH has no <keyframe> at all — the pre-existing behaviour
  // this key was added to give a way out of.
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  ExpectPose(sim, 0.0, 0.0, "no keyframe and no YAML should mean zeros");
}

// ── Override lane: YAML present ─────────────────────────────────────────────

TEST(InitialQpos, YamlOverridesKeyframe) {
  auto cfg = MakeKeyframeConfig();
  cfg.groups[0].initial_qpos = {0.90, 0.40};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  ExpectPose(sim, 0.90, 0.40, "YAML should outrank the keyframe");
}

TEST(InitialQpos, YamlAppliesWhenThereIsNoKeyframe) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].initial_qpos = {-0.25, 0.60};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  ExpectPose(sim, -0.25, 0.60, "YAML should apply with no keyframe present");
}

// The one case that separates a precedence FLAG from a "did the caller set
// non-zero values?" test. Both implementations pass every assertion above.
TEST(InitialQpos, YamlAllZerosStillBeatsKeyframe) {
  auto cfg = MakeKeyframeConfig();
  cfg.groups[0].initial_qpos = {0.0, 0.0};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  ExpectPose(sim, 0.0, 0.0, "an explicit all-zero YAML pose must not fall back");
}

// ── Reset lane ──────────────────────────────────────────────────────────────

// HandleReset (the viewer's R key) restores JointGroup::initial_qpos, so the
// YAML value has to reach it too. Without the positive control below this
// would pass on a reset that did nothing at all.
TEST(InitialQpos, ResetRestoresTheYamlPose) {
  auto cfg = MakeKeyframeConfig();
  cfg.groups[0].initial_qpos = {0.90, 0.40};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  ExpectPose(sim, 0.90, 0.40, "startup");

  sim.SetCommand(0, {-0.80, 1.10});
  for (int i = 0; i < 400; ++i) {
    sim.StepForTest();
  }
  const auto moved = sim.GetPositions(0);
  ASSERT_GT(std::abs(moved[0] - 0.90), 0.05)
      << "positive control: the arm never left the home pose";

  sim.ResetForTest();
  ExpectPose(sim, 0.90, 0.40, "reset must return to the YAML pose, not the keyframe or zeros");
}

// ── Rejection lane ──────────────────────────────────────────────────────────

TEST(InitialQpos, WrongSizeIsRejected) {
  auto cfg = MakeKeyframeConfig();
  cfg.groups[0].initial_qpos = {0.90};  // group has two command joints
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize())
      << "a short initial_qpos must fail Initialize, not apply partially";
}

TEST(InitialQpos, FakeGroupIsRejected) {
  // A fake group echoes commands through an LPF and owns no qpos, so accepting
  // the key there would be a config that looks honoured and is not.
  auto cfg = test::MakeMinimalConfig();
  JointGroupConfig fake;
  fake.name = "echo";
  fake.command_joint_names = {"j1", "j2"};
  fake.state_joint_names = {"j1", "j2"};
  fake.command_topic = "/echo/cmd";
  fake.state_topic = "/echo/state";
  fake.is_robot = false;
  fake.initial_qpos = {0.1, 0.2};
  cfg.groups.push_back(std::move(fake));

  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize()) << "initial_qpos on a fake_response group must fail Initialize";
}

}  // namespace
}  // namespace rtc
