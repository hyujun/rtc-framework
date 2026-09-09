// ── test_contact_wrench_site_frame.cpp ───────────────────────────────────────
// Pins ContactWrenchConfig::reference_frame.
//
// WHAT MAKES THIS TESTABLE AT ALL. "body" and "site" mode differ only by the
// site's rotation relative to its body, and a wrench rotated by that quat is
// still a perfectly plausible wrench — no magnitude, sign or NaN check can see
// the difference. The only oracle is the rotation itself, applied literally.
// So the fixture carries a quat that maps every axis to a different axis
// (see contact_site_frame.xml for why a symmetric one would not do), and the
// test asserts the exact componentwise relation between the two modes.
//
// WHY THE TWO MODES RUN THROUGH StepForTest RATHER THAN Start()/Stop().
// The relation being asserted is exact, so the two runs must see bit-identical
// physics. The threaded SimLoop steps as many times as the wall clock allows
// between the command writes, so two runs of it converge to *similar* forces,
// not equal ones — and the test would then need a tolerance wide enough to
// swallow the very rotation it is checking. StepForTest is the same tick
// (ApplyCommand → PreparePhysicsStep → mj_step → reads) with the thread and
// the clock taken out, so both runs land on the same number.
// ────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#ifndef CONTACT_SITE_FRAME_MJCF_PATH
#error "CONTACT_SITE_FRAME_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

// The fixture's bracket site quat, as a rotation of site axes expressed in the
// body frame. Spelled out here rather than read back from the model: an oracle
// that asks the model under test for the answer cannot catch the model being
// wrong. Keep in sync with contact_site_frame.xml.
//
//   R_body_site = [[0, 0, -1],
//                  [1, 0,  0],
//                  [0, -1, 0]]
//
// so v_site = R^T v_body = (v_body.y, -v_body.z, -v_body.x).
[[nodiscard]] std::array<double, 3> BodyVecToBracketSite(const std::array<double, 3>& v_body) {
  return {v_body[1], -v_body[2], -v_body[0]};
}

MuJoCoSimulator::Config MakeConfig(const std::string& reference_frame,
                                   const std::string& site_suffix) {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = CONTACT_SITE_FRAME_MJCF_PATH;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = 10.0;
  cfg.max_rtf = 0.0;
  cfg.n_substeps = 1;
  cfg.viewer_refresh_rate = 60.0;

  JointGroupConfig grp;
  grp.name = "slider";
  grp.command_joint_names = {"j1"};
  grp.state_joint_names = {"j1"};
  grp.command_topic = "/slider/cmd";
  grp.state_topic = "/slider/state";
  grp.sensor_names = {"auto"};
  grp.is_robot = true;
  grp.contact_wrench.enabled = true;
  grp.contact_wrench.topic_prefix = "/test/contact_wrench";
  grp.contact_wrench.sensor_name_suffixes = {"_contact"};
  grp.contact_wrench.reference_site_suffixes = {site_suffix};
  grp.contact_wrench.reference_frame = reference_frame;
  cfg.groups.push_back(grp);
  return cfg;
}

// Hold the arm straight down so the sphere overlaps the ground box (the same
// steady-contact pose contact_minimal.xml uses), then run a fixed number of
// deterministic ticks.
constexpr int kSettleSteps = 600;

[[nodiscard]] JointGroup::ContactWrenchSample RunToContact(MuJoCoSimulator& sim) {
  sim.StageCommand(0, JointControlMode::kPosition, {0.0}, {}, {}, {});
  for (int i = 0; i < kSettleSteps; ++i) {
    sim.StageCommand(0, JointControlMode::kPosition, {0.0}, {}, {}, {});
    sim.StepForTest();
  }
  const auto& samples = sim.GetContactWrenchSamplesForTest(0);
  EXPECT_EQ(samples.size(), 1u);
  return samples.empty() ? JointGroup::ContactWrenchSample{} : samples[0];
}

// ── 1. frame_id follows the mode, so a consumer can tell them apart. ────────
TEST(ContactWrenchSiteFrame, FrameIdNamesTheFrameTheNumbersAreIn) {
  MuJoCoSimulator body_sim(MakeConfig("body", "_ft_site"));
  ASSERT_TRUE(body_sim.Initialize());
  ASSERT_EQ(body_sim.GetContactWrenchInfos(0).size(), 1u);
  EXPECT_EQ(body_sim.GetContactWrenchInfos(0)[0].frame_id, "tip_link");
  EXPECT_FALSE(body_sim.GetContactWrenchInfos(0)[0].use_site_frame);

  MuJoCoSimulator site_sim(MakeConfig("site", "_bracket"));
  ASSERT_TRUE(site_sim.Initialize());
  ASSERT_EQ(site_sim.GetContactWrenchInfos(0).size(), 1u);
  EXPECT_EQ(site_sim.GetContactWrenchInfos(0)[0].frame_id, "tip_a_bracket");
  EXPECT_TRUE(site_sim.GetContactWrenchInfos(0)[0].use_site_frame);
}

// ── 2. The two modes differ by exactly the site rotation. ───────────────────
//
// This is the whole feature. Both assertions are load-bearing: the equality
// says the right rotation was applied, and the inequality says a rotation was
// applied at all — without it a silent fallback to the body frame would pass
// every remaining check in this file.
TEST(ContactWrenchSiteFrame, SiteModeRotatesTheWrenchByTheSiteQuat) {
  MuJoCoSimulator body_sim(MakeConfig("body", "_ft_site"));
  ASSERT_TRUE(body_sim.Initialize());
  const auto body_sample = RunToContact(body_sim);

  MuJoCoSimulator site_sim(MakeConfig("site", "_bracket"));
  ASSERT_TRUE(site_sim.Initialize());
  const auto site_sample = RunToContact(site_sim);

  ASSERT_TRUE(body_sample.found);
  ASSERT_TRUE(site_sample.found);

  // The contact must actually carry force, or every relation below holds
  // trivially on zeros and the test is vacuous.
  ASSERT_GT(std::abs(body_sample.force[2]), 1.0);

  const auto expected_force = BodyVecToBracketSite(body_sample.force);
  const auto expected_torque = BodyVecToBracketSite(body_sample.torque);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(site_sample.force[static_cast<std::size_t>(i)],
                expected_force[static_cast<std::size_t>(i)], 1e-9)
        << "force component " << i;
    EXPECT_NEAR(site_sample.torque[static_cast<std::size_t>(i)],
                expected_torque[static_cast<std::size_t>(i)], 1e-9)
        << "torque component " << i;
  }

  // Not the identity: the dominant body-frame component is +z, and in the site
  // frame it must have moved off z entirely.
  EXPECT_GT(std::abs(site_sample.force[1]), 1.0);
  EXPECT_NEAR(site_sample.force[2], 0.0, 1e-6);
}

// ── 3. Both modes see the same physics — the pose is not what changed. ──────
//
// Guards the reading above: if the two runs had diverged (different contact
// depth, different settle), the rotation relation could hold for the wrong
// reason. dist comes straight from the sensor and is frame-independent.
TEST(ContactWrenchSiteFrame, BothModesObserveTheSameContact) {
  MuJoCoSimulator body_sim(MakeConfig("body", "_ft_site"));
  ASSERT_TRUE(body_sim.Initialize());
  const auto body_sample = RunToContact(body_sim);

  MuJoCoSimulator site_sim(MakeConfig("site", "_bracket"));
  ASSERT_TRUE(site_sim.Initialize());
  const auto site_sample = RunToContact(site_sim);

  EXPECT_NEAR(body_sample.dist, site_sample.dist, 1e-12);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(body_sample.point_world[static_cast<std::size_t>(i)],
                site_sample.point_world[static_cast<std::size_t>(i)], 1e-12)
        << "contact point component " << i;
  }
}

// ── 4. An unknown spelling is rejected, not silently read as "body". ────────
TEST(ContactWrenchSiteFrame, UnknownReferenceFrameFailsInitialize) {
  MuJoCoSimulator sim(MakeConfig("bracket", "_bracket"));
  EXPECT_FALSE(sim.Initialize());
}

// ── 5. The default is "body", so existing scenes are untouched. ─────────────
TEST(ContactWrenchSiteFrame, DefaultReferenceFrameIsBody) {
  const JointGroupConfig::ContactWrenchConfig defaults;
  EXPECT_EQ(defaults.reference_frame, "body");
}

}  // namespace
}  // namespace rtc
