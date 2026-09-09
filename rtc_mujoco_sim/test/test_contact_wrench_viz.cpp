// ── test_contact_wrench_viz.cpp ──────────────────────────────────────────────
// The viewer's fingertip force arrows must be the SAME QUANTITY the
// contact_wrench topic carries.
//
// WHY THAT NEEDS A TEST AT ALL. The arrow and the message are produced by two
// different code paths reading the same sensordata: ReadContactWrenches rotates
// into the reference frame and hands the result to the ROS publisher, while
// SnapshotContactWrenchViz keeps the world-frame vector for the renderer. Each
// is separately plausible, and the failure mode of a disagreement is a picture
// that quietly contradicts the topic — an arrow pointing the wrong way while
// `ros2 topic echo` shows the right numbers, or vice versa. Nothing about the
// running system makes that visible; a human comparing them by eye is the only
// other instrument.
//
// So the oracle here is the relation, not a literal: rotating the PUBLISHED
// link-frame force back into world by the reference frame's own rotation must
// reproduce the snapshot's force, componentwise. That catches a dropped or
// doubled negation between the two paths (the sign convention lives in only one
// of them), a transposed rotation, and a component permutation — while staying
// silent about magnitudes, which the contact solver owns and this file does not.
//
// Rendering itself is not under test: mjv_connector and the GLFW loop need a
// display. What is under test is everything that decides WHAT gets drawn.
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#ifndef CONTACT_MJCF_PATH
#error "CONTACT_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

// Same fixture as test_contact_wrench.cpp: a single-DoF slider whose sphere tip
// overlaps a static ground box at j1 = 0, so contact is reached by holding the
// commanded position rather than by staging a grasp.
MuJoCoSimulator::Config MakeVizConfig(bool visualize, double scale) {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = CONTACT_MJCF_PATH;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = 10.0;
  cfg.n_substeps = 1;

  JointGroupConfig grp;
  grp.name = "slider";
  grp.command_joint_names = {"j1"};
  grp.state_joint_names = {"j1"};
  grp.command_topic = "/slider/cmd";
  grp.state_topic = "/slider/state";
  grp.sensor_topic = "/slider/sensors";
  grp.sensor_names = {"auto"};
  grp.is_robot = true;
  grp.contact_wrench.enabled = true;
  grp.contact_wrench.topic_prefix = "/test/contact_wrench";
  grp.contact_wrench.sensor_name_suffixes = {"_contact"};
  grp.contact_wrench.reference_site_suffixes = {"_ft_site"};
  grp.contact_wrench.visualize = visualize;
  grp.contact_wrench.visualize_scale = scale;
  cfg.groups.push_back(grp);
  return cfg;
}

constexpr double kVizScale = 0.004;

// Hold the arm at j1 = 0 (tip into the ground) for long enough that the contact
// is established and the solver has settled. StepForTest runs the same
// ApplyCommand → mj_step → ReadContactWrenches sequence the SimLoop does, so
// this reaches contact through the production path and stays deterministic.
void SettleIntoContact(MuJoCoSimulator& sim, int steps = 600) {
  for (int i = 0; i < steps; ++i) {
    sim.SetCommand(0, {0.0});
    sim.StepForTest();
  }
}

// R_ref (row-major 3x3) for a discovered sensor, from the same arrays
// ReadContactWrenches uses. `reference_frame` is left at its "body" default in
// this fixture, so this reads mjData::xmat.
const double* ReferenceRotation(const MuJoCoSimulator& sim,
                                const JointGroup::ContactWrenchInfo& info) {
  const mjData* d = sim.GetData();
  return info.use_site_frame ? d->site_xmat + 9 * info.ft_site_id : d->xmat + 9 * info.body_id;
}

// ── 1. One entry per discovered sensor, in discovery order. ────────────────
TEST(ContactWrenchViz, SnapshotHasOneEntryPerSensor) {
  MuJoCoSimulator sim(MakeVizConfig(true, kVizScale));
  ASSERT_TRUE(sim.Initialize());
  ASSERT_TRUE(sim.HasContactWrenches(0));

  const auto viz = sim.RefreshContactWrenchVizForTest();
  EXPECT_EQ(viz.size(), sim.GetContactWrenchInfos(0).size());
}

// ── 2. The arrow and the topic are the same vector. ────────────────────────
//
// The assertion this file exists for. See the header comment for why the oracle
// is R_ref * f_published rather than a literal.
TEST(ContactWrenchViz, SnapshotForceMatchesThePublishedWrenchRotatedToWorld) {
  MuJoCoSimulator sim(MakeVizConfig(true, kVizScale));
  ASSERT_TRUE(sim.Initialize());
  SettleIntoContact(sim);

  const auto& infos = sim.GetContactWrenchInfos(0);
  const auto& samples = sim.GetContactWrenchSamplesForTest(0);
  const auto viz = sim.RefreshContactWrenchVizForTest();
  ASSERT_EQ(viz.size(), infos.size());
  ASSERT_EQ(samples.size(), infos.size());

  bool any_contact = false;
  for (std::size_t i = 0; i < infos.size(); ++i) {
    if (!samples[i].found) {
      continue;
    }
    any_contact = true;
    EXPECT_TRUE(viz[i].active) << "sensor " << i << ": published a contact but drew nothing";

    // v_world = R_ref * v_link. Row-major, so row r is elements [3r .. 3r+2].
    const double* r = ReferenceRotation(sim, infos[i]);
    const auto& f = samples[i].force;
    const double expected[3] = {r[0] * f[0] + r[1] * f[1] + r[2] * f[2],
                                r[3] * f[0] + r[4] * f[1] + r[5] * f[2],
                                r[6] * f[0] + r[7] * f[1] + r[8] * f[2]};

    // Both sides come from the same mjData with no intervening step, so the
    // only difference is floating-point rounding through the two rotations.
    for (int k = 0; k < 3; ++k) {
      EXPECT_NEAR(viz[i].force[static_cast<std::size_t>(k)], expected[k], 1e-9)
          << "sensor " << i << " axis " << k;
    }

    // Non-trivial, or the comparison above would hold for an all-zero pair.
    const double mag = std::sqrt(expected[0] * expected[0] + expected[1] * expected[1] +
                                 expected[2] * expected[2]);
    EXPECT_GT(mag, 0.1) << "sensor " << i << ": contact force too small to pin a direction";
  }
  ASSERT_TRUE(any_contact) << "the fixture never reached contact — nothing was compared";
}

// ── 3. The arrow starts at the reference site, not the body origin. ────────
//
// Those coincide in many scenes, which is exactly why a wrong one survives
// inspection; the fixture's ft_site is offset from its body so they do not.
TEST(ContactWrenchViz, SnapshotOriginIsTheReferenceSite) {
  MuJoCoSimulator sim(MakeVizConfig(true, kVizScale));
  ASSERT_TRUE(sim.Initialize());
  SettleIntoContact(sim);

  const auto& infos = sim.GetContactWrenchInfos(0);
  const auto viz = sim.RefreshContactWrenchVizForTest();
  ASSERT_EQ(viz.size(), infos.size());

  const mjData* d = sim.GetData();
  for (std::size_t i = 0; i < infos.size(); ++i) {
    if (!viz[i].active) {
      continue;
    }
    const double* p = d->site_xpos + 3 * infos[i].ft_site_id;
    for (int k = 0; k < 3; ++k) {
      EXPECT_DOUBLE_EQ(viz[i].origin[static_cast<std::size_t>(k)], p[k])
          << "sensor " << i << " axis " << k;
    }
  }
}

// ── 4. No contact → nothing drawn, and the entry stays in place. ───────────
//
// The array is parallel to the infos rather than compacted, so index i means
// the same fingertip on every frame; `active` is what the renderer skips on.
TEST(ContactWrenchViz, NoContactClearsTheEntryWithoutRemovingIt) {
  MuJoCoSimulator sim(MakeVizConfig(true, kVizScale));
  ASSERT_TRUE(sim.Initialize());

  // No steps: nothing has touched anything yet.
  const auto viz = sim.RefreshContactWrenchVizForTest();
  ASSERT_EQ(viz.size(), sim.GetContactWrenchInfos(0).size());
  for (const auto& s : viz) {
    if (s.active) {
      continue;
    }
    EXPECT_DOUBLE_EQ(s.force[0], 0.0);
    EXPECT_DOUBLE_EQ(s.force[1], 0.0);
    EXPECT_DOUBLE_EQ(s.force[2], 0.0);
  }
}

// ── 5. The configured scale reaches the renderer. ──────────────────────────
//
// The renderer multiplies by this and nothing else validates it, so a scale
// that never left the config would draw every arrow at zero length — visually
// identical to "no contact".
TEST(ContactWrenchViz, ScaleIsCarriedFromTheGroupConfig) {
  MuJoCoSimulator sim(MakeVizConfig(true, kVizScale));
  ASSERT_TRUE(sim.Initialize());

  const auto viz = sim.RefreshContactWrenchVizForTest();
  ASSERT_FALSE(viz.empty());
  for (const auto& s : viz) {
    EXPECT_FLOAT_EQ(s.scale, static_cast<float>(kVizScale));
  }
}

// ── 6. visualize:false really opts out. ────────────────────────────────────
//
// Negative control for every test above: with the flag off the snapshot is
// empty, so the sim thread does no per-tick work for a feature nobody asked
// for — and the tests above would pass vacuously if the flag were ignored.
TEST(ContactWrenchViz, DisabledVisualizeProducesNoEntries) {
  MuJoCoSimulator sim(MakeVizConfig(false, kVizScale));
  ASSERT_TRUE(sim.Initialize());
  ASSERT_TRUE(sim.HasContactWrenches(0)) << "the lane itself must still be running";
  SettleIntoContact(sim);

  EXPECT_TRUE(sim.RefreshContactWrenchVizForTest().empty());
}

}  // namespace
}  // namespace rtc
