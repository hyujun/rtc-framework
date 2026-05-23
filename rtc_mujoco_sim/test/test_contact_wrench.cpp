// ── test_contact_wrench.cpp ──────────────────────────────────────────────────
// Verifies MJCF <sensor><contact> → world→link transform → callback pipeline.
// Uses a single-DoF slider whose sphere tip can be commanded to penetrate a
// static box, producing a netforce contact whose direction and magnitude are
// known a priori.
// ────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#ifndef CONTACT_MJCF_PATH
#error "CONTACT_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

using namespace std::chrono_literals;

MuJoCoSimulator::Config MakeContactConfig() {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = CONTACT_MJCF_PATH;
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
  grp.sensor_topic = "/slider/sensors";
  grp.sensor_names = {"auto"};  // includes mjSENS_CONTACT
  grp.is_robot = true;
  grp.contact_wrench.enabled = true;
  grp.contact_wrench.topic_prefix = "/test/contact_wrench";
  // suffix defaults already match "_contact" / "_ft_site" — explicit for clarity:
  grp.contact_wrench.sensor_name_suffixes = {"_contact"};
  grp.contact_wrench.reference_site_suffixes = {"_ft_site"};
  cfg.groups.push_back(grp);
  return cfg;
}

// ── 1. Auto-discovery: target/site/body resolved from suffix rules. ─────────
TEST(ContactWrench, AutoDiscoveryFindsTarget) {
  MuJoCoSimulator sim(MakeContactConfig());
  ASSERT_TRUE(sim.Initialize());
  EXPECT_TRUE(sim.HasContactWrenches(0));
  const auto& infos = sim.GetContactWrenchInfos(0);
  ASSERT_EQ(infos.size(), 1u);
  // target_name mirrors the MJCF <contact name="..."> verbatim so the ROS
  // topic round-trips losslessly. The suffix list (here "_contact") is only
  // used internally to derive site_stem = "tip_a" for ft_site lookup.
  EXPECT_EQ(infos[0].target_name, "tip_a_contact");
  EXPECT_EQ(infos[0].frame_id, "tip_link");
  EXPECT_GE(infos[0].sensor_id, 0);
  EXPECT_GE(infos[0].ft_site_id, 0);
  EXPECT_GE(infos[0].body_id, 0);
}

// ── 2. No-contact: tip parked above the ground → found=false, wrench≈0. ─────
TEST(ContactWrench, NoContactPublishesZeroSample) {
  MuJoCoSimulator sim(MakeContactConfig());
  ASSERT_TRUE(sim.Initialize());

  std::atomic<int> cb_count{0};
  std::mutex last_mutex;
  JointGroup::ContactWrenchSample last{};
  sim.SetContactWrenchCallback(
      0, [&](const std::vector<JointGroup::ContactWrenchInfo>&,
             const std::vector<JointGroup::ContactWrenchSample>& samples) {
        cb_count.fetch_add(1, std::memory_order_relaxed);
        if (!samples.empty()) {
          std::lock_guard lock(last_mutex);
          last = samples[0];
        }
      });

  sim.Start();
  // Lift arm horizontal (θ ≈ +π/2) → tip clears ground.
  for (int i = 0; i < 400; ++i) {
    sim.SetCommand(0, {1.57});
    std::this_thread::sleep_for(1ms);
  }
  sim.Stop();

  EXPECT_GT(cb_count.load(), 0);
  std::lock_guard lock(last_mutex);
  EXPECT_FALSE(last.found);
  EXPECT_NEAR(last.force[0], 0.0, 1e-9);
  EXPECT_NEAR(last.force[1], 0.0, 1e-9);
  EXPECT_NEAR(last.force[2], 0.0, 1e-9);
}

// ── 3. Hang straight down → tip overlaps ground → contact detected. ────────
//
// At j1=0 the arm hangs vertical with the sphere tip overlapping the ground
// box (sphere bottom at z=0.02, ground top at z=0.05 → 0.03 m penetration).
// At rest the tip_link frame is unrotated (no rotation about hinge from
// θ=0), so world-frame +z reaction maps directly to link-frame +z force.
TEST(ContactWrench, HangingArmContactProducesPositiveZForce) {
  MuJoCoSimulator sim(MakeContactConfig());
  ASSERT_TRUE(sim.Initialize());

  std::atomic<int> cb_count{0};
  std::mutex last_mutex;
  JointGroup::ContactWrenchSample last{};
  sim.SetContactWrenchCallback(
      0, [&](const std::vector<JointGroup::ContactWrenchInfo>&,
             const std::vector<JointGroup::ContactWrenchSample>& samples) {
        cb_count.fetch_add(1, std::memory_order_relaxed);
        if (!samples.empty()) {
          std::lock_guard lock(last_mutex);
          last = samples[0];
        }
      });

  sim.Start();
  // Hold arm straight down (θ=0) — contact occurs because sphere overlaps
  // the ground geom by design.
  for (int i = 0; i < 600; ++i) {
    sim.SetCommand(0, {0.0});
    std::this_thread::sleep_for(1ms);
  }
  sim.Stop();

  EXPECT_GT(cb_count.load(), 0);
  std::lock_guard lock(last_mutex);
  EXPECT_TRUE(last.found) << "expected contact while arm hangs into ground";
  // tip_link inherits arm_base rotation; at θ=0 the link frame matches world,
  // so world +z reaction maps to link +z.
  EXPECT_GT(last.force[2], 0.1) << "force z should be positive and non-trivial";
}

}  // namespace
}  // namespace rtc
