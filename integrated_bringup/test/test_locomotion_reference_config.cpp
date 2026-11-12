// locomotion_controller.command.* · gait.* · swing.* · terrain.* · reference.* (g1_loco): the
// installed files, composed by the CM's own loader (rtc::LoadControllerConfig — the `include:`
// merge included), load through rtc_mpc's reference/ loaders and initialise the stage they
// configure. The schema and every refusal are rtc_mpc's own tests; this one is the wiring between
// the shipped fragment and that loader. No ROS node.
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace {

YAML::Node InstalledLocomotionConfig() {
  const std::string main = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/g1_loco/controllers/locomotion_controller.yaml";
  return rtc::LoadControllerConfig(main, "locomotion_controller");
}

TEST(LocomotionReferenceConfig, TheInstalledCommandSectionLoadsAndInitsTheFilter) {
  const YAML::Node command = InstalledLocomotionConfig()["command"];
  ASSERT_TRUE(command.IsMap()) << "the include: merge did not bring command.* in";
  rtc::mpc::CommandFilterConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadCommandFilterConfig(command, cfg),
            rtc::mpc::CommandFilterError::kNoError);
  rtc::mpc::CommandFilter filter;
  EXPECT_EQ(filter.Init(cfg), rtc::mpc::CommandFilterError::kNoError);
  EXPECT_EQ(filter.output().base_height, cfg.base_height);
}

// The one test that wires contact names to bits: the gait's names are the
// model's contacts, in model order — so walk's single supports are bit 0 then 1.
TEST(LocomotionReferenceConfig, TheInstalledGaitSectionBitsFollowTheModelContactOrder) {
  const YAML::Node root = InstalledLocomotionConfig();
  const YAML::Node contacts = root["model"]["contacts"];
  ASSERT_TRUE(contacts.IsSequence());
  std::vector<std::string> names;
  for (const auto& c : contacts) {
    names.push_back(c["name"].as<std::string>());
  }
  rtc::mpc::GaitTable table;
  std::vector<std::string> gaits;
  ASSERT_EQ(rtc::mpc::LoadGaitTable(root["gait"], names, table, &gaits),
            rtc::mpc::GaitTableError::kNoError);
  ASSERT_EQ(gaits, (std::vector<std::string>{"stance", "slow_walk", "walk"}));
  const rtc::mpc::GaitTemplate& walk = table.gaits[2];
  ASSERT_EQ(walk.n_phases, 4U);
  EXPECT_EQ(walk.mode[0], 0b01U);  // contacts[0] alone
  EXPECT_EQ(walk.mode[1], 0b11U);
  EXPECT_EQ(walk.mode[2], 0b10U);  // contacts[1] alone
  EXPECT_EQ(walk.mode[3], 0b11U);
  EXPECT_EQ(table.gaits[0].mode[0], 0b11U);
  rtc::mpc::GaitScheduler scheduler;
  EXPECT_EQ(scheduler.Init(table, 1.0), rtc::mpc::GaitSchedulerError::kNoError);
}

TEST(LocomotionReferenceConfig, TheInstalledSwingAndTerrainSectionsLoadAndBuildASwing) {
  const YAML::Node root = InstalledLocomotionConfig();
  ASSERT_TRUE(root["swing"].IsMap()) << "the include: merge did not bring swing.* in";
  ASSERT_TRUE(root["terrain"].IsMap()) << "the include: merge did not bring terrain.* in";
  rtc::mpc::SwingConfig swing;
  ASSERT_EQ(rtc::mpc::LoadSwingConfig(root["swing"], swing), rtc::mpc::SwingError::kNoError);
  rtc::mpc::SurfaceFrame terrain;
  ASSERT_EQ(rtc::mpc::LoadTerrainConfig(root["terrain"], terrain),
            rtc::mpc::SurfaceFrameError::kNoError);
  // The shipped values plan a walk-length swing above the shipped ground.
  rtc::mpc::SwingInterval stretch;
  stretch.lift_off = 0.0;
  stretch.touch_down = 0.6;
  rtc::mpc::SwingTrajectory s = rtc::mpc::SwingTrajectory::Stance(0.0);
  ASSERT_EQ(rtc::mpc::BuildSwing(stretch, swing, rtc::mpc::HeightAt(terrain, 0.0, 0.0), s),
            rtc::mpc::SwingError::kNoError);
  EXPECT_NEAR(s.Evaluate(0.3).z,
              rtc::mpc::HeightAt(terrain, 0.0, 0.0) + swing.touch_down_offset + swing.height,
              1e-12);
}

// reference.* names exactly the actuated joints of the shipped rosters (the body
// group's 23 in model order, then the hand group's 10) — the joints the CM's
// actuated model moves; the arm-swing joints are checked even though it is off.
TEST(LocomotionReferenceConfig, TheInstalledReferenceSectionCoversTheShippedRosters) {
  const YAML::Node root = InstalledLocomotionConfig();
  ASSERT_TRUE(root["reference"].IsMap()) << "the include: merge did not bring reference.* in";
  const std::string base = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/g1_loco/_base.yaml";
  const YAML::Node devices = YAML::LoadFile(base)["/**"]["ros__parameters"]["devices"];
  std::vector<std::string> joints;
  for (const char* group : {"g1", "p1b"}) {
    ASSERT_TRUE(devices[group]["joint_state_names"].IsSequence()) << group;
    for (const auto& j : devices[group]["joint_state_names"]) {
      joints.push_back(j.as<std::string>());
    }
  }
  ASSERT_EQ(joints.size(), 33U);
  rtc::mpc::ReferenceConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadReferenceConfig(root["reference"], joints, cfg),
            rtc::mpc::ReferenceError::kNoError);
  EXPECT_EQ(cfg.q_default.size(), joints.size());
  EXPECT_FALSE(cfg.arm_swing_enabled);
  EXPECT_EQ(cfg.arm_swing_joints.size(), 4U);
  // One roster joint missing is refused — the loader does check coverage.
  joints.push_back("left_wrist_roll_joint");
  EXPECT_EQ(rtc::mpc::LoadReferenceConfig(root["reference"], joints, cfg),
            rtc::mpc::ReferenceError::kMissingJoint);
}

}  // namespace
