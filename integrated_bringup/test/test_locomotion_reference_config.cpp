// locomotion_controller.command.* · gait.* (g1_loco): the installed files, composed by the
// CM's own loader (rtc::LoadControllerConfig — the `include:` merge included),
// load through rtc_mpc's reference/ loaders and initialise the stage they
// configure. The schema and every refusal are rtc_mpc's own tests; this one is
// the wiring between the shipped fragment and that loader. No ROS node.
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"

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

}  // namespace
