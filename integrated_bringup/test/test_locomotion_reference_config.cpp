// locomotion_controller.command.* (g1_loco): the installed files, composed by the
// CM's own loader (rtc::LoadControllerConfig — the `include:` merge included),
// load through rtc_mpc's reference/ loaders and initialise the stage they
// configure. The schema and every refusal are rtc_mpc's own tests; this one is
// the wiring between the shipped fragment and that loader. No ROS node.
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/reference/command_filter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>

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

}  // namespace
