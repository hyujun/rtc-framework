// The installed g1_loco and b7_loco profiles, as the sim launch hands them to the
// RT node: the body group's mujoco_native backend configured from the installed
// `_base.yaml` + `sim.yaml` claims both the floating-base and the IMU lane, and
// the IMU site order its `imu_state.site_names` gives the lane is the order the
// locomotion controller's `model.imu.sites` names — the controller knows the
// sites only by these labels, so the two lists are one contract in two files.
//
// Configure only: no message is exchanged, so no simulator is needed.
#include "integrated_bringup/backends/mujoco_native_backend.hpp"
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_controller_manager/device_backend.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace loco = integrated_bringup::locomotion;

std::string ProfileDir(const std::string& profile) {
  return ament_index_cpp::get_package_share_directory("integrated_bringup") + "/config/" + profile;
}

// The installed controller's `model` section, through the CM's own loader (the
// `include:` merge that brings `locomotion/model.yaml` in).
loco::HumanoidModelConfig InstalledModel(const std::string& profile) {
  const YAML::Node model =
      rtc::LoadControllerConfig(ProfileDir(profile) + "/controllers/locomotion_controller.yaml",
                                "locomotion_controller")["model"];
  return loco::LoadHumanoidModelConfig(model, profile + " locomotion_controller.model");
}

// `devices.<group>.backend` of the installed sim.yaml.
YAML::Node InstalledBackend(const std::string& profile, const std::string& group) {
  return YAML::LoadFile(ProfileDir(profile) +
                        "/sim.yaml")["/**"]["ros__parameters"]["devices"][group]["backend"];
}

class InstalledLocoImuLane : public ::testing::TestWithParam<const char*> {};

TEST_P(InstalledLocoImuLane, BodyBackendClaimsBothLanesInTheModelsSiteOrder) {
  const std::string profile = GetParam();
  const loco::HumanoidModelConfig model = InstalledModel(profile);
  const std::string group = model.body_group;
  const YAML::Node backend_yaml = InstalledBackend(profile, group);
  ASSERT_TRUE(backend_yaml.IsMap()) << profile << ": no devices." << group << ".backend";
  ASSERT_EQ(backend_yaml["type"].as<std::string>(), "mujoco_native");

  // The site order: imu_state.site_names IS model.imu.sites, element by element.
  const YAML::Node imu_yaml = backend_yaml["imu_state"];
  ASSERT_TRUE(imu_yaml.IsMap()) << profile << ": no imu_state block";
  EXPECT_EQ(imu_yaml["site_names"].as<std::vector<std::string>>(), model.imu.sites) << profile;
  EXPECT_NE(std::find(model.imu.sites.begin(), model.imu.sites.end(), model.imu.primary_site),
            model.imu.sites.end())
      << profile << ": primary_site '" << model.imu.primary_site << "' is not a site";
  // The bundled lane's keys are gone from the base block.
  EXPECT_FALSE(backend_yaml["base_state"]["imu_gyro"].IsDefined()) << profile;
  EXPECT_FALSE(backend_yaml["base_state"]["imu_accel"].IsDefined()) << profile;

  // Both lanes claimed from the installed files, layered as the launch layers them.
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", ProfileDir(profile) + "/_base.yaml",
                     "--params-file", ProfileDir(profile) + "/sim.yaml"});
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loco_imu_lane_" + profile, options);
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rtc::DeviceBackendConfig cfg;
  cfg.group_name = group;
  cfg.type = "mujoco_native";
  cfg.state_topic = backend_yaml["state_topic"].as<std::string>();
  cfg.command_topic = backend_yaml["command_topic"].as<std::string>();
  rtc::MujocoNativeBackend backend;
  backend.Configure(node.get(), cfg, cb_group);
  EXPECT_TRUE(backend.HasFloatingBase()) << profile;
  EXPECT_TRUE(backend.HasImuState()) << profile;
  // The backend read the same list the YAML comparison above read.
  EXPECT_EQ(
      node->get_parameter("devices." + group + ".backend.imu_state.site_names").as_string_array(),
      model.imu.sites)
      << profile;
}

INSTANTIATE_TEST_SUITE_P(Profiles, InstalledLocoImuLane, ::testing::Values("g1_loco", "b7_loco"),
                         [](const ::testing::TestParamInfo<const char*>& info) {
                           return std::string(info.param);
                         });

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
