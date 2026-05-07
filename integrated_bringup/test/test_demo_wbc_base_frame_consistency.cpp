// ── DemoWbcController base_frame consistency tests (F-2) ────────────────────
//
// Exercises the lifecycle gate that compares every captured SE3/MPC
// `base_frame` YAML value against the primary device's `urdf.root_link`.
// The check fires inside OnDeviceConfigsSet and surfaces via
// `IsBaseFrameMismatchForTesting()`; on_configure consumes the same flag
// to fail the lifecycle transition.

#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include <rtc_base/types/types.hpp>

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

using integrated_bringup::DemoWbcController;
using rtc::DeviceNameConfig;
using rtc::DeviceUrdfConfig;

DeviceNameConfig MakeUr5eConfig(std::string root_link) {
  DeviceNameConfig cfg;
  cfg.device_name = "ur5e";
  DeviceUrdfConfig urdf;
  urdf.package = "robot_descriptions";
  urdf.path = "robots/ur5e/urdf/ur5e.urdf";
  urdf.root_link = std::move(root_link);
  urdf.tip_link = "tool0";
  cfg.urdf = std::move(urdf);
  return cfg;
}

TEST(DemoWbcBaseFrameConsistency, MatchesRootLink) {
  DemoWbcController ctrl{""};
  ctrl.SetBaseFrameEntriesForTesting({
      {"tsid.tasks.se3_tcp.base_frame", "base"},
      {"mpc.light.model.base_frame", "base"},
      {"mpc.rich.model.base_frame", "base"},
  });

  std::map<std::string, DeviceNameConfig> configs;
  configs["ur5e"] = MakeUr5eConfig("base");
  ctrl.SetDeviceNameConfigs(std::move(configs));

  EXPECT_FALSE(ctrl.IsBaseFrameMismatchForTesting());
  EXPECT_TRUE(ctrl.GetBaseFrameMismatchDetailForTesting().empty());
}

TEST(DemoWbcBaseFrameConsistency, MismatchFlagsLifecycleFailure) {
  DemoWbcController ctrl{""};
  ctrl.SetBaseFrameEntriesForTesting({
      {"tsid.tasks.se3_tcp.base_frame", "base_link"},  // stale value
      {"mpc.light.model.base_frame", "base"},
  });

  std::map<std::string, DeviceNameConfig> configs;
  configs["ur5e"] = MakeUr5eConfig("base");
  ctrl.SetDeviceNameConfigs(std::move(configs));

  EXPECT_TRUE(ctrl.IsBaseFrameMismatchForTesting());
  // Detail message must name the offending entry and the resolved root_link.
  const auto& detail = ctrl.GetBaseFrameMismatchDetailForTesting();
  EXPECT_NE(detail.find("se3_tcp"), std::string::npos) << "detail=" << detail;
  EXPECT_NE(detail.find("base_link"), std::string::npos) << "detail=" << detail;
  EXPECT_NE(detail.find("'base'"), std::string::npos) << "detail=" << detail;
}

TEST(DemoWbcBaseFrameConsistency, EmptyEntriesSkipsCheck) {
  // No base_frame entries captured (e.g. tsid_disabled + mpc_disabled). The
  // F-2 mismatch gate only fires when entries exist; absent strict-mode
  // enforcement on the consumer side is the responsibility of TSID/MPC
  // (F-4: SE3Task throws and RobotModelHandler returns kInvalidYamlSchema
  // when their YAML lacks `base_frame`).
  DemoWbcController ctrl{""};
  ctrl.SetBaseFrameEntriesForTesting({});

  std::map<std::string, DeviceNameConfig> configs;
  configs["ur5e"] = MakeUr5eConfig("base");
  ctrl.SetDeviceNameConfigs(std::move(configs));

  EXPECT_FALSE(ctrl.IsBaseFrameMismatchForTesting());
}

TEST(DemoWbcBaseFrameConsistency, MissingPrimaryDeviceUrdfSkipsCheck) {
  // When the primary device config has no urdf block (or root_link is
  // empty), there is no ground truth to compare against — leave validation
  // to whoever configured the device name configs upstream.
  DemoWbcController ctrl{""};
  ctrl.SetBaseFrameEntriesForTesting({
      {"tsid.tasks.se3_tcp.base_frame", "world"},
  });

  std::map<std::string, DeviceNameConfig> configs;
  DeviceNameConfig cfg;
  cfg.device_name = "ur5e";  // no urdf field set
  configs["ur5e"] = std::move(cfg);
  ctrl.SetDeviceNameConfigs(std::move(configs));

  EXPECT_FALSE(ctrl.IsBaseFrameMismatchForTesting());
}

}  // namespace
