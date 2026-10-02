// ── A controller with an arm model and no arm tip refuses the configure ──────
//
// The arm tip frame is looked up once, in OnDeviceConfigsSet, from the tip link
// the controller manager resolved for the primary device group. When the lookup
// finds nothing the frame id stays 0 — pinocchio's universe frame. joint, task
// and compliance then publish a pose that is not the arm tip's as the arm tip,
// flagged valid, on both tick lanes; wbc seeds its Cartesian hold through that
// frame (support/arm_tip_resolution.hpp has the lane-by-lane account). Nothing
// on the tick can tell, so the configure is where it is stopped.
//
// What this file pins, for the four controllers that report an arm tip pose
// (joint, task, compliance, wbc), on a real arm + hand:
//
//   - a tip link the arm model does not carry            → FAILURE
//   - a device config that carries no tip link at all    → FAILURE
//   - the same rig with the tip link it should have      → SUCCESS
//   - a controller with no arm model (no URDF)           → SUCCESS, and no crash
//     when its device config names a tip link
//
// A bare FAILURE proves little: on_configure can fail for something else. Each
// refusal is therefore pinned three ways — the same rig with the right tip
// configures, the controller's own verdict names the expected cause, and the
// other configure-time errors a test can read are empty.
//
// Both bring-up orders run (iiwa7_leap_controller_yamls.hpp). The verdict is
// read off the controller's state at on_configure; a verdict latched earlier,
// or a frame id cleared when the config is loaded again, would hold on one
// order and not on the other.
//
// The wbc controller runs on a YAML with no `tsid:` block: with TSID up it
// already refuses a configure that could not enable its CLIK backbone, and that
// refusal would answer these cases whether or not this one existed.

#include "iiwa7_leap_controller_yamls.hpp"
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/support/arm_tip_resolution.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
using fx::BringUp;
using integrated_bringup::ArmTipUnresolvedReason;
using integrated_bringup::DemoComplianceController;
using integrated_bringup::DemoJointController;
using integrated_bringup::DemoTaskController;
using integrated_bringup::DemoWbcController;

// ── The verdict and its wording ─────────────────────────────────────────────

rtc::DeviceNameConfig DeviceWithTip(const std::string& tip_link) {
  rtc::DeviceNameConfig device;
  device.device_name = "arm";
  rtc::DeviceUrdfConfig urdf;
  urdf.root_link = "base";
  urdf.tip_link = tip_link;
  device.urdf = urdf;
  return device;
}

TEST(ArmTipUnresolvedReason, NothingToRefuseWithoutAnArmModelOrWithAResolvedTip) {
  const auto device = DeviceWithTip("no_such_link");
  EXPECT_EQ(ArmTipUnresolvedReason(/*has_arm_model=*/false, /*tip_resolved=*/false, "arm", &device),
            "");
  EXPECT_EQ(ArmTipUnresolvedReason(true, /*tip_resolved=*/true, "arm", &device), "");
  EXPECT_EQ(ArmTipUnresolvedReason(false, false, "arm", nullptr), "");
}

TEST(ArmTipUnresolvedReason, NamesTheLinkThatIsNotOnTheModel) {
  const auto device = DeviceWithTip("no_such_link");
  const std::string reason = ArmTipUnresolvedReason(true, false, "arm", &device);
  EXPECT_NE(reason.find("'no_such_link' is not a frame of the model"), std::string::npos) << reason;
  EXPECT_NE(reason.find("primary device 'arm'"), std::string::npos) << reason;
  // ... and says where to fix it.
  EXPECT_NE(reason.find("urdf.sub_models.arm.tip_link"), std::string::npos) << reason;
  EXPECT_NE(reason.find("devices.arm.urdf.tip_link"), std::string::npos) << reason;
}

// Three ways a device config ends up with no tip link: an empty string, no
// `urdf` block, no device config at all. They are one case to the operator.
TEST(ArmTipUnresolvedReason, SaysWhenNoTipLinkWasGiven) {
  const auto empty_tip = DeviceWithTip("");
  rtc::DeviceNameConfig no_urdf;
  no_urdf.device_name = "arm";
  const std::vector<const rtc::DeviceNameConfig*> devices = {&empty_tip, &no_urdf, nullptr};
  for (const rtc::DeviceNameConfig* device : devices) {
    const std::string reason = ArmTipUnresolvedReason(true, false, "arm", device);
    EXPECT_NE(reason.find("no arm tip link"), std::string::npos) << reason;
    EXPECT_EQ(reason.find("is not a frame of the model"), std::string::npos) << reason;
    EXPECT_NE(reason.find("urdf.sub_models.arm"), std::string::npos) << reason;
  }
}

// ── The four controllers ────────────────────────────────────────────────────

std::map<std::string, rtc::DeviceNameConfig> WithArmTip(const std::string& tip_link) {
  auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  devices.at("iiwa7").urdf->tip_link = tip_link;
  return devices;
}

std::map<std::string, rtc::DeviceNameConfig> WithNoArmLinks() {
  auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  devices.at("iiwa7").urdf.reset();
  return devices;
}

class ArmTipResolution : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  template <class Ctrl>
  static void ExpectRefused(const std::map<std::string, rtc::DeviceNameConfig>& devices,
                            const std::string& tag, const std::string& reason) {
    for (const BringUp order : {BringUp::kControllerManager, BringUp::kConfigLoadedTwice}) {
      const std::string who = std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order);

      const auto good =
          fx::ConfigureIiwa7Leap<Ctrl>(order, fx::MakeIiwa7LeapDeviceConfigs(), tag + "_ok");
      EXPECT_EQ(good.rc, Ctrl::CallbackReturn::SUCCESS) << who;
      EXPECT_EQ(good.ctrl->ArmTipConfigError(), "") << who;

      const auto bad = fx::ConfigureIiwa7Leap<Ctrl>(order, devices, tag);
      EXPECT_EQ(bad.rc, Ctrl::CallbackReturn::FAILURE) << who;
      EXPECT_NE(bad.ctrl->ArmTipConfigError().find(reason), std::string::npos)
          << who << ": refused for another reason, or not at all — '"
          << bad.ctrl->ArmTipConfigError() << "'";
      // Nothing else is wrong with this config: the arm tip is the reason.
      EXPECT_EQ(bad.ctrl->HandFkWiringErrorForTesting(), "") << who;
      EXPECT_EQ(bad.ctrl->MomentumObserverConfigErrorForTesting(), "") << who;
      if constexpr (requires { bad.ctrl->IsBaseFrameMismatchForTesting(); }) {
        EXPECT_FALSE(bad.ctrl->IsBaseFrameMismatchForTesting()) << who;
      }
    }
  }

  template <class Ctrl>
  static void ExpectBothRefusals() {
    ExpectRefused<Ctrl>(WithArmTip("no_such_link"), "tip_unknown",
                        "'no_such_link' is not a frame of the model");
    ExpectRefused<Ctrl>(WithNoArmLinks(), "tip_missing", "no arm tip link");
  }

  // A controller brought up with no URDF has no arm model, so there is no arm
  // tip to resolve — whatever links its device config names. It configures, and
  // getting that far means OnDeviceConfigsSet did not go through a null handle.
  template <class Ctrl>
  static void ExpectModelLessControllerConfigures() {
    for (const BringUp order : {BringUp::kControllerManager, BringUp::kConfigLoadedTwice}) {
      const std::string who = std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order);
      const auto up = fx::ConfigureIiwa7Leap<Ctrl>(order, fx::MakeIiwa7LeapDeviceConfigs(),
                                                   "no_model", /*with_model=*/false);
      EXPECT_EQ(up.rc, Ctrl::CallbackReturn::SUCCESS) << who;
      EXPECT_EQ(up.ctrl->ArmTipConfigError(), "") << who;
    }
  }
};

TEST_F(ArmTipResolution, JointRefusesAnArmWithNoTip) {
  ExpectBothRefusals<DemoJointController>();
}

TEST_F(ArmTipResolution, TaskRefusesAnArmWithNoTip) {
  ExpectBothRefusals<DemoTaskController>();
}

TEST_F(ArmTipResolution, ComplianceRefusesAnArmWithNoTip) {
  ExpectBothRefusals<DemoComplianceController>();
}

TEST_F(ArmTipResolution, WbcRefusesAnArmWithNoTip) {
  ExpectBothRefusals<DemoWbcController>();
}

TEST_F(ArmTipResolution, JointWithoutAModelConfigures) {
  ExpectModelLessControllerConfigures<DemoJointController>();
}

TEST_F(ArmTipResolution, TaskWithoutAModelConfigures) {
  ExpectModelLessControllerConfigures<DemoTaskController>();
}

TEST_F(ArmTipResolution, ComplianceWithoutAModelConfigures) {
  ExpectModelLessControllerConfigures<DemoComplianceController>();
}

TEST_F(ArmTipResolution, WbcWithoutAModelConfigures) {
  ExpectModelLessControllerConfigures<DemoWbcController>();
}

}  // namespace
