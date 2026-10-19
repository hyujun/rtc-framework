// ── A hand sensor_layout the fingertip parser cannot read refuses configure ──
//
// The joint, task and compliance controllers read the hand's raw sensor lane
// at the fixed stride kHandSensorValuesPerFingertipCapacity (8 barometer + 3
// ToF), and all four demo controllers read its inference lane at
// kHandInferenceValuesPerFingertipCapacity (7). The device's sensor_layout
// sets the stride the backend WRITES at. When the two differ, every fingertip
// after the first is read from its neighbour's slots — finite values under the
// right names, on the RT tick, with nothing to flag them (#885). The
// controller manager only bounds the strides by the buffers' capacity, so the
// configure of each controller is where a stride it does not match is stopped.
//
// What this file pins, for joint, task, compliance and wbc on a real arm + hand:
//
//   - raw stride 12 (8 + 3 + 1 spare)                      → FAILURE
//   - 4 + 3 values in an 11-wide stride                   → FAILURE
//   - 8 + 3 values in a 7-wide stride                     → FAILURE
//   - inference stride 8                                  → FAILURE
//   - the shipped iiwa7_leap layout (inference-only, 7)   → SUCCESS
//   - the shipped fingertip hand layout (8 / 3 / 11 / 7)  → SUCCESS
//
// Each refusal is pinned the way test_arm_tip_resolution.cpp pins its own: the
// controller's verdict names the layout, and the other configure-time errors a
// test can read are empty. Both bring-up orders run — the verdict is read off
// the device config at on_configure, and a value latched by an earlier load
// would hold on one order and not on the other.

#include "iiwa7_leap_controller_yamls.hpp"
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
using fx::BringUp;
using integrated_bringup::DemoComplianceController;
using integrated_bringup::DemoJointController;
using integrated_bringup::DemoTaskController;
using integrated_bringup::DemoWbcController;
using integrated_bringup::HandSensorLayoutError;

constexpr const char* kVerdict = "does not match the fixed fingertip parser";

rtc::DeviceSensorLayout Layout(int primary, int secondary, int values_per_group,
                               int inference_values_per_group) {
  rtc::DeviceSensorLayout layout;
  layout.primary_count_per_group = primary;
  layout.secondary_count_per_group = secondary;
  layout.values_per_group = values_per_group;
  layout.inference_values_per_group = inference_values_per_group;
  return layout;
}

// ── The verdict and its wording ─────────────────────────────────────────────

TEST(HandSensorLayoutError, AcceptsNoLayoutAndEveryLayoutTheParserReads) {
  EXPECT_EQ(HandSensorLayoutError(std::nullopt, "hand"), "");
  // Shipped: the fingertip hand, the force-only hands, and an inference-only
  // block with every count left out.
  EXPECT_EQ(HandSensorLayoutError(Layout(8, 3, 11, 7), "hand"), "");
  EXPECT_EQ(HandSensorLayoutError(Layout(0, 0, 0, 7), "hand"), "");
  // Either lane may be absent on its own.
  EXPECT_EQ(HandSensorLayoutError(Layout(0, 0, 0, 0), "hand"), "");
  EXPECT_EQ(HandSensorLayoutError(Layout(8, 3, 11, 0), "hand"), "");
}

TEST(HandSensorLayoutError, RefusesEveryStrideTheParserDoesNotRead) {
  const std::vector<std::pair<std::string, rtc::DeviceSensorLayout>> cases = {
      {"raw stride 12", Layout(8, 3, 12, 7)},      {"4 barometers", Layout(4, 3, 11, 7)},
      {"raw stride 7", Layout(8, 3, 7, 7)},        {"counts without a stride", Layout(8, 3, 0, 7)},
      {"inference stride 8", Layout(8, 3, 11, 8)}, {"inference stride 4", Layout(0, 0, 0, 4)},
  };
  for (const auto& [label, layout] : cases) {
    const std::string reason = HandSensorLayoutError(layout, "hand");
    EXPECT_NE(reason.find(kVerdict), std::string::npos) << label << ": '" << reason << "'";
    EXPECT_NE(reason.find("device 'hand'"), std::string::npos) << label << ": '" << reason << "'";
  }
}

TEST(HandSensorLayoutError, NamesTheValuesItRefusedAndTheOnesItReads) {
  const std::string reason = HandSensorLayoutError(Layout(8, 3, 12, 7), "leap");
  EXPECT_NE(reason.find("device 'leap'"), std::string::npos) << reason;
  EXPECT_NE(reason.find("primary=8 secondary=3 values_per_group=12 inference_values_per_group=7"),
            std::string::npos)
      << reason;
  EXPECT_NE(reason.find("0/0/0 or 8/3/11"), std::string::npos) << reason;
  EXPECT_NE(reason.find("inference_values_per_group 0 or 7"), std::string::npos) << reason;
}

// ── The four controllers ────────────────────────────────────────────────────

std::map<std::string, rtc::DeviceNameConfig> WithHandLayout(const rtc::DeviceSensorLayout& layout) {
  auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  devices.at("leap").sensor_layout = layout;
  return devices;
}

class HandSensorLayoutGate : public ::testing::Test {
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
  static void ExpectConfigures(const std::map<std::string, rtc::DeviceNameConfig>& devices,
                               const std::string& tag) {
    for (const BringUp order : {BringUp::kControllerManager, BringUp::kConfigLoadedTwice}) {
      const std::string who =
          std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order) + "/" + tag;
      const auto up = fx::ConfigureIiwa7Leap<Ctrl>(order, devices, tag);
      EXPECT_EQ(up.rc, Ctrl::CallbackReturn::SUCCESS) << who;
      EXPECT_EQ(up.ctrl->HandSensorLayoutConfigError(), "") << who;
    }
  }

  template <class Ctrl>
  static void ExpectRefused(const rtc::DeviceSensorLayout& layout, const std::string& tag) {
    for (const BringUp order : {BringUp::kControllerManager, BringUp::kConfigLoadedTwice}) {
      const std::string who =
          std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order) + "/" + tag;
      const auto bad = fx::ConfigureIiwa7Leap<Ctrl>(order, WithHandLayout(layout), tag);
      EXPECT_EQ(bad.rc, Ctrl::CallbackReturn::FAILURE) << who;
      const std::string reason = bad.ctrl->HandSensorLayoutConfigError();
      EXPECT_NE(reason.find(kVerdict), std::string::npos)
          << who << ": refused for another reason, or not at all — '" << reason << "'";
      EXPECT_NE(reason.find("device 'leap'"), std::string::npos) << who << ": '" << reason << "'";
      // Nothing else is wrong with this config: the layout is the reason.
      EXPECT_EQ(bad.ctrl->ArmTipConfigError(), "") << who;
      EXPECT_EQ(bad.ctrl->HandFkWiringErrorForTesting(), "") << who;
      EXPECT_EQ(bad.ctrl->MomentumObserverConfigErrorForTesting(), "") << who;
    }
  }

  template <class Ctrl>
  static void ExpectTheGate() {
    ExpectConfigures<Ctrl>(fx::MakeIiwa7LeapDeviceConfigs(), "shipped_leap");
    ExpectConfigures<Ctrl>(WithHandLayout(Layout(8, 3, 11, 7)), "fingertip_hand");
    ExpectRefused<Ctrl>(Layout(8, 3, 12, 7), "raw_stride_12");
    ExpectRefused<Ctrl>(Layout(4, 3, 11, 7), "four_barometers");
    ExpectRefused<Ctrl>(Layout(8, 3, 7, 7), "raw_stride_7");
    ExpectRefused<Ctrl>(Layout(0, 0, 0, 8), "inference_stride_8");
  }
};

TEST_F(HandSensorLayoutGate, JointRefusesAStrideItDoesNotRead) {
  ExpectTheGate<DemoJointController>();
}

TEST_F(HandSensorLayoutGate, TaskRefusesAStrideItDoesNotRead) {
  ExpectTheGate<DemoTaskController>();
}

TEST_F(HandSensorLayoutGate, ComplianceRefusesAStrideItDoesNotRead) {
  ExpectTheGate<DemoComplianceController>();
}

TEST_F(HandSensorLayoutGate, WbcRefusesAStrideItDoesNotRead) {
  ExpectTheGate<DemoWbcController>();
}

}  // namespace
