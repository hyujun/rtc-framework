// locomotion_controller's Joy command (joy_command.hpp): the mapping bit for bit,
// the zero rules, the config refusals, the layout copy against rtc_teleop_input's
// installed table, and the shipped g1_loco joy overlay. Pure — no node, no spin.
// The ROS path (subscription → filter → planner, the stop branches) is in
// test_locomotion_mpc's LocomotionCommandTest.
#include "integrated_bringup/controllers/locomotion/joy_command.hpp"
#include "locomotion_controller_rig.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

using integrated_bringup::ParseLocomotionControllerConfig;
using integrated_bringup::locomotion::JoyCommandMapping;
using integrated_bringup::locomotion::JoyCommandResult;
using integrated_bringup::locomotion::JoyCommandScale;
using integrated_bringup::locomotion::JoyToCommand;
using integrated_bringup::locomotion::kStandardGamepadAxes;
using integrated_bringup::locomotion::kStandardGamepadButtons;
using integrated_bringup::locomotion::ParseJoyCommandMapping;
using loco_rig::ExpectRefused;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// The shipped g1_loco mapping (lefty / leftx / rightx, LB) and command.v_max /
// hdot_max, written out.
constexpr JoyCommandMapping kMap{1, 0, 2, -1, 9};
constexpr JoyCommandScale kScale{2.0, 1.2, 1.0, 0.1};

std::vector<std::int32_t> Buttons(int deadman_value, std::size_t n = 21) {
  std::vector<std::int32_t> b(n, 0);
  if (n > 9) {
    b[9] = deadman_value;
  }
  return b;
}

JoyCommandResult Convert(const std::vector<float>& axes, const std::vector<std::int32_t>& buttons,
                         const JoyCommandMapping& map = kMap) {
  return JoyToCommand(map, kScale, axes, buttons);
}

bool BitEqual(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

void ExpectCommand(const JoyCommandResult& r, double vx, double vy, double wz, double hdot = 0.0) {
  EXPECT_TRUE(BitEqual(r.twist.linear_x, vx)) << r.twist.linear_x << " vs " << vx;
  EXPECT_TRUE(BitEqual(r.twist.linear_y, vy)) << r.twist.linear_y << " vs " << vy;
  EXPECT_TRUE(BitEqual(r.twist.angular_z, wz)) << r.twist.angular_z << " vs " << wz;
  EXPECT_TRUE(BitEqual(r.twist.linear_z, hdot)) << r.twist.linear_z << " vs " << hdot;
  EXPECT_TRUE(BitEqual(r.twist.angular_x, 0.0));
  EXPECT_TRUE(BitEqual(r.twist.angular_y, 0.0));
}

void ExpectAllZero(const JoyCommandResult& r) {
  ExpectCommand(r, 0.0, 0.0, 0.0, 0.0);
}

// ── C2: the mapping, bit for bit (expected values are float32→double literals) ──

TEST(JoyToCommand, MapsTheAxesBitForBit) {
  struct Case {
    std::vector<float> axes;  // leftx, lefty, rightx, righty, triggerleft, triggerright
    double vx, vy, wz;
  };

  const std::vector<Case> cases{
      {{0, 0, 0, 0, 0, 0}, 0.0, 0.0, 0.0},
      {{0, 1.0F, 0, 0, 0, 0}, 2.0, 0.0, 0.0},
      {{0, -1.0F, 0, 0, 0, 0}, -2.0, 0.0, 0.0},
      {{0.5F, 0, -0.25F, 0, 0, 0}, 0.0, 0.6, -0.25},
      {{0, 0.15F, 0, 0, 0, 0}, 0.30000001192092896, 0.0, 0.0},
      {{0, 1.5F, 0, 0, 0, 0}, 2.0, 0.0, 0.0},    // clamped to 1
      {{-7.0F, 0, 0, 0, 0, 0}, 0.0, -1.2, 0.0},  // clamped to −1
      {{0, 0, 0.33333334F, 0, 0, 0}, 0.0, 0.0, 0.3333333432674408},
      {{0, 1e-38F, 0, 0, -1.0F, 1.0F}, 1.9999998700912808e-38, 0.0, 0.0},
      {{0.7F, -0.6F, 0, kNaN, kNaN, kInf}, -1.2000000476837158, 0.8399999856948852, 0.0},
  };
  for (std::size_t i = 0; i < cases.size(); ++i) {
    SCOPED_TRACE(i);
    const auto r = Convert(cases[i].axes, Buttons(1));
    EXPECT_EQ(r.deadman, 1);
    EXPECT_FALSE(r.rejected) << "an unmapped non-finite axis is not read";
    ExpectCommand(r, cases[i].vx, cases[i].vy, cases[i].wz);
  }
}

TEST(JoyToCommand, AMappedHeightRateUsesHdotMax) {
  JoyCommandMapping map = kMap;
  map.height_rate = 3;  // righty
  ExpectCommand(Convert({0, 0, 0, 0.5F, 0, 0}, Buttons(1), map), 0.0, 0.0, 0.0, 0.05);
  ExpectCommand(Convert({0, 0, 0, -2.0F, 0, 0}, Buttons(1), map), 0.0, 0.0, 0.0, -0.1);
}

// ── C3: the zero rules ──

TEST(JoyToCommand, TheDeadmanMustReadExactlyOne) {
  for (const int v : {0, 2, -1}) {
    SCOPED_TRACE(v);
    const auto r = Convert({0.5F, 1.0F, 1.0F, 0, 0, 0}, Buttons(v));
    EXPECT_EQ(r.deadman, 0);
    EXPECT_FALSE(r.rejected) << "released is a readable stop";
    ExpectAllZero(r);
  }
}

TEST(JoyToCommand, AnUnreadableMessageIsZeroAndRejected) {
  const std::vector<float> axes{0.5F, 1.0F, 1.0F, 0, 0, 0};
  const auto expect_rejected = [](const JoyCommandResult& r, const char* what) {
    EXPECT_TRUE(r.rejected) << what;
    ExpectAllZero(r);
  };
  expect_rejected(Convert(axes, Buttons(1, 9)), "buttons shorter than the deadman index");
  EXPECT_EQ(Convert(axes, Buttons(1, 9)).deadman, -1) << "not read: neither held nor released";
  expect_rejected(Convert(axes, {}), "no buttons");
  expect_rejected(Convert({0.5F, 1.0F}, Buttons(1)), "axes shorter than a mapped index");
  expect_rejected(Convert({}, Buttons(1)), "no axes");
  for (const float bad : {kNaN, kInf, -kInf}) {
    for (const std::size_t mapped : {0U, 1U, 2U}) {
      std::vector<float> a = axes;
      a[mapped] = bad;
      expect_rejected(Convert(a, Buttons(1)), "a mapped axis non-finite");
      expect_rejected(Convert(a, Buttons(0)), "non-finite with the deadman released");
    }
  }
}

// ── C6: config refusals (and the accepted shapes) ──

YAML::Node JoyConfig() {
  YAML::Node cfg = YAML::Clone(loco_rig::ShippedConfig());
  loco_rig::SetJoyCommand(cfg);
  return cfg;
}

TEST(JoyCommandConfig, TheTypeAndTheMapAreParsed) {
  const auto parsed = ParseLocomotionControllerConfig(JoyConfig());
  EXPECT_TRUE(parsed.velocity_command_joy);
  EXPECT_EQ(parsed.velocity_command_topic, "g1/joy");
  const auto& m = parsed.velocity_command_joy_map;
  EXPECT_EQ(m.linear_x, 1);
  EXPECT_EQ(m.linear_y, 0);
  EXPECT_EQ(m.angular_z, 2);
  EXPECT_EQ(m.height_rate, -1);
  EXPECT_EQ(m.deadman, 9);
  YAML::Node with_height = JoyConfig();
  with_height["velocity_command"]["joy"]["height_rate"] = "righty";
  EXPECT_EQ(ParseLocomotionControllerConfig(with_height).velocity_command_joy_map.height_rate, 3);
}

TEST(JoyCommandConfig, NoTypeAndTypeTwistParseAlike) {
  YAML::Node none = YAML::Clone(loco_rig::ShippedConfig());
  none["velocity_command"]["topic"] = "g1/cmd_vel";
  YAML::Node twist = YAML::Clone(none);
  twist["velocity_command"]["type"] = "twist";
  const auto a = ParseLocomotionControllerConfig(none);
  const auto b = ParseLocomotionControllerConfig(twist);
  EXPECT_FALSE(a.velocity_command_joy);
  EXPECT_FALSE(b.velocity_command_joy);
  EXPECT_EQ(a.velocity_command_topic, b.velocity_command_topic);
  EXPECT_EQ(std::memcmp(&a.velocity_command_joy_map, &b.velocity_command_joy_map,
                        sizeof(JoyCommandMapping)),
            0);
}

TEST(JoyCommandConfig, BadShapesAreRefused) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    ExpectRefused(JoyConfig(), spoil, what);
  };
  refused([](YAML::Node& c) { c["velocity_command"]["type"] = "gamepad"; }, "unknown type");
  refused([](YAML::Node& c) { c["velocity_command"].remove("joy"); }, "joy without joy:");
  refused([](YAML::Node& c) { c["velocity_command"]["type"] = "twist"; }, "twist with joy:");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"]["linear_x"] = "stick"; },
          "unknown layout name");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"].remove("deadman"); }, "no deadman");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"]["linear_x"] = "a"; },
          "a button where an axis goes");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"]["deadman"] = "lefty"; },
          "an axis where the deadman goes");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"]["angular_z"] = "lefty"; },
          "one axis on two commands");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"]["height_rate"] = "leftx"; },
          "height on an axis already used");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"]["scale"] = 1.0; },
          "unknown key under joy:");
  refused([](YAML::Node& c) { c["velocity_command"]["joy"] = "lefty"; }, "joy: not a map");
  refused([](YAML::Node& c) { c["velocity_command"]["qos"] = 1; }, "qos still refused");
}

TEST(JoyCommandConfig, ParseJoyCommandMappingNamesTheKey) {
  YAML::Node joy;
  joy["linear_x"] = "lefty";
  joy["linear_y"] = "leftx";
  joy["angular_z"] = "rightx";
  joy["deadman"] = "nope";
  try {
    static_cast<void>(ParseJoyCommandMapping(joy, "x.velocity_command.joy"));
    FAIL() << "accepted an unknown deadman";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("x.velocity_command.joy.deadman"), std::string::npos)
        << e.what();
  }
}

// ── C7: the C++ table is rtc_teleop_input's installed layout ──

TEST(StandardGamepadLayout, TheCopyMatchesTheInstalledTable) {
  const std::string path = ament_index_cpp::get_package_share_directory("rtc_teleop_input") +
                           "/config/standard_gamepad.yaml";
  const YAML::Node layout = YAML::LoadFile(path);
  const auto axes = layout["axes"].as<std::vector<std::string>>();
  const auto buttons = layout["buttons"].as<std::vector<std::string>>();
  ASSERT_EQ(axes.size(), kStandardGamepadAxes.size()) << path;
  ASSERT_EQ(buttons.size(), kStandardGamepadButtons.size()) << path;
  for (std::size_t i = 0; i < axes.size(); ++i) {
    EXPECT_EQ(axes[i], kStandardGamepadAxes[i]) << "axis " << i;
  }
  for (std::size_t i = 0; i < buttons.size(); ++i) {
    EXPECT_EQ(buttons[i], kStandardGamepadButtons[i]) << "button " << i;
  }
}

// ── C8: the shipped joy overlay parses, with the shipped mapping ──

TEST(JoyCommandOverlay, TheShippedOverlayParses) {
  const std::string path = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/g1_loco/sim_overlays/joy_command.yaml";
  const YAML::Node overlay = YAML::LoadFile(path);
  const YAML::Node vc = overlay["integrated_rt_controller"]["ros__parameters"]
                               ["locomotion_controller"]["velocity_command"];
  ASSERT_TRUE(vc.IsMap()) << path;
  // The launch applies it over the shipped controller YAML, key by key.
  YAML::Node cfg = YAML::Clone(loco_rig::ShippedConfig());
  for (const auto& kv : vc) {
    cfg["velocity_command"][kv.first.as<std::string>()] = YAML::Clone(kv.second);
  }
  const auto parsed = ParseLocomotionControllerConfig(cfg);
  EXPECT_TRUE(parsed.velocity_command_joy);
  EXPECT_EQ(parsed.velocity_command_topic, "g1/joy");
  const auto& m = parsed.velocity_command_joy_map;
  EXPECT_EQ(m.linear_x, 1) << "lefty";
  EXPECT_EQ(m.linear_y, 0) << "leftx";
  EXPECT_EQ(m.angular_z, 2) << "rightx";
  EXPECT_EQ(m.height_rate, 3) << "righty";
  EXPECT_EQ(m.deadman, 9) << "leftshoulder (LB)";
}

}  // namespace
