// virtual_tcp_* runtime parameters (#847): the node-free half of the helper.
//
// ApplyVirtualTcpParameter is the whole validation the three controllers' set
// callbacks run for these keys, so its refusals are asserted here once rather
// than per controller. The controller suites pin the wiring around it (the
// inactive gate, the hand flag); the compliance suite pins the node half,
// DeclareVirtualTcpParameters, on a real LifecycleNode.

#include "integrated_bringup/support/virtual_tcp_params.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace {

using integrated_bringup::ApplyVirtualTcpParameter;
using integrated_bringup::IsVirtualTcpParameter;
using integrated_bringup::kVirtualTcpModeNames;
using integrated_bringup::kVirtualTcpModeParam;
using integrated_bringup::kVirtualTcpOffsetParam;
using integrated_bringup::kVirtualTcpOrientationParam;
using integrated_bringup::ParseVirtualTcpMode;
using integrated_bringup::VirtualTcpConfig;
using integrated_bringup::VirtualTcpMode;
using integrated_bringup::VirtualTcpModeName;

/// A config no valid set produces by accident, so "untouched" is observable.
VirtualTcpConfig Sentinel() {
  VirtualTcpConfig c;
  c.mode = VirtualTcpMode::kCentroid;
  c.offset = {0.1, 0.2, 0.3};
  c.orientation = {0.4, 0.5, 0.6};
  return c;
}

void ExpectUntouched(const VirtualTcpConfig& c) {
  const VirtualTcpConfig s = Sentinel();
  EXPECT_EQ(c.mode, s.mode);
  EXPECT_EQ(c.offset, s.offset);
  EXPECT_EQ(c.orientation, s.orientation);
}

TEST(VirtualTcpParams, ModeNamesRoundTripAndUnknownIsNotFoldedIntoDisabled) {
  for (const auto& [mode, name] : kVirtualTcpModeNames) {
    EXPECT_EQ(std::string(VirtualTcpModeName(mode)), name);
    const auto parsed = ParseVirtualTcpMode(name);
    ASSERT_TRUE(parsed.has_value()) << name;
    EXPECT_EQ(*parsed, mode);
  }
  EXPECT_FALSE(ParseVirtualTcpMode("Constant").has_value());
  EXPECT_FALSE(ParseVirtualTcpMode("").has_value());
}

TEST(VirtualTcpParams, OnlyTheThreeKeysAreClaimed) {
  EXPECT_TRUE(IsVirtualTcpParameter(kVirtualTcpModeParam));
  EXPECT_TRUE(IsVirtualTcpParameter(kVirtualTcpOffsetParam));
  EXPECT_TRUE(IsVirtualTcpParameter(kVirtualTcpOrientationParam));
  EXPECT_FALSE(IsVirtualTcpParameter("virtual_tcp"));
  EXPECT_FALSE(IsVirtualTcpParameter("kp_translation"));

  auto c = Sentinel();
  EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter("kp_translation", 1.0), true, c), nullptr);
  ExpectUntouched(c);
}

TEST(VirtualTcpParams, EveryModeIsAcceptedWithAHand) {
  for (const auto& [mode, name] : kVirtualTcpModeNames) {
    auto c = Sentinel();
    const char* why =
        ApplyVirtualTcpParameter(rclcpp::Parameter(kVirtualTcpModeParam, std::string(name)),
                                 /*has_hand=*/true, c);
    EXPECT_EQ(why, nullptr) << name << ": " << why;
    EXPECT_EQ(c.mode, mode);
    EXPECT_EQ(c.offset, Sentinel().offset) << "a mode set wrote the offset";
  }
}

TEST(VirtualTcpParams, WithoutAHandOnlyDisabledIsAccepted) {
  for (const auto& [mode, name] : kVirtualTcpModeNames) {
    auto c = Sentinel();
    const char* why =
        ApplyVirtualTcpParameter(rclcpp::Parameter(kVirtualTcpModeParam, std::string(name)),
                                 /*has_hand=*/false, c);
    if (mode == VirtualTcpMode::kDisabled) {
      EXPECT_EQ(why, nullptr) << why;
      EXPECT_EQ(c.mode, VirtualTcpMode::kDisabled);
    } else {
      EXPECT_NE(why, nullptr) << name << " accepted without a hand model";
      ExpectUntouched(c);
    }
  }
}

TEST(VirtualTcpParams, ModeRefusesAnUnknownNameAndAWrongType) {
  auto c = Sentinel();
  EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter(kVirtualTcpModeParam, "fixed"), true, c),
            nullptr);
  ExpectUntouched(c);
  EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter(kVirtualTcpModeParam, int64_t{3}), true, c),
            nullptr);
  ExpectUntouched(c);
}

class VirtualTcpTripleParams : public ::testing::TestWithParam<const char*> {
 protected:
  static std::array<double, 3>& Field(VirtualTcpConfig& c, const char* name) {
    return std::string(name) == kVirtualTcpOffsetParam ? c.offset : c.orientation;
  }
};

TEST_P(VirtualTcpTripleParams, ThreeFiniteValuesAreWrittenAndNothingElse) {
  auto c = Sentinel();
  const char* why = ApplyVirtualTcpParameter(
      rclcpp::Parameter(GetParam(), std::vector<double>{-0.01, 0.0, 0.15}), true, c);
  ASSERT_EQ(why, nullptr) << why;
  EXPECT_EQ(Field(c, GetParam()), (std::array<double, 3>{-0.01, 0.0, 0.15}));
  EXPECT_EQ(c.mode, Sentinel().mode);
}

TEST_P(VirtualTcpTripleParams, WrongLengthIsRefused) {
  for (const auto& v : {std::vector<double>{}, std::vector<double>{0.0, 0.1},
                        std::vector<double>{0.0, 0.1, 0.2, 0.3}}) {
    auto c = Sentinel();
    EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter(GetParam(), v), true, c), nullptr)
        << v.size() << " values accepted";
    ExpectUntouched(c);
  }
}

TEST_P(VirtualTcpTripleParams, NonFiniteIsRefused) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (const auto& v : {std::vector<double>{kNan, 0.0, 0.0}, std::vector<double>{0.0, kInf, 0.0},
                        std::vector<double>{0.0, 0.0, -kInf}}) {
    auto c = Sentinel();
    EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter(GetParam(), v), true, c), nullptr);
    ExpectUntouched(c);
  }
}

// `ros2 param set … "[0, 0, 1]"` arrives as an integer array. The declared
// parameter's static type refuses it before the callback in production; the
// helper refuses it too rather than reading it as doubles.
TEST_P(VirtualTcpTripleParams, IntegerArrayAndScalarAreRefused) {
  auto c = Sentinel();
  EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter(GetParam(), std::vector<int64_t>{0, 0, 1}),
                                     true, c),
            nullptr);
  ExpectUntouched(c);
  EXPECT_NE(ApplyVirtualTcpParameter(rclcpp::Parameter(GetParam(), 0.1), true, c), nullptr);
  ExpectUntouched(c);
}

INSTANTIATE_TEST_SUITE_P(OffsetAndOrientation, VirtualTcpTripleParams,
                         ::testing::Values(kVirtualTcpOffsetParam, kVirtualTcpOrientationParam));

}  // namespace
