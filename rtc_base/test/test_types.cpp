// ── rtc_base types tests ─────────────────────────────────────────────────────
//
// Guards the wire-string spelling of the shared CommandType / GoalType mappings.
// CommandTypeToString is the single source the device backends (mujoco_native,
// udp_hand_native) encode onto JointCommand.command_type, so a typo here would
// silently desync the wire format across every backend.
//
// Also pins the floating-base lane's defaults and freshness helpers (WD-16):
// the identity default of `base_pose` and the fail-closed readings of
// BaseAgeNs / IsBaseFresh, which the first consumer would otherwise re-derive.

#include "rtc_base/types/types.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>

namespace {

TEST(CommandTypeToStringTest, MapsEveryEnumerator) {
  EXPECT_EQ(std::string_view{rtc::CommandTypeToString(rtc::CommandType::kPosition)}, "position");
  EXPECT_EQ(std::string_view{rtc::CommandTypeToString(rtc::CommandType::kTorque)}, "torque");
  EXPECT_EQ(std::string_view{rtc::CommandTypeToString(rtc::CommandType::kPdFeedforward)},
            "pd_feedforward");
}

TEST(GoalTypeToStringTest, MapsEveryEnumerator) {
  EXPECT_EQ(std::string_view{rtc::GoalTypeToString(rtc::GoalType::kJoint)}, "joint");
  EXPECT_EQ(std::string_view{rtc::GoalTypeToString(rtc::GoalType::kTask)}, "task");
}

// A default-constructed device reports the base lane as never received, with a
// well-formed (identity) pose — not a zero quaternion — and the mirror-seam
// polarity of everything else at zero / false.
TEST(FloatingBaseLaneTest, DefaultIsIdentityPoseAndNeverReceived) {
  const rtc::DeviceState dev{};
  EXPECT_EQ(0.0, dev.base_pose[0]);
  EXPECT_EQ(0.0, dev.base_pose[1]);
  EXPECT_EQ(0.0, dev.base_pose[2]);
  EXPECT_EQ(0.0, dev.base_pose[3]);
  EXPECT_EQ(0.0, dev.base_pose[4]);
  EXPECT_EQ(0.0, dev.base_pose[5]);
  EXPECT_EQ(1.0, dev.base_pose[6]) << "q_w — the default must be a rotation";
  for (double v : dev.base_twist) {
    EXPECT_EQ(0.0, v);
  }
  EXPECT_EQ(0, dev.num_imu_sites);
  EXPECT_EQ(0, dev.base_recv_steady_ns);
  EXPECT_EQ(0U, dev.base_sequence);
  EXPECT_FALSE(dev.base_valid);
  EXPECT_LT(rtc::BaseAgeNs(dev, /*now_steady_ns=*/1'000), 0)
      << "never received is negative, not old";
  EXPECT_FALSE(rtc::IsBaseFresh(dev, /*now_steady_ns=*/1'000, /*max_age_ns=*/1'000'000));
}

// Same truth table as IsSensorGroupFresh: BOTH the producer's flag and the
// consumer's age bound are required, and a non-positive bound withholds.
TEST(FloatingBaseLaneTest, FreshnessNeedsFlagAndAgeAndPositiveBound) {
  rtc::DeviceState dev{};
  dev.base_recv_steady_ns = 5'000;
  dev.base_valid = true;
  EXPECT_EQ(2'000, rtc::BaseAgeNs(dev, /*now_steady_ns=*/7'000));
  EXPECT_TRUE(rtc::IsBaseFresh(dev, 7'000, /*max_age_ns=*/2'000)) << "age == bound is fresh";
  EXPECT_FALSE(rtc::IsBaseFresh(dev, 7'000, /*max_age_ns=*/1'999));
  EXPECT_FALSE(rtc::IsBaseFresh(dev, 7'000, /*max_age_ns=*/0)) << "zero bound withholds";
  EXPECT_FALSE(rtc::IsBaseFresh(dev, 7'000, /*max_age_ns=*/-1));
  dev.base_valid = false;
  EXPECT_FALSE(rtc::IsBaseFresh(dev, 7'000, /*max_age_ns=*/1'000'000))
      << "a stale verdict from the producer is not overridden by a generous bound";
  EXPECT_EQ(2'000, rtc::BaseAgeNs(dev, 7'000)) << "the age itself is still reported";
}

TEST(DeviceCapabilityTest, FloatingBaseIsItsOwnBit) {
  const uint16_t caps = rtc::DeviceCapability::kJointState | rtc::DeviceCapability::kFloatingBase;
  EXPECT_TRUE(rtc::HasCapability(caps, rtc::DeviceCapability::kFloatingBase));
  EXPECT_FALSE(rtc::HasCapability(caps, rtc::DeviceCapability::kInference));
  EXPECT_FALSE(rtc::HasCapability(static_cast<uint16_t>(rtc::DeviceCapability::kInference),
                                  rtc::DeviceCapability::kFloatingBase));
}

}  // namespace
