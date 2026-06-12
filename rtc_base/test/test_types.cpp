// ── rtc_base types enum-to-string tests ──────────────────────────────────────
//
// Guards the wire-string spelling of the shared CommandType / GoalType mappings.
// CommandTypeToString is the single source the device backends (mujoco_native,
// udp_hand_native) encode onto JointCommand.command_type, so a typo here would
// silently desync the wire format across every backend.

#include "rtc_base/types/types.hpp"

#include <gtest/gtest.h>

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

}  // namespace
