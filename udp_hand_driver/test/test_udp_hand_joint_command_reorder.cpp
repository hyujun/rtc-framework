// Tests for name-based reordering of incoming /…/joint_command.
//
// The publisher (controller/backend) labels JointCommand values in its own
// joint_names order (URDF / _base.yaml). The hand serves motors in firmware
// slot order (joint_state_names / UDP wire order). For p1b these differ only at
// slots 0/1 (thumb_cmc_aa ↔ thumb_cmc_fe). BuildCommandNameReorder maps
// firmware slot i ← the published index of that joint so the positional
// CommLoop write receives each command on the correct motor.

#include "udp_hand_driver/udp_hand_constants.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

namespace {

using udp_hand_driver::BuildCommandNameReorder;
using udp_hand_driver::kNumHandMotors;

// Firmware / UDP wire order (p1b udp_hand_node_p1b.yaml joint_state_names).
const std::vector<std::string> kWire = {
    "thumb_cmc_fe_joint",  "thumb_cmc_aa_joint", "thumb_mcp_joint",    "thumb_dip_fe_joint",
    "index_mcp_aa_joint",  "index_mcp_fe_joint", "index_dip_fe_joint", "middle_mcp_fe_joint",
    "middle_dip_fe_joint", "ring_mcp_fe_joint"};

// Controller / URDF order (_base.yaml joint_state_names) — slots 0/1 swapped.
const std::vector<std::string> kUrdf = {
    "thumb_cmc_aa_joint",  "thumb_cmc_fe_joint", "thumb_mcp_joint",    "thumb_dip_fe_joint",
    "index_mcp_aa_joint",  "index_mcp_fe_joint", "index_dip_fe_joint", "middle_mcp_fe_joint",
    "middle_dip_fe_joint", "ring_mcp_fe_joint"};

// Identical publisher/firmware order → identity map (no reorder needed).
TEST(CommandNameReorder, IdentityWhenSameOrder) {
  const auto map = BuildCommandNameReorder(kWire, kWire);
  for (int i = 0; i < kNumHandMotors; ++i) {
    EXPECT_EQ(map[static_cast<std::size_t>(i)], i) << "slot " << i;
  }
}

// URDF-ordered publisher vs wire-ordered firmware: thumb_cmc slots swap, rest
// pass through. This is the exact p1b failure the fix addresses.
TEST(CommandNameReorder, SwapsThumbCmcForUrdfOrder) {
  const auto map = BuildCommandNameReorder(kWire, kUrdf);
  EXPECT_EQ(map[0], 1);  // wire slot 0 = thumb_cmc_fe → URDF index 1
  EXPECT_EQ(map[1], 0);  // wire slot 1 = thumb_cmc_aa → URDF index 0
  for (int i = 2; i < kNumHandMotors; ++i) {
    EXPECT_EQ(map[static_cast<std::size_t>(i)], i) << "slot " << i;
  }
}

// A firmware joint missing from the published names maps to -1 (the callback
// then falls back to positional consumption).
TEST(CommandNameReorder, UnmatchedJointYieldsNegativeOne) {
  std::vector<std::string> partial = kUrdf;
  partial[0] = "nonexistent_joint";  // thumb_cmc_aa no longer published
  const auto map = BuildCommandNameReorder(kWire, partial);
  EXPECT_EQ(map[1], -1);  // wire slot 1 = thumb_cmc_aa not found
  EXPECT_EQ(map[0], 1);   // thumb_cmc_fe still resolves to URDF index 1
}

// Empty published names → nothing matches → all -1 (positional fallback).
TEST(CommandNameReorder, EmptyPublishedNamesAllNegativeOne) {
  const auto map = BuildCommandNameReorder(kWire, {});
  for (int i = 0; i < kNumHandMotors; ++i) {
    EXPECT_EQ(map[static_cast<std::size_t>(i)], -1) << "slot " << i;
  }
}

// End-to-end apply step mirrors the callback: cmd[i] = values[map[i]] - offset[i].
// Values are laid out in URDF (publisher) order; the result must be in wire
// (firmware) order, with the two thumb_cmc commands landing on the right slot.
TEST(CommandNameReorder, ApplyReordersValuesIntoFirmwareOrder) {
  const auto map = BuildCommandNameReorder(kWire, kUrdf);

  std::array<double, kNumHandMotors> values{};  // URDF order
  for (int i = 0; i < kNumHandMotors; ++i) {
    values[static_cast<std::size_t>(i)] = static_cast<double>(i) * 0.1;
  }
  std::array<float, kNumHandMotors> offset{};
  offset.fill(0.0F);

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    const auto src = static_cast<std::size_t>(map[static_cast<std::size_t>(i)]);
    cmd[static_cast<std::size_t>(i)] =
        static_cast<float>(values[src]) - offset[static_cast<std::size_t>(i)];
  }

  // Firmware slot 0 (thumb_cmc_fe) must carry the URDF-index-1 command (0.1),
  // slot 1 (thumb_cmc_aa) the URDF-index-0 command (0.0).
  EXPECT_FLOAT_EQ(cmd[0], 0.1F);
  EXPECT_FLOAT_EQ(cmd[1], 0.0F);
  for (int i = 2; i < kNumHandMotors; ++i) {
    EXPECT_FLOAT_EQ(cmd[static_cast<std::size_t>(i)], static_cast<float>(i) * 0.1F) << "slot " << i;
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
