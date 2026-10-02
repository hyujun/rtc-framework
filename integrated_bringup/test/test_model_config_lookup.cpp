// ── FindTreeModel: the tree model declared for a device group ───────────────
//
// A device group and the model declared for it share one name, and every demo
// controller looks the secondary group's tree model up by it. They all go
// through this one function; these cases pin what it answers at the edges,
// where a hand-written loop and the function can differ.

#include "integrated_bringup/support/model_config_lookup.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using integrated_bringup::FindTreeModel;

rtc_urdf_bridge::ModelConfig Config() {
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.tree_models.push_back({"hand", "hand_base", {"tip_a", "tip_b"}});
  cfg.tree_models.push_back({"body", "pelvis", {"left_tip"}});
  return cfg;
}

TEST(FindTreeModel, ReturnsTheEntryWithThatName) {
  const auto cfg = Config();
  const auto* body = FindTreeModel(cfg, "body");
  ASSERT_NE(body, nullptr);
  EXPECT_EQ(body->root_link, "pelvis");
  // A pointer into the config, not a copy.
  EXPECT_EQ(body, &cfg.tree_models[1]);
  EXPECT_EQ(FindTreeModel(cfg, "hand"), &cfg.tree_models[0]);
}

TEST(FindTreeModel, ReturnsNullWhenNoEntryHasThatName) {
  const auto cfg = Config();
  EXPECT_EQ(FindTreeModel(cfg, "arm"), nullptr);
  // A name is matched whole.
  EXPECT_EQ(FindTreeModel(cfg, "han"), nullptr);
  EXPECT_EQ(FindTreeModel(cfg, "hand2"), nullptr);
  EXPECT_EQ(FindTreeModel(rtc_urdf_bridge::ModelConfig{}, "hand"), nullptr);
}

// "No secondary group" arrives here as an empty name. It names no model — not
// even one that was itself declared with an empty name.
TEST(FindTreeModel, AnEmptyNameMatchesNothing) {
  auto cfg = Config();
  EXPECT_EQ(FindTreeModel(cfg, ""), nullptr);
  cfg.tree_models.push_back({"", "world", {}});
  EXPECT_EQ(FindTreeModel(cfg, ""), nullptr);
}

TEST(FindTreeModel, TheFirstOfTwoEntriesWithOneNameWins) {
  auto cfg = Config();
  cfg.tree_models.push_back({"hand", "other_base", {"tip_z"}});
  const auto* hand = FindTreeModel(cfg, "hand");
  ASSERT_NE(hand, nullptr);
  EXPECT_EQ(hand->root_link, "hand_base");
  EXPECT_EQ(hand, &cfg.tree_models[0]);
}

}  // namespace
