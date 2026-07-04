/// Unit tests for finger_resolver.hpp: the finger→joint-index strategies
/// (Seam C). PrefixFingerResolver reproduces the legacy joint-name prefix
/// matching; ExplicitFingerResolver validates the config-map path used by
/// hands whose joint names carry no finger prefix (e.g. LEAP numeric names).

#include "ur5e_bt_coordinator/finger_resolver.hpp"

#include <gtest/gtest.h>

#include <memory>

using rtc_bt::ExplicitFingerResolver;
using rtc_bt::FingerResolver;
using rtc_bt::PrefixFingerResolver;

// ── PrefixFingerResolver == legacy FingerJointIndices behavior ─────────────

TEST(PrefixFingerResolver, Proto1bVariableDof) {
  // proto_1b hand joint order (thumb:4 / index:3 / middle:2 / ring:1).
  const std::vector<std::string> names = {
      "thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_aa",  "thumb_mcp_fe",  "index_mcp_aa",
      "index_mcp_fe", "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "ring_mcp_fe"};
  PrefixFingerResolver r;

  EXPECT_EQ(r.Resolve(names, "thumb"), (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(r.Resolve(names, "index"), (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(r.Resolve(names, "middle"), (std::vector<int>{7, 8}));
  EXPECT_EQ(r.Resolve(names, "ring"), (std::vector<int>{9}));
  // Sub-group via `<finger>_<seg>` prefix.
  EXPECT_EQ(r.Resolve(names, "thumb_mcp"), (std::vector<int>{2, 3}));
  // Unknown finger → empty (caller throws).
  EXPECT_TRUE(r.Resolve(names, "pinky").empty());
}

TEST(PrefixFingerResolver, NumericNamesMatchNothing) {
  // A prefix strategy cannot resolve fingers when joint names are numeric —
  // this is exactly the failure ExplicitFingerResolver exists to fix.
  const std::vector<std::string> names = {"0", "1", "2", "3", "4", "5", "6", "7"};
  PrefixFingerResolver r;
  EXPECT_TRUE(r.Resolve(names, "thumb").empty());
  EXPECT_TRUE(r.Resolve(names, "index").empty());
}

// ── ExplicitFingerResolver: config map, joint names ignored ────────────────

TEST(ExplicitFingerResolver, ResolvesFromMapIgnoringNames) {
  // LEAP-style: 16 numeric joints, finger layout supplied by config.
  const std::vector<std::string> numeric_names = {"0", "1", "2",  "3",  "4",  "5",  "6",  "7",
                                                  "8", "9", "10", "11", "12", "13", "14", "15"};
  ExplicitFingerResolver r({
      {"index", {0, 1, 2, 3}},
      {"middle", {4, 5, 6, 7}},
      {"ring", {8, 9, 10, 11}},
      {"thumb", {12, 13, 14, 15}},
  });

  EXPECT_EQ(r.Resolve(numeric_names, "index"), (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(r.Resolve(numeric_names, "thumb"), (std::vector<int>{12, 13, 14, 15}));
  // The map is authoritative — the same result regardless of the names passed.
  EXPECT_EQ(r.Resolve({}, "ring"), (std::vector<int>{8, 9, 10, 11}));
}

TEST(ExplicitFingerResolver, UnknownKeyIsEmpty) {
  ExplicitFingerResolver r({{"thumb", {0, 1}}});
  EXPECT_TRUE(r.Resolve({}, "index").empty());
  EXPECT_TRUE(r.Resolve({}, "").empty());
}

TEST(ExplicitFingerResolver, SubGroupKeysMustBeExplicit) {
  // Unlike the prefix strategy, sub-groups are not derived — they are listed.
  ExplicitFingerResolver r({{"thumb", {12, 13, 14, 15}}, {"thumb_mcp", {14, 15}}});
  EXPECT_EQ(r.Resolve({}, "thumb_mcp"), (std::vector<int>{14, 15}));
  // A sub-group that was not configured stays empty (no prefix fallback).
  EXPECT_TRUE(r.Resolve({}, "thumb_dip").empty());
}

// ── Polymorphic use through the abstract interface ─────────────────────────

TEST(FingerResolver, PolymorphicDispatch) {
  std::unique_ptr<FingerResolver> prefix = std::make_unique<PrefixFingerResolver>();
  std::unique_ptr<FingerResolver> explicit_r = std::make_unique<ExplicitFingerResolver>(
      std::map<std::string, std::vector<int>>{{"thumb", {0, 1, 2, 3}}});

  const std::vector<std::string> names = {"thumb_a", "thumb_b", "thumb_c", "thumb_d"};
  EXPECT_EQ(prefix->Resolve(names, "thumb"), (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(explicit_r->Resolve(names, "thumb"), (std::vector<int>{0, 1, 2, 3}));
}
