// rtc_controllers/params/yaml_read.hpp: each reader on a node it must refuse and on the
// same node made acceptable, so a reader that accepts everything (or nothing) is red.
#include "rtc_controllers/params/yaml_read.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>

namespace rtc::params {
namespace {

TEST(YamlRead, ReadSectionNodeTellsAbsentMapAndNotAMap) {
  const YAML::Node root = YAML::Load("a: {x: 1}\nb: 3\nc:\n");
  EXPECT_EQ(ReadSectionNode(root, "a").kind, SectionKind::kMap);
  EXPECT_EQ(ReadSectionNode(root, "a").node["x"].as<int>(), 1);
  EXPECT_EQ(ReadSectionNode(root, "b").kind, SectionKind::kNotAMap);
  EXPECT_EQ(ReadSectionNode(root, "c").kind, SectionKind::kAbsent);  // empty `c:`
  EXPECT_EQ(ReadSectionNode(root, "d").kind, SectionKind::kAbsent);
  EXPECT_EQ(ReadSectionNode(YAML::Load("3"), "a").kind, SectionKind::kAbsent);
}

TEST(YamlRead, FindUnknownKeyNamesTheFirstKeyOutsideTheSet) {
  const YAML::Node sec = YAML::Load("alpha: 1\nbeta: 2\n");
  EXPECT_FALSE(FindUnknownKey(sec, {"alpha", "beta"}).has_value());
  const auto unknown = FindUnknownKey(sec, {"alpha"});
  ASSERT_TRUE(unknown.has_value());
  EXPECT_EQ(*unknown, "beta");
}

TEST(YamlRead, FindMissingKeyNamesTheFirstRequiredKeyWithoutAValue) {
  const YAML::Node sec = YAML::Load("alpha: 1\nbeta:\n");
  EXPECT_FALSE(FindMissingKey(sec, {"alpha"}).has_value());
  const auto empty_value = FindMissingKey(sec, {"alpha", "beta"});
  ASSERT_TRUE(empty_value.has_value());
  EXPECT_EQ(*empty_value, "beta");  // `beta:` with no value is missing, not present
  const auto absent = FindMissingKey(sec, {"gamma", "alpha"});
  ASSERT_TRUE(absent.has_value());
  EXPECT_EQ(*absent, "gamma");
  const auto not_a_map = FindMissingKey(YAML::Load("[1, 2]"), {"alpha"});
  ASSERT_TRUE(not_a_map.has_value());
  EXPECT_EQ(*not_a_map, "alpha");
}

TEST(YamlRead, ReadFixedVectorReadsExactlyNFiniteNumbers) {
  const YAML::Node ok = YAML::Load("[1.5, -2, 3e-3]");
  const VectorRead read = ReadFixedVector(ok, 3);
  ASSERT_EQ(read.kind, VectorReadKind::kRead);
  EXPECT_DOUBLE_EQ(read.value[0], 1.5);
  EXPECT_DOUBLE_EQ(read.value[1], -2.0);
  EXPECT_DOUBLE_EQ(read.value[2], 3e-3);

  EXPECT_EQ(ReadFixedVector(YAML::Node(), 3).kind, VectorReadKind::kAbsent);
  EXPECT_EQ(ReadFixedVector(YAML::Load("1.0"), 3).kind, VectorReadKind::kNotASequence);
  EXPECT_EQ(ReadFixedVector(YAML::Load("{a: 1}"), 3).kind, VectorReadKind::kNotASequence);

  const VectorRead short_seq = ReadFixedVector(YAML::Load("[1, 2]"), 3);
  EXPECT_EQ(short_seq.kind, VectorReadKind::kWrongLength);
  EXPECT_EQ(short_seq.length, 2U);

  const VectorRead word = ReadFixedVector(YAML::Load("[1, x, 3]"), 3);
  EXPECT_EQ(word.kind, VectorReadKind::kNotANumber);
  EXPECT_EQ(word.index, 1U);

  const YAML::Node nan = YAML::Load("[1, 2, .nan]");
  const VectorRead non_finite = ReadFixedVector(nan, 3);
  EXPECT_EQ(non_finite.kind, VectorReadKind::kNonFinite);
  EXPECT_EQ(non_finite.index, 2U);
  // The same node passes when finiteness is not asked for.
  EXPECT_EQ(ReadFixedVector(nan, 3, /*require_finite=*/false).kind, VectorReadKind::kRead);
  EXPECT_EQ(ReadFixedVector(YAML::Load("[1, .inf, 3]"), 3).kind, VectorReadKind::kNonFinite);
}

TEST(YamlRead, ReadEnumIndexesTheClosedSet) {
  const auto second = ReadEnum(YAML::Load("beta"), {"alpha", "beta"});
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(*second, 1U);
  EXPECT_FALSE(ReadEnum(YAML::Load("Beta"), {"alpha", "beta"}).has_value());  // case matters
  EXPECT_FALSE(ReadEnum(YAML::Load("[beta]"), {"alpha", "beta"}).has_value());
  EXPECT_FALSE(ReadEnum(YAML::Node(), {"alpha", "beta"}).has_value());
}

TEST(YamlRead, SpellingQuotesWhatWasWritten) {
  EXPECT_EQ(Spelling(YAML::Load("1e-10")), "'1e-10'");
  EXPECT_EQ(Spelling(YAML::Load("[1]")), "a sequence");
  EXPECT_EQ(Spelling(YAML::Load("{a: 1}")), "a map");
  EXPECT_EQ(Spelling(YAML::Node()), "an empty value");
}

}  // namespace
}  // namespace rtc::params
