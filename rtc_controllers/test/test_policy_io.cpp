// ── Learned-policy I/O: marshalling core + YAML schema ───────────────────────
//
// The schema half is written as "this config is wrong and must be refused"
// rather than "the parser works", because the failure this layer exists to
// prevent is silent: a feature list whose order drifted from training still
// produces finite, in-limits commands. Every rejection case below is a drift
// that would otherwise reach the actuator boundary looking healthy.
//
// The headline case is ReorderingFeaturesReordersTheTensor — it drives the
// parser and the packer together, because "YAML order becomes tensor order" is
// a property of the pair, not of either one.

#include "rtc_controllers/inference/policy_io.hpp"
#include "rtc_controllers/params/policy_io_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace {

using rtc::inference::ApplyAffine;
using rtc::inference::BlendPosture;
using rtc::inference::InputSegment;
using rtc::inference::OutputSlice;
using rtc::inference::PackSegment;
using rtc::inference::UnpackSlice;
using rtc::params::ParsePolicyIoParams;
using rtc::params::PolicyIoParams;

/// Stand-in for the binding's robot-specific resolver. Sizes match the shipped
/// ur5e_p1b schema so the fixtures below read like the real thing, but the core
/// never sees this table — that is the ARCH-1 boundary being exercised.
int FeatureSize(std::string_view id) {
  if (id == "ur5e.position") {
    return 6;
  }
  if (id == "p1b.position") {
    return 10;
  }
  if (id == "palm.position") {
    return 3;
  }
  if (id == "palm.orientation_xyzw") {
    return 4;
  }
  if (id == "p1b.fingertip_force_norm") {
    return 4;
  }
  if (id == "object.position") {
    return 3;
  }
  if (id == "object.orientation_xyzw") {
    return 4;
  }
  return 0;  // unknown → the parser must refuse, not silently size it 0
}

/// The shipped 34-in / (6+1)-out schema, as YAML text so a case can mutate one
/// line and keep the rest honest.
constexpr const char* kBaseYaml = R"(
input_shape: [1, 34]
output_shapes: [[1, 6], [1, 1]]
input_features:
  - "ur5e.position"
  - "p1b.position"
  - "palm.position"
  - "palm.orientation_xyzw"
  - "p1b.fingertip_force_norm"
  - "object.position"
  - "object.orientation_xyzw"
output_features:
  - { name: "ur5e.target_position", head: 0, offset: 0, count: 6 }
  - { name: "p1b.posture_scalar",   head: 1, offset: 0, count: 1 }
decimation: 10
)";

PolicyIoParams ParseText(const std::string& yaml) {
  return ParsePolicyIoParams(YAML::Load(yaml), FeatureSize);
}

/// Assert that @p yaml is refused AND that it was refused for the stated
/// reason. Plain EXPECT_THROW is not enough here: every rejection in this
/// schema throws the same type, so a fixture that is broken in a second way
/// (a width that no longer adds up, a key that got mangled by the edit) would
/// still go green while never exercising the rule the case is named after.
///
/// `ADD_FAILURE` rather than `FAIL` because a gtest fatal inside a helper
/// returns from the HELPER only; the caller would carry on regardless.
void ExpectRejectMentioning(const std::string& yaml, std::string_view needle) {
  try {
    static_cast<void>(ParseText(yaml));
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(needle), std::string::npos)
        << "rejected, but for a different reason than this case tests.\n"
        << "  expected to mention: " << needle << "\n  actual: " << what;
    return;
  }
  ADD_FAILURE() << "expected the schema to be refused, but it parsed";
}

}  // namespace

// ── Schema: the happy path pins the numbers the binding depends on ──────────

TEST(PolicyIoParams, ShippedSchemaResolvesToContiguousSegments) {
  const auto p = ParseText(kBaseYaml);

  EXPECT_EQ(p.InputNumel(), 34U);
  EXPECT_EQ(p.decimation, 10);
  ASSERT_EQ(p.input_segments.size(), 7U);

  // Prefix sum over YAML order, no gaps, ending exactly at the tensor width.
  const std::array<int, 7> want_offset{0, 6, 16, 19, 23, 27, 30};
  const std::array<int, 7> want_count{6, 10, 3, 4, 4, 3, 4};
  for (std::size_t i = 0; i < want_offset.size(); ++i) {
    EXPECT_EQ(p.input_segments[i].offset, want_offset[i]) << "segment " << i;
    EXPECT_EQ(p.input_segments[i].count, want_count[i]) << "segment " << i;
  }

  ASSERT_EQ(p.output_slices.size(), 2U);
  EXPECT_EQ(p.output_slices[0].head, 0);
  EXPECT_EQ(p.output_slices[0].count, 6);
  EXPECT_EQ(p.output_slices[1].head, 1);
  EXPECT_EQ(p.output_slices[1].count, 1);

  // Both affine lanes omitted → identity, not a zero-filled lane.
  EXPECT_TRUE(p.input_offset.empty());
  EXPECT_TRUE(p.input_scale.empty());
}

// ── The contract: YAML order IS tensor order ────────────────────────────────

TEST(PolicyIoParams, ReorderingFeaturesReordersTheTensor) {
  // Same seven features, object pose moved to the FRONT. If the parser or the
  // packer ever stopped honouring list order, the two buffers below would come
  // out identical and a retrained policy would be fed a scrambled observation
  // that is elementwise finite and in range.
  const std::string reordered = R"(
input_shape: [1, 34]
output_shapes: [[1, 6], [1, 1]]
input_features:
  - "object.position"
  - "object.orientation_xyzw"
  - "ur5e.position"
  - "p1b.position"
  - "palm.position"
  - "palm.orientation_xyzw"
  - "p1b.fingertip_force_norm"
output_features:
  - { name: "ur5e.target_position", head: 0, offset: 0, count: 6 }
  - { name: "p1b.posture_scalar",   head: 1, offset: 0, count: 1 }
)";

  const auto base = ParseText(kBaseYaml);
  const auto moved = ParseText(reordered);

  // ur5e.position is feature 0 in the base schema and feature 2 in the other.
  EXPECT_EQ(base.input_segments[0].offset, 0);
  EXPECT_EQ(moved.input_segments[2].offset, 7);  // 3 (object pos) + 4 (object quat)

  // Pack the SAME arm values through each layout and observe them land in
  // different places.
  const std::vector<double> arm{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<float> buf_base(34, 0.0F);
  std::vector<float> buf_moved(34, 0.0F);
  ASSERT_TRUE(PackSegment(buf_base, base.input_segments[0], arm));
  ASSERT_TRUE(PackSegment(buf_moved, moved.input_segments[2], arm));

  EXPECT_FLOAT_EQ(buf_base[0], 1.0F);
  EXPECT_FLOAT_EQ(buf_moved[0], 0.0F);
  EXPECT_FLOAT_EQ(buf_moved[7], 1.0F);
  EXPECT_NE(buf_base, buf_moved);
}

// ── Schema rejections ───────────────────────────────────────────────────────

TEST(PolicyIoParams, RejectsUnknownFeatureId) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("\"palm.position\""), std::string("\"palm.position\"").size(),
               "\"palm.postion\"");  // typo, and it is 3 wide so the sum still works out
  ExpectRejectMentioning(yaml, "unknown id 'palm.postion'");
}

TEST(PolicyIoParams, RejectsFeatureSumMismatch) {
  // Declared width 33, features sum to 34 — the single most likely edit after a
  // retrain that added or dropped one observation.
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("[1, 34]"), std::string("[1, 34]").size(), "[1, 33]");
  ExpectRejectMentioning(yaml, "input_features sum to");
}

TEST(PolicyIoParams, RejectsDuplicateFeatureId) {
  const std::string yaml = R"(
input_shape: [1, 12]
output_shapes: [[1, 1]]
input_features: ["palm.position", "palm.position", "object.position", "object.position"]
output_features:
  - { name: "s", head: 0, offset: 0, count: 1 }
)";
  ExpectRejectMentioning(yaml, "repeats id");
}

TEST(PolicyIoParams, RejectsNonPositiveShapeDimension) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("[1, 34]"), std::string("[1, 34]").size(), "[0, 34]");
  ExpectRejectMentioning(yaml, "input_shape[0] must be > 0");
}

TEST(PolicyIoParams, RejectsOverlappingSlicesOnOneHead) {
  const std::string yaml = R"(
input_shape: [1, 6]
output_shapes: [[1, 6]]
input_features: ["ur5e.position"]
output_features:
  - { name: "a", head: 0, offset: 0, count: 4 }
  - { name: "b", head: 0, offset: 3, count: 3 }
)";
  ExpectRejectMentioning(yaml, "overlaps output_features[0]");
}

TEST(PolicyIoParams, AllowsSameOffsetOnDifferentHeads) {
  // The mirror of the case above: two heads both starting at 0 is the shipped
  // layout, so the overlap check must be per head and not global.
  const auto p = ParseText(kBaseYaml);
  EXPECT_EQ(p.output_slices[0].offset, 0);
  EXPECT_EQ(p.output_slices[1].offset, 0);
  EXPECT_NE(p.output_slices[0].head, p.output_slices[1].head);
}

TEST(PolicyIoParams, RejectsSlicePastHead) {
  const std::string yaml = R"(
input_shape: [1, 6]
output_shapes: [[1, 6]]
input_features: ["ur5e.position"]
output_features:
  - { name: "a", head: 0, offset: 4, count: 4 }
)";
  ExpectRejectMentioning(yaml, "past head 0");
}

TEST(PolicyIoParams, RejectsUnknownHeadIndex) {
  const std::string yaml = R"(
input_shape: [1, 6]
output_shapes: [[1, 6]]
input_features: ["ur5e.position"]
output_features:
  - { name: "a", head: 1, offset: 0, count: 6 }
)";
  ExpectRejectMentioning(yaml, "names head 1");
}

TEST(PolicyIoParams, RejectsDuplicateOutputName) {
  const std::string yaml = R"(
input_shape: [1, 6]
output_shapes: [[1, 6]]
input_features: ["ur5e.position"]
output_features:
  - { name: "a", head: 0, offset: 0, count: 3 }
  - { name: "a", head: 0, offset: 3, count: 3 }
)";
  ExpectRejectMentioning(yaml, "repeats name 'a'");
}

TEST(PolicyIoParams, RejectsPartialAffineLane) {
  std::string yaml = kBaseYaml;
  yaml += "input_scale: [1.0, 1.0, 1.0]\n";  // 3 of 34 — would normalise a prefix only
  ExpectRejectMentioning(yaml, "input_scale has 3 entries");
}

TEST(PolicyIoParams, AcceptsEmptyAffineLaneAsIdentity) {
  std::string yaml = kBaseYaml;
  yaml += "input_scale: []\ninput_offset: []\n";
  const auto p = ParseText(yaml);
  EXPECT_TRUE(p.input_scale.empty());
  EXPECT_TRUE(p.input_offset.empty());
}

TEST(PolicyIoParams, RejectsNonFiniteAffineValue) {
  std::string yaml = kBaseYaml;
  yaml += "input_scale: [";
  for (int i = 0; i < 34; ++i) {
    yaml += (i == 5) ? ".nan" : "1.0";
    if (i != 33) {
      yaml += ", ";
    }
  }
  yaml += "]\n";
  ExpectRejectMentioning(yaml, "input_scale[5] must be finite");
}

TEST(PolicyIoParams, RejectsDecimationBelowOne) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("decimation: 10"), std::string("decimation: 10").size(), "decimation: 0");
  ExpectRejectMentioning(yaml, "decimation must be >= 1");
}

TEST(PolicyIoParams, DecimationDefaultsToEveryTick) {
  const std::string yaml = R"(
input_shape: [1, 6]
output_shapes: [[1, 6]]
input_features: ["ur5e.position"]
output_features:
  - { name: "a", head: 0, offset: 0, count: 6 }
)";
  EXPECT_EQ(ParseText(yaml).decimation, 1);
}

TEST(PolicyIoParams, RejectsMissingResolver) {
  EXPECT_THROW(
      static_cast<void>(ParsePolicyIoParams(YAML::Load(kBaseYaml), rtc::params::FeatureSizeFn{})),
      std::invalid_argument);
}

// ── PackSegment: all-or-nothing ─────────────────────────────────────────────

TEST(PolicyIoCore, PackSegmentWritesNothingWhenItWouldOverrun) {
  std::vector<float> buf(4, -1.0F);
  const std::vector<double> src{1.0, 2.0, 3.0};
  EXPECT_FALSE(PackSegment(buf, InputSegment{2, 3}, src));
  for (const float v : buf) {
    EXPECT_FLOAT_EQ(v, -1.0F) << "a rejected segment must leave the buffer untouched";
  }
}

TEST(PolicyIoCore, PackSegmentWritesNothingWhenSourceIsShort) {
  std::vector<float> buf(8, -1.0F);
  const std::vector<double> src{1.0, 2.0};
  EXPECT_FALSE(PackSegment(buf, InputSegment{0, 6}, src));
  EXPECT_FLOAT_EQ(buf[0], -1.0F);
}

TEST(PolicyIoCore, PackSegmentRejectsNegativeDescriptor) {
  std::vector<float> buf(4, 0.0F);
  const std::vector<double> src{1.0};
  EXPECT_FALSE(PackSegment(buf, InputSegment{-1, 1}, src));
  EXPECT_FALSE(PackSegment(buf, InputSegment{0, -1}, src));
}

// ── ApplyAffine ─────────────────────────────────────────────────────────────

TEST(PolicyIoCore, ApplyAffineIsIdentityWhenBothLanesEmpty) {
  std::vector<float> buf{1.0F, 2.0F, 3.0F};
  ApplyAffine(buf, {}, {});
  EXPECT_FLOAT_EQ(buf[0], 1.0F);
  EXPECT_FLOAT_EQ(buf[2], 3.0F);
}

TEST(PolicyIoCore, ApplyAffineSubtractsThenScales) {
  std::vector<float> buf{10.0F, 20.0F};
  const std::vector<float> offset{1.0F, 2.0F};
  const std::vector<float> scale{2.0F, 0.5F};
  ApplyAffine(buf, offset, scale);
  EXPECT_FLOAT_EQ(buf[0], 18.0F);  // (10 - 1) * 2
  EXPECT_FLOAT_EQ(buf[1], 9.0F);   // (20 - 2) * 0.5
}

TEST(PolicyIoCore, ApplyAffineLeavesNanAlone) {
  // Scrubbing here would hide a bad observation behind a plausible number; the
  // caller's validation is what owns the NaN.
  std::vector<float> buf{std::numeric_limits<float>::quiet_NaN()};
  const std::vector<float> offset{1.0F};
  const std::vector<float> scale{2.0F};
  ApplyAffine(buf, offset, scale);
  EXPECT_TRUE(std::isnan(buf[0]));
}

// ── UnpackSlice ─────────────────────────────────────────────────────────────

TEST(PolicyIoCore, UnpackSliceCopiesTheRequestedRun) {
  const std::array<float, 4> head{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<double> out(2, 0.0);
  ASSERT_TRUE(UnpackSlice(out, OutputSlice{0, 1, 2}, head.data(), head.size()));
  EXPECT_DOUBLE_EQ(out[0], 2.0);
  EXPECT_DOUBLE_EQ(out[1], 3.0);
}

TEST(PolicyIoCore, UnpackSliceRefusesNullHeadBuffer) {
  // Exactly what the stub inference engine hands back when ONNX Runtime is
  // absent — the caller turns this into a hold rather than dereferencing it.
  std::vector<double> out(2, -1.0);
  EXPECT_FALSE(UnpackSlice(out, OutputSlice{0, 0, 2}, nullptr, 4));
  EXPECT_DOUBLE_EQ(out[0], -1.0);
}

TEST(PolicyIoCore, UnpackSliceRefusesRunPastHead) {
  const std::array<float, 4> head{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<double> out(4, -1.0);
  EXPECT_FALSE(UnpackSlice(out, OutputSlice{0, 3, 2}, head.data(), head.size()));
  EXPECT_DOUBLE_EQ(out[0], -1.0);
}

TEST(PolicyIoCore, UnpackSliceRefusesShortDestination) {
  const std::array<float, 4> head{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<double> out(1, -1.0);
  EXPECT_FALSE(UnpackSlice(out, OutputSlice{0, 0, 4}, head.data(), head.size()));
}

// ── BlendPosture ────────────────────────────────────────────────────────────

TEST(PolicyIoCore, BlendPostureHitsBothEndpoints) {
  const std::vector<double> open{0.0, 0.0, 0.0};
  const std::vector<double> close{-1.0, 0.5, -0.25};
  std::vector<double> out(3, 99.0);

  auto rep = BlendPosture(out, open, close, 0.0);
  EXPECT_TRUE(rep.valid);
  EXPECT_FALSE(rep.clamped);
  EXPECT_DOUBLE_EQ(out[0], 0.0);

  rep = BlendPosture(out, open, close, 1.0);
  EXPECT_TRUE(rep.valid);
  EXPECT_FALSE(rep.clamped);
  EXPECT_DOUBLE_EQ(out[0], -1.0);
  EXPECT_DOUBLE_EQ(out[1], 0.5);
}

TEST(PolicyIoCore, BlendPostureInterpolatesMidpoint) {
  const std::vector<double> open{0.0, 2.0};
  const std::vector<double> close{-1.0, 0.0};
  std::vector<double> out(2, 0.0);
  const auto rep = BlendPosture(out, open, close, 0.5);
  ASSERT_TRUE(rep.valid);
  EXPECT_DOUBLE_EQ(out[0], -0.5);
  EXPECT_DOUBLE_EQ(out[1], 1.0);
}

TEST(PolicyIoCore, BlendPostureClampsOutOfRangeScalarAndSaysSo) {
  const std::vector<double> open{0.0};
  const std::vector<double> close{-1.0};
  std::vector<double> out(1, 0.0);

  auto rep = BlendPosture(out, open, close, 1.7);
  EXPECT_TRUE(rep.valid);
  EXPECT_TRUE(rep.clamped) << "an out-of-range scalar is a normalisation-mismatch signal";
  EXPECT_DOUBLE_EQ(out[0], -1.0);

  rep = BlendPosture(out, open, close, -0.3);
  EXPECT_TRUE(rep.valid);
  EXPECT_TRUE(rep.clamped);
  EXPECT_DOUBLE_EQ(out[0], 0.0);
}

TEST(PolicyIoCore, BlendPostureRefusesNonFiniteScalar) {
  // The whole reason the clamp is hand-rolled: every comparison against NaN is
  // false, so a clamp would pass it into all ten joints wearing a valid shape.
  const std::vector<double> open{0.0, 0.0};
  const std::vector<double> close{-1.0, -1.0};
  std::vector<double> out(2, 42.0);

  for (const double bad :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    const auto rep = BlendPosture(out, open, close, bad);
    EXPECT_FALSE(rep.valid);
    EXPECT_DOUBLE_EQ(out[0], 42.0) << "a refused blend must not write";
  }
}

TEST(PolicyIoCore, BlendPostureRefusesRaggedPostures) {
  const std::vector<double> open{0.0, 0.0, 0.0};
  const std::vector<double> close{-1.0};  // shorter than `out` — a config that got past nothing
  std::vector<double> out(3, 7.0);
  const auto rep = BlendPosture(out, open, close, 0.5);
  EXPECT_FALSE(rep.valid);
  EXPECT_DOUBLE_EQ(out[0], 7.0);
}
