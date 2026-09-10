// ── Learned-policy I/O: marshalling core + YAML schema ───────────────────────
//
// The schema half is written as "this config is wrong and must be refused"
// rather than "the parser works", because the failure this layer exists to
// prevent is silent: a feature list whose order drifted from training still
// produces finite, in-limits commands. Every rejection case below is a drift
// that would otherwise reach the actuator boundary looking healthy.
//
// Two headline cases carry the #511 P3 contract, and both drive the parser and
// the packer TOGETHER because each is a property of the pair:
//   - ReorderingFeaturesReordersTheTensor — YAML order becomes tensor order
//   - EachTensorGetsItsOwnOffsetSpace — a feature's offset is meaningless
//     without the tensor it belongs to, so the second tensor restarts at 0
//     rather than continuing a policy-wide prefix sum

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

/// The shipped single-tensor 34-in / (6+1)-out schema, as YAML text so a case
/// can mutate one line and keep the rest honest.
constexpr const char* kBaseYaml = R"(
inputs:
  - name: "obs"
    shape: [1, 34]
    features:
      - "ur5e.position"
      - "p1b.position"
      - "palm.position"
      - "palm.orientation_xyzw"
      - "p1b.fingertip_force_norm"
      - "object.position"
      - "object.orientation_xyzw"
outputs:
  - name: "arm_action"
    shape: [1, 6]
  - name: "posture"
    shape: [1, 1]
output_features:
  - { name: "ur5e.target_position", tensor: "arm_action" }
  - { name: "p1b.posture_scalar",   tensor: "posture" }
decimation: 10
)";

/// Two input tensors, the shape #511 exists for. Widths are chosen so the two
/// feature sums differ (16 vs 7): a parser that kept one policy-wide prefix sum
/// would still satisfy the total and only the per-tensor offsets would betray
/// it.
constexpr const char* kTwoTensorYaml = R"(
inputs:
  - name: "obs"
    shape: [1, 16]
    features: ["ur5e.position", "p1b.position"]
  - name: "aux"
    shape: [1, 7]
    features: ["palm.position", "palm.orientation_xyzw"]
outputs:
  - name: "arm_action"
    shape: [1, 6]
output_features:
  - { name: "ur5e.target_position", tensor: "arm_action" }
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

  EXPECT_EQ(p.decimation, 10);
  ASSERT_EQ(p.inputs.size(), 1U);
  EXPECT_EQ(p.inputs[0].name, "obs");
  EXPECT_EQ(p.inputs[0].Numel(), 34U);
  ASSERT_EQ(p.inputs[0].segments.size(), 7U);

  // Prefix sum over YAML order, no gaps, ending exactly at the tensor width.
  const std::array<int, 7> want_offset{0, 6, 16, 19, 23, 27, 30};
  const std::array<int, 7> want_count{6, 10, 3, 4, 4, 3, 4};
  for (std::size_t i = 0; i < want_offset.size(); ++i) {
    EXPECT_EQ(p.inputs[0].segments[i].tensor, 0) << "segment " << i;
    EXPECT_EQ(p.inputs[0].segments[i].offset, want_offset[i]) << "segment " << i;
    EXPECT_EQ(p.inputs[0].segments[i].count, want_count[i]) << "segment " << i;
  }

  ASSERT_EQ(p.outputs.size(), 2U);
  EXPECT_EQ(p.outputs[0].name, "arm_action");
  EXPECT_EQ(p.outputs[1].Numel(), 1U);

  ASSERT_EQ(p.output_slices.size(), 2U);
  EXPECT_EQ(p.output_slices[0].tensor, 0);
  EXPECT_EQ(p.output_slices[0].count, 6);
  EXPECT_EQ(p.output_slices[1].tensor, 1);
  EXPECT_EQ(p.output_slices[1].count, 1);

  // Both affine lanes omitted → identity, not a zero-filled lane.
  EXPECT_TRUE(p.inputs[0].offset.empty());
  EXPECT_TRUE(p.inputs[0].scale.empty());

  // The slot #511 P5 fills. Empty here is the contract, not an accident.
  EXPECT_TRUE(p.recurrent_links.empty());
}

// ── The contract: YAML order IS tensor order ────────────────────────────────

TEST(PolicyIoParams, ReorderingFeaturesReordersTheTensor) {
  // Same seven features, object pose moved to the FRONT. If the parser or the
  // packer ever stopped honouring list order, the two buffers below would come
  // out identical and a retrained policy would be fed a scrambled observation
  // that is elementwise finite and in range.
  const std::string reordered = R"(
inputs:
  - name: "obs"
    shape: [1, 34]
    features:
      - "object.position"
      - "object.orientation_xyzw"
      - "ur5e.position"
      - "p1b.position"
      - "palm.position"
      - "palm.orientation_xyzw"
      - "p1b.fingertip_force_norm"
outputs:
  - name: "arm_action"
    shape: [1, 6]
  - name: "posture"
    shape: [1, 1]
output_features:
  - { name: "ur5e.target_position", tensor: "arm_action" }
  - { name: "p1b.posture_scalar",   tensor: "posture" }
)";

  const auto base = ParseText(kBaseYaml);
  const auto moved = ParseText(reordered);

  // ur5e.position is feature 0 in the base schema and feature 2 in the other.
  EXPECT_EQ(base.inputs[0].segments[0].offset, 0);
  EXPECT_EQ(moved.inputs[0].segments[2].offset, 7);  // 3 (object pos) + 4 (object quat)

  // Pack the SAME arm values through each layout and observe them land in
  // different places.
  const std::vector<double> arm{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<float> buf_base(34, 0.0F);
  std::vector<float> buf_moved(34, 0.0F);
  ASSERT_TRUE(PackSegment(buf_base, base.inputs[0].segments[0], arm));
  ASSERT_TRUE(PackSegment(buf_moved, moved.inputs[0].segments[2], arm));

  EXPECT_FLOAT_EQ(buf_base[0], 1.0F);
  EXPECT_FLOAT_EQ(buf_moved[0], 0.0F);
  EXPECT_FLOAT_EQ(buf_moved[7], 1.0F);
  EXPECT_NE(buf_base, buf_moved);
}

// ── The #511 P3 contract: one address space PER tensor ──────────────────────

TEST(PolicyIoParams, EachTensorGetsItsOwnOffsetSpace) {
  const auto p = ParseText(kTwoTensorYaml);
  ASSERT_EQ(p.inputs.size(), 2U);
  EXPECT_EQ(p.inputs[0].name, "obs");
  EXPECT_EQ(p.inputs[1].name, "aux");

  ASSERT_EQ(p.inputs[0].segments.size(), 2U);
  EXPECT_EQ(p.inputs[0].segments[0].tensor, 0);
  EXPECT_EQ(p.inputs[0].segments[0].offset, 0);
  EXPECT_EQ(p.inputs[0].segments[1].offset, 6);

  // The whole point: the second tensor RESTARTS at 0. A policy-wide prefix sum
  // would have put palm.position at 16 — an offset that is inside neither
  // tensor's meaning and, on a 34-wide buffer, still a perfectly valid write.
  ASSERT_EQ(p.inputs[1].segments.size(), 2U);
  EXPECT_EQ(p.inputs[1].segments[0].tensor, 1);
  EXPECT_EQ(p.inputs[1].segments[0].offset, 0);
  EXPECT_EQ(p.inputs[1].segments[1].tensor, 1);
  EXPECT_EQ(p.inputs[1].segments[1].offset, 3);

  // Drive it through the packer against a buffer sized for the SECOND tensor
  // only, which is what the engine hands back for `input_buffer(0, 1)`. A
  // segment that had kept the flat offset would be refused by PackSegment
  // rather than silently landing elsewhere — but it would be refused for every
  // tick, i.e. a permanent hold, so the offset is what has to be right.
  const std::vector<double> palm{0.1, 0.2, 0.3};
  std::vector<float> aux(7, -1.0F);
  ASSERT_TRUE(PackSegment(aux, p.inputs[1].segments[0], palm));
  EXPECT_FLOAT_EQ(aux[0], 0.1F);
  EXPECT_FLOAT_EQ(aux[2], 0.3F);
  EXPECT_FLOAT_EQ(aux[3], -1.0F) << "the quaternion segment starts here and was not written";
}

TEST(PolicyIoParams, AffineLanesAreSizedPerTensor) {
  // A lane sized for the FIRST tensor attached to the second. Under the old
  // flat schema there was one lane for one buffer and this length was correct;
  // now it normalises a tensor it does not describe, so it must be refused.
  std::string yaml = kTwoTensorYaml;
  yaml.replace(yaml.find("    features: [\"palm.position\""),
               std::string("    features: [\"palm.position\"").size(),
               "    scale: [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]\n    features: [\"palm.position\"");
  ExpectRejectMentioning(yaml, "inputs[1].scale has 16 entries but the tensor has 7");
}

TEST(PolicyIoParams, AffineLaneAppliesToItsOwnTensorOnly) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
    offset: [1, 1, 1, 1, 1, 1]
  - name: "aux"
    shape: [1, 3]
    features: ["palm.position"]
outputs:
  - name: "arm_action"
    shape: [1, 6]
output_features:
  - { name: "ur5e.target_position", tensor: "arm_action" }
)";
  const auto p = ParseText(yaml);
  ASSERT_EQ(p.inputs.size(), 2U);
  EXPECT_EQ(p.inputs[0].offset.size(), 6U);
  EXPECT_TRUE(p.inputs[1].offset.empty()) << "an untouched tensor must stay identity";

  std::vector<float> aux{5.0F, 5.0F, 5.0F};
  ApplyAffine(aux, p.inputs[1].offset, p.inputs[1].scale);
  EXPECT_FLOAT_EQ(aux[0], 5.0F);
}

// ── Tensor names are mandatory (#511 D-1) ───────────────────────────────────

TEST(PolicyIoParams, RejectsUnnamedInputTensor) {
  const std::string yaml = R"(
inputs:
  - shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "a"
    shape: [1, 6]
output_features:
  - { name: "arm", tensor: "a" }
)";
  ExpectRejectMentioning(yaml, "inputs[0] is missing 'name'");
}

TEST(PolicyIoParams, RejectsUnnamedOutputTensor) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - shape: [1, 6]
output_features:
  - { name: "arm", tensor: "a" }
)";
  ExpectRejectMentioning(yaml, "outputs[0] is missing 'name'");
}

TEST(PolicyIoParams, RejectsDuplicateTensorName) {
  // Two inputs called "obs" bind the same .onnx tensor twice, leaving a real
  // model input unwritten — and the engine's own arity check would still pass.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
  - name: "obs"
    shape: [1, 3]
    features: ["palm.position"]
outputs:
  - name: "a"
    shape: [1, 6]
output_features:
  - { name: "arm", tensor: "a" }
)";
  ExpectRejectMentioning(yaml, "inputs[1] repeats tensor name 'obs'");
}

// ── Output slices refer to a tensor by NAME ─────────────────────────────────

TEST(PolicyIoParams, RejectsSliceOfUndeclaredTensor) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("tensor: \"posture\""), std::string("tensor: \"posture\"").size(),
               "tensor: \"postrue\"");
  ExpectRejectMentioning(yaml, "names output tensor 'postrue'");
}

TEST(PolicyIoParams, RejectsSliceWithoutTensorReference) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "a"
    shape: [1, 6]
output_features:
  - { name: "arm", offset: 0, count: 6 }
)";
  ExpectRejectMentioning(yaml, "must name the output tensor it slices");
}

TEST(PolicyIoParams, SliceOrderIsIndependentOfTensorOrder) {
  // The two lists are deliberately crossed: `posture` is sliced first while
  // `outputs:` still declares `arm_action` first. Under a positional reference
  // the arm command would read the 1-element posture tensor and the posture
  // would read six — a resolution that is wrong without being out of bounds
  // once the two widths happen to match.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "arm_action"
    shape: [1, 6]
  - name: "posture"
    shape: [1, 1]
output_features:
  - { name: "p1b.posture_scalar",   tensor: "posture" }
  - { name: "ur5e.target_position", tensor: "arm_action" }
)";
  const auto p = ParseText(yaml);
  ASSERT_EQ(p.output_slices.size(), 2U);
  EXPECT_EQ(p.output_slices[0].tensor, 1) << "posture is output tensor 1, not slice 0's position";
  EXPECT_EQ(p.output_slices[0].count, 1);
  EXPECT_EQ(p.output_slices[1].tensor, 0);
  EXPECT_EQ(p.output_slices[1].count, 6);
}

TEST(PolicyIoParams, SliceDefaultsToTheWholeTensor) {
  // Omitting offset/count is the common case (one tensor IS one command).
  // Spelling them out would be a second place for the width to drift from the
  // shape declared right above it.
  const auto p = ParseText(kBaseYaml);
  EXPECT_EQ(p.output_slices[0].offset, 0);
  EXPECT_EQ(p.output_slices[0].count, 6);
  EXPECT_EQ(p.output_slices[1].count, 1);
}

TEST(PolicyIoParams, SliceHonoursAnExplicitOffsetAndCount) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 8]
output_features:
  - { name: "arm",  tensor: "action", offset: 0, count: 6 }
  - { name: "grip", tensor: "action", offset: 6, count: 2 }
)";
  const auto p = ParseText(yaml);
  ASSERT_EQ(p.output_slices.size(), 2U);
  EXPECT_EQ(p.output_slices[1].tensor, 0);
  EXPECT_EQ(p.output_slices[1].offset, 6);
  EXPECT_EQ(p.output_slices[1].count, 2);
}

TEST(PolicyIoParams, SliceWithOffsetOnlyRunsToTheEnd) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 8]
output_features:
  - { name: "grip", tensor: "action", offset: 6 }
)";
  const auto p = ParseText(yaml);
  EXPECT_EQ(p.output_slices[0].offset, 6);
  EXPECT_EQ(p.output_slices[0].count, 2);
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
  ExpectRejectMentioning(yaml, "features sum to 34 elements but its shape declares 33");
}

TEST(PolicyIoParams, RejectsDuplicateFeatureIdWithinOneTensor) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 12]
    features: ["palm.position", "palm.position", "object.position", "object.position"]
outputs:
  - name: "a"
    shape: [1, 1]
output_features:
  - { name: "s", tensor: "a" }
)";
  ExpectRejectMentioning(yaml, "repeats id 'palm.position'");
}

TEST(PolicyIoParams, RejectsDuplicateFeatureIdAcrossTensors) {
  // #511 D-6. The message names BOTH places: which of the two copies is the
  // mistake is not knowable from here, and an operator handed only the second
  // one will delete the wrong line half the time.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
  - name: "aux"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "a"
    shape: [1, 6]
output_features:
  - { name: "arm", tensor: "a" }
)";
  ExpectRejectMentioning(yaml,
                         "inputs[1].features[0] repeats id 'ur5e.position', already "
                         "declared at inputs[0].features[0]");
}

TEST(PolicyIoParams, RejectsNonPositiveShapeDimension) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("[1, 34]"), std::string("[1, 34]").size(), "[0, 34]");
  ExpectRejectMentioning(yaml, "inputs[0].shape[0] must be > 0");
}

TEST(PolicyIoParams, RejectsOverlappingSlicesOnOneTensor) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { name: "a", tensor: "action", offset: 0, count: 4 }
  - { name: "b", tensor: "action", offset: 3, count: 3 }
)";
  ExpectRejectMentioning(yaml, "overlaps output_features[0]");
}

TEST(PolicyIoParams, AllowsSameOffsetOnDifferentTensors) {
  // The mirror of the case above: two tensors both starting at 0 is the shipped
  // layout, so the overlap check must be per tensor and not global.
  const auto p = ParseText(kBaseYaml);
  EXPECT_EQ(p.output_slices[0].offset, 0);
  EXPECT_EQ(p.output_slices[1].offset, 0);
  EXPECT_NE(p.output_slices[0].tensor, p.output_slices[1].tensor);
}

TEST(PolicyIoParams, RejectsSlicePastTensor) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { name: "a", tensor: "action", offset: 4, count: 4 }
)";
  ExpectRejectMentioning(yaml, "past tensor 'action's 6 elements");
}

TEST(PolicyIoParams, RejectsDuplicateOutputName) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { name: "a", tensor: "action", offset: 0, count: 3 }
  - { name: "a", tensor: "action", offset: 3, count: 3 }
)";
  ExpectRejectMentioning(yaml, "repeats name 'a'");
}

TEST(PolicyIoParams, RejectsPartialAffineLane) {
  std::string yaml = kBaseYaml;
  // 3 of 34 — would normalise a prefix only.
  yaml.replace(yaml.find("    features:"), 0, "    scale: [1.0, 1.0, 1.0]\n");
  ExpectRejectMentioning(yaml, "inputs[0].scale has 3 entries");
}

TEST(PolicyIoParams, AcceptsEmptyAffineLaneAsIdentity) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("    features:"), 0, "    scale: []\n    offset: []\n");
  const auto p = ParseText(yaml);
  EXPECT_TRUE(p.inputs[0].scale.empty());
  EXPECT_TRUE(p.inputs[0].offset.empty());
}

TEST(PolicyIoParams, RejectsNonFiniteAffineValue) {
  std::string lane = "    scale: [";
  for (int i = 0; i < 34; ++i) {
    lane += (i == 5) ? ".nan" : "1.0";
    if (i != 33) {
      lane += ", ";
    }
  }
  lane += "]\n";
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("    features:"), 0, lane);
  ExpectRejectMentioning(yaml, "inputs[0].scale[5] must be finite");
}

TEST(PolicyIoParams, RejectsDecimationBelowOne) {
  std::string yaml = kBaseYaml;
  yaml.replace(yaml.find("decimation: 10"), std::string("decimation: 10").size(), "decimation: 0");
  ExpectRejectMentioning(yaml, "decimation must be >= 1");
}

TEST(PolicyIoParams, DecimationDefaultsToEveryTick) {
  EXPECT_EQ(ParseText(kTwoTensorYaml).decimation, 1);
}

TEST(PolicyIoParams, RejectsMissingResolver) {
  EXPECT_THROW(
      static_cast<void>(ParsePolicyIoParams(YAML::Load(kBaseYaml), rtc::params::FeatureSizeFn{})),
      std::invalid_argument);
}

// ── Reserved for #511 P5: refused, not ignored ──────────────────────────────

TEST(PolicyIoParams, RejectsRecurrentInputSourceUntilP5) {
  // Accepting this key while the feedback path does not exist would leave the
  // policy reading a tensor nobody writes — every action finite and plausible.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
  - name: "h_in"
    shape: [1, 1, 4]
    source: recurrent
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { name: "arm", tensor: "action" }
)";
  ExpectRejectMentioning(yaml, "inputs[1] declares 'source'");
}

TEST(PolicyIoParams, RejectsRecurrentOutputFeedsUntilP5) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
  - name: "h_out"
    shape: [1, 1, 4]
    feeds: "h_in"
output_features:
  - { name: "arm", tensor: "action" }
)";
  ExpectRejectMentioning(yaml, "outputs[1] declares 'feeds'");
}

// ── The pre-#511 schema is named, not left to fail obscurely ────────────────

TEST(PolicyIoParams, RejectsLegacyFlatSchemaByName) {
  // A config left on the old format would otherwise fail as "inputs must be a
  // non-empty sequence", which is true and tells the operator nothing about
  // what changed.
  const std::string yaml = R"(
input_shape: [1, 6]
output_shapes: [[1, 6]]
input_features: ["ur5e.position"]
output_features:
  - { name: "a", head: 0, offset: 0, count: 6 }
)";
  ExpectRejectMentioning(yaml, "'input_shape' is the pre-#511 flat schema");
}

// ── PackSegment: all-or-nothing ─────────────────────────────────────────────

TEST(PolicyIoCore, PackSegmentWritesNothingWhenItWouldOverrun) {
  std::vector<float> buf(4, -1.0F);
  const std::vector<double> src{1.0, 2.0, 3.0};
  EXPECT_FALSE(PackSegment(buf, InputSegment{0, 2, 3}, src));
  for (const float v : buf) {
    EXPECT_FLOAT_EQ(v, -1.0F) << "a rejected segment must leave the buffer untouched";
  }
}

TEST(PolicyIoCore, PackSegmentWritesNothingWhenSourceIsShort) {
  std::vector<float> buf(8, -1.0F);
  const std::vector<double> src{1.0, 2.0};
  EXPECT_FALSE(PackSegment(buf, InputSegment{0, 0, 6}, src));
  EXPECT_FLOAT_EQ(buf[0], -1.0F);
}

TEST(PolicyIoCore, PackSegmentRejectsNegativeDescriptor) {
  std::vector<float> buf(4, 0.0F);
  const std::vector<double> src{1.0};
  EXPECT_FALSE(PackSegment(buf, InputSegment{0, -1, 1}, src));
  EXPECT_FALSE(PackSegment(buf, InputSegment{0, 0, -1}, src));
}

TEST(PolicyIoCore, PackSegmentIgnoresTheTensorIndex) {
  // The core is handed ONE tensor's buffer, so `tensor` is routing information
  // for the caller and must not change what a write does. Pinning it here is
  // what stops a future caller from reading `tensor` as a second offset.
  std::vector<float> a(4, 0.0F);
  std::vector<float> b(4, 0.0F);
  const std::vector<double> src{7.0, 8.0};
  ASSERT_TRUE(PackSegment(a, InputSegment{0, 1, 2}, src));
  ASSERT_TRUE(PackSegment(b, InputSegment{3, 1, 2}, src));
  EXPECT_EQ(a, b);
  EXPECT_FLOAT_EQ(a[1], 7.0F);
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
  const std::array<float, 4> tensor{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<double> out(2, 0.0);
  ASSERT_TRUE(UnpackSlice(out, OutputSlice{0, 1, 2}, tensor.data(), tensor.size()));
  EXPECT_DOUBLE_EQ(out[0], 2.0);
  EXPECT_DOUBLE_EQ(out[1], 3.0);
}

TEST(PolicyIoCore, UnpackSliceRefusesNullBuffer) {
  // Exactly what the stub inference engine hands back when ONNX Runtime is
  // absent — the caller turns this into a hold rather than dereferencing it.
  std::vector<double> out(2, -1.0);
  EXPECT_FALSE(UnpackSlice(out, OutputSlice{0, 0, 2}, nullptr, 4));
  EXPECT_DOUBLE_EQ(out[0], -1.0);
}

TEST(PolicyIoCore, UnpackSliceRefusesRunPastTensor) {
  const std::array<float, 4> tensor{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<double> out(4, -1.0);
  EXPECT_FALSE(UnpackSlice(out, OutputSlice{0, 3, 2}, tensor.data(), tensor.size()));
  EXPECT_DOUBLE_EQ(out[0], -1.0);
}

TEST(PolicyIoCore, UnpackSliceRefusesShortDestination) {
  const std::array<float, 4> tensor{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<double> out(1, -1.0);
  EXPECT_FALSE(UnpackSlice(out, OutputSlice{0, 0, 4}, tensor.data(), tensor.size()));
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
