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
using rtc::inference::CopyFiniteChecked;
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
  - { tensor: "arm_action", role: "joint_target",   device: "ur5e" }
  - { tensor: "posture",    role: "posture_scalar", device: "p1b" }
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
  - { tensor: "arm_action", role: "joint_target",   device: "ur5e" }
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

  ASSERT_EQ(p.output_features.size(), 2U);
  EXPECT_EQ(p.output_features[0].slice.tensor, 0);
  EXPECT_EQ(p.output_features[0].slice.count, 6);
  EXPECT_EQ(p.output_features[1].slice.tensor, 1);
  EXPECT_EQ(p.output_features[1].slice.count, 1);

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
  - { tensor: "arm_action", role: "joint_target",   device: "ur5e" }
  - { tensor: "posture",    role: "posture_scalar", device: "p1b" }
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
  - { tensor: "arm_action", role: "joint_target",   device: "ur5e" }
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
  - { tensor: "a", role: "joint_target", device: "arm" }
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
  - { tensor: "a", role: "joint_target", device: "arm" }
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
  - { tensor: "a", role: "joint_target", device: "arm" }
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
  - { role: "joint_target", device: "arm", offset: 0, count: 6 }
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
  - { tensor: "posture",    role: "posture_scalar", device: "p1b" }
  - { tensor: "arm_action", role: "joint_target",   device: "ur5e" }
)";
  const auto p = ParseText(yaml);
  ASSERT_EQ(p.output_features.size(), 2U);
  EXPECT_EQ(p.output_features[0].slice.tensor, 1)
      << "posture is output tensor 1, not slice 0's position";
  EXPECT_EQ(p.output_features[0].slice.count, 1);
  EXPECT_EQ(p.output_features[1].slice.tensor, 0);
  EXPECT_EQ(p.output_features[1].slice.count, 6);
}

TEST(PolicyIoParams, SliceDefaultsToTheWholeTensor) {
  // Omitting offset/count is the common case (one tensor IS one command).
  // Spelling them out would be a second place for the width to drift from the
  // shape declared right above it.
  const auto p = ParseText(kBaseYaml);
  EXPECT_EQ(p.output_features[0].slice.offset, 0);
  EXPECT_EQ(p.output_features[0].slice.count, 6);
  EXPECT_EQ(p.output_features[1].slice.count, 1);
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
  - { tensor: "action", role: "joint_target",   device: "arm",  offset: 0, count: 6 }
  - { tensor: "action", role: "posture_scalar", device: "hand", offset: 6, count: 2 }
)";
  const auto p = ParseText(yaml);
  ASSERT_EQ(p.output_features.size(), 2U);
  EXPECT_EQ(p.output_features[1].slice.tensor, 0);
  EXPECT_EQ(p.output_features[1].slice.offset, 6);
  EXPECT_EQ(p.output_features[1].slice.count, 2);
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
  - { tensor: "action", role: "posture_scalar", device: "hand", offset: 6 }
)";
  const auto p = ParseText(yaml);
  EXPECT_EQ(p.output_features[0].slice.offset, 6);
  EXPECT_EQ(p.output_features[0].slice.count, 2);
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
  - { tensor: "a", role: "posture_scalar", device: "hand" }
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
  - { tensor: "a", role: "joint_target", device: "arm" }
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
  - { tensor: "action", role: "joint_target",   device: "arm",  offset: 0, count: 4 }
  - { tensor: "action", role: "posture_scalar", device: "hand", offset: 3, count: 3 }
)";
  ExpectRejectMentioning(yaml, "overlaps output_features[0]");
}

TEST(PolicyIoParams, AllowsSameOffsetOnDifferentTensors) {
  // The mirror of the case above: two tensors both starting at 0 is the shipped
  // layout, so the overlap check must be per tensor and not global.
  const auto p = ParseText(kBaseYaml);
  EXPECT_EQ(p.output_features[0].slice.offset, 0);
  EXPECT_EQ(p.output_features[1].slice.offset, 0);
  EXPECT_NE(p.output_features[0].slice.tensor, p.output_features[1].slice.tensor);
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
  - { tensor: "action", role: "joint_target", device: "arm", offset: 4, count: 4 }
)";
  ExpectRejectMentioning(yaml, "past tensor 'action's 6 elements");
}

TEST(PolicyIoParams, RejectsAMalformedOffsetInsteadOfReadingItAsZero) {
  // `offset` is optional (the whole tensor is the default), and the naive way
  // to spell that — `as<int>(0)` — cannot tell "the key is absent" from "the
  // key is there and unparseable". 0 is a perfectly legal offset, so a typed
  // `6.0` would slice [0, 6) of a tensor whose arm half starts at 6: the arm
  // driven by the hand's elements, finite and inside the joint limits, with no
  // diagnostic anywhere. `count` cannot fail this way because its own default
  // of 0 is already illegal.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 12]
output_features:
  - { tensor: "action", role: "joint_target", device: "arm", offset: 6.0, count: 6 }
)";
  ExpectRejectMentioning(yaml, "must declare an integer offset >= 0");
}

TEST(PolicyIoParams, RejectsTheSameRoleTwiceOnOneDevice) {
  // Two disjoint slices, so nothing overlaps and both widths fit — the config
  // is wrong only in that one device cannot be driven twice in the same role.
  // The pre-#511 binding resolved this by keeping whichever came last.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { tensor: "action", role: "joint_target", device: "arm", offset: 0, count: 3 }
  - { tensor: "action", role: "joint_target", device: "arm", offset: 3, count: 3 }
)";
  ExpectRejectMentioning(yaml, "repeats arm/joint_target, already declared at output_features[0]");
}

TEST(PolicyIoParams, AllowsTheSameRoleOnDifferentDevices) {
  // The shape #511 B-2 was found on: a multi-output policy that commands two
  // devices in the same role. This must PARSE — refusing it would be the same
  // mistake in the other direction.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "arm_action"
    shape: [1, 6]
  - name: "hand_action"
    shape: [1, 10]
output_features:
  - { tensor: "arm_action",  role: "joint_target", device: "ur5e" }
  - { tensor: "hand_action", role: "joint_target", device: "p1b" }
)";
  const auto p = ParseText(yaml);
  ASSERT_EQ(p.output_features.size(), 2U);
  EXPECT_EQ(p.output_features[0].device, "ur5e");
  EXPECT_EQ(p.output_features[1].device, "p1b");
  EXPECT_EQ(p.output_features[0].role, p.output_features[1].role);
  EXPECT_EQ(p.output_features[0].slice.count, 6);
  EXPECT_EQ(p.output_features[1].slice.count, 10);
}

TEST(PolicyIoParams, RejectsAnOutputFeatureWithoutARole) {
  // Written out rather than edited out of kBaseYaml: string surgery on a
  // fixture that clang-format may realign is a case that fails for the wrong
  // reason the day the alignment moves (it did, once, in this very file).
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { tensor: "action", device: "arm" }
)";
  ExpectRejectMentioning(yaml, "must declare what the slice means");
}

TEST(PolicyIoParams, RejectsAnOutputFeatureWithoutADevice) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { tensor: "action", role: "joint_target" }
)";
  ExpectRejectMentioning(yaml, "must declare the device group it drives");
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

// ── Recurrent links (#511 P5) ───────────────────────────────────────────────

namespace {

/// A one-layer recurrent policy: `obs` + `h_in` in, `action` + `h_out` out,
/// with `h_out` feeding `h_in` on the next step.
constexpr const char* kRecurrentYaml = R"(
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
  - name: "h_out"
    shape: [1, 1, 4]
    feeds: "h_in"
output_features:
  - { tensor: "action", role: "joint_target", device: "arm" }
)";

}  // namespace

TEST(PolicyIoParams, ResolvesARecurrentLink) {
  const auto p = ParseText(kRecurrentYaml);

  ASSERT_EQ(p.inputs.size(), 2U);
  EXPECT_FALSE(p.inputs[0].recurrent);
  EXPECT_TRUE(p.inputs[1].recurrent);
  EXPECT_TRUE(p.inputs[1].features.empty()) << "a recurrent tensor has no observation features";
  EXPECT_TRUE(p.inputs[1].segments.empty());
  EXPECT_EQ(p.inputs[1].Numel(), 4U);

  ASSERT_EQ(p.outputs.size(), 2U);
  EXPECT_TRUE(p.outputs[0].feeds.empty());
  EXPECT_EQ(p.outputs[1].feeds, "h_in");

  ASSERT_EQ(p.recurrent_links.size(), 1U);
  EXPECT_EQ(p.recurrent_links[0].output_tensor, 1);
  EXPECT_EQ(p.recurrent_links[0].input_tensor, 1);
  EXPECT_EQ(p.recurrent_links[0].numel, 4U);

  // The state tensor is NOT a command, so it contributes no slice.
  ASSERT_EQ(p.output_features.size(), 1U);
  EXPECT_EQ(p.output_features[0].slice.tensor, 0);
}

TEST(PolicyIoParams, AFeedForwardSchemaHasNoLinks) {
  EXPECT_TRUE(ParseText(kBaseYaml).recurrent_links.empty());
}

TEST(PolicyIoParams, RejectsAnInputThatIsBothFeaturesAndRecurrent) {
  // Every step after the first would overwrite the observation with the
  // policy's own last answer, and the actions would stay finite throughout.
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
    source: recurrent
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { tensor: "action", role: "joint_target", device: "arm" }
)";
  ExpectRejectMentioning(yaml, "not both and not neither");
}

TEST(PolicyIoParams, RejectsAnInputThatIsNeitherFeaturesNorRecurrent) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
outputs:
  - name: "action"
    shape: [1, 6]
output_features:
  - { tensor: "action", role: "joint_target", device: "arm" }
)";
  ExpectRejectMentioning(yaml, "not both and not neither");
}

TEST(PolicyIoParams, RejectsAnUnknownInputSource) {
  // "sensor" is a name no filler owns. The case used to spell `constant` here,
  // which stops being unknown once a policy needs a tensor held at fixed values
  // (a root pose the observation already expresses in its own frame).
  std::string yaml = kRecurrentYaml;
  yaml.replace(yaml.find("source: recurrent"), std::string("source: recurrent").size(),
               "source: sensor");
  ExpectRejectMentioning(yaml, "declares source 'sensor'");
}

TEST(PolicyIoParams, RejectsAnAffineLaneOnARecurrentTensor) {
  std::string yaml = kRecurrentYaml;
  yaml.replace(yaml.find("    source: recurrent"), std::string("    source: recurrent").size(),
               "    source: recurrent\n    scale: [1.0, 1.0, 1.0, 1.0]");
  ExpectRejectMentioning(yaml, "cannot carry an affine lane");
}

TEST(PolicyIoParams, RejectsFeedsToAnUndeclaredInput) {
  std::string yaml = kRecurrentYaml;
  yaml.replace(yaml.find("    feeds: \"h_in\""), std::string("    feeds: \"h_in\"").size(),
               "    feeds: \"c_in\"");
  ExpectRejectMentioning(yaml, "feeds 'c_in' but no such input tensor is declared");
}

TEST(PolicyIoParams, RejectsFeedsToAnObservationInput) {
  std::string yaml = kRecurrentYaml;
  yaml.replace(yaml.find("    feeds: \"h_in\""), std::string("    feeds: \"h_in\"").size(),
               "    feeds: \"obs\"");
  ExpectRejectMentioning(yaml, "filled by observation features");
}

TEST(PolicyIoParams, RejectsAFeedsWidthMismatch) {
  // The failure a retrain that changed the hidden width produces. Both tensors
  // are perfectly valid on their own.
  std::string yaml = kRecurrentYaml;
  yaml.replace(yaml.rfind("    shape: [1, 1, 4]"), std::string("    shape: [1, 1, 4]").size(),
               "    shape: [1, 1, 8]");
  ExpectRejectMentioning(yaml, "8 elements but feeds input 'h_in', which has 4");
}

TEST(PolicyIoParams, RejectsTwoOutputsFeedingOneInput) {
  const std::string yaml = R"(
inputs:
  - name: "obs"
    shape: [1, 6]
    features: ["ur5e.position"]
  - name: "h_in"
    shape: [1, 4]
    source: recurrent
outputs:
  - name: "action"
    shape: [1, 6]
  - name: "h_out"
    shape: [1, 4]
    feeds: "h_in"
  - name: "c_out"
    shape: [1, 4]
    feeds: "h_in"
output_features:
  - { tensor: "action", role: "joint_target", device: "arm" }
)";
  ExpectRejectMentioning(yaml, "already feeds");
}

TEST(PolicyIoParams, RejectsARecurrentInputNothingFeeds) {
  // Never written after the initial zero, so the policy reads a constant zero
  // state forever while looking exactly like a working recurrent policy.
  std::string yaml = kRecurrentYaml;
  yaml.replace(yaml.find("    feeds: \"h_in\"\n"), std::string("    feeds: \"h_in\"\n").size(), "");
  ExpectRejectMentioning(yaml, "no output declares `feeds:");
}

TEST(PolicyIoParams, RejectsSlicingATensorThatFeedsAnInput) {
  // Hidden units read as radians: finite, in range, and driving the arm.
  std::string yaml = kRecurrentYaml;
  yaml.replace(
      yaml.find("  - { tensor: \"action\", role: \"joint_target\", device: \"arm\" }"),
      std::string("  - { tensor: \"action\", role: \"joint_target\", device: \"arm\" }").size(),
      "  - { tensor: \"action\", role: \"joint_target\", device: \"arm\" }\n"
      "  - { tensor: \"h_out\", role: \"posture_scalar\", device: \"hand\", offset: 0, "
      "count: 1 }");
  ExpectRejectMentioning(yaml, "which feeds recurrent input 'h_in'");
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

// ── CopyFiniteChecked: the recurrent feedback primitive ─────────────────────

TEST(PolicyIoCore, CopyFiniteCheckedCopiesAFiniteState) {
  const std::array<float, 4> src{1.0F, -2.0F, 0.0F, 3.5F};
  std::vector<float> dst(4, 99.0F);
  EXPECT_TRUE(CopyFiniteChecked(dst, src.data(), src.size()));
  EXPECT_FLOAT_EQ(dst[0], 1.0F);
  EXPECT_FLOAT_EQ(dst[3], 3.5F);
}

TEST(PolicyIoCore, CopyFiniteCheckedZeroesRatherThanFreezingANonFiniteState) {
  // The requirement, not a nicety. A NaN latched into the state becomes the
  // next step's input, every subsequent output is NaN, every tick fails its
  // finiteness check and holds — and the hold path is silent, so the robot
  // stops with nothing in the log and no recovery short of re-activation.
  const std::array<float, 3> src{1.0F, std::numeric_limits<float>::quiet_NaN(), 3.0F};
  std::vector<float> dst{7.0F, 8.0F, 9.0F};
  EXPECT_FALSE(CopyFiniteChecked(dst, src.data(), src.size()));
  for (const float v : dst) {
    EXPECT_FLOAT_EQ(v, 0.0F) << "a refused state must be RESET, never left as it was";
  }
}

TEST(PolicyIoCore, CopyFiniteCheckedZeroesOnAnInfinity) {
  const std::array<float, 2> src{std::numeric_limits<float>::infinity(), 1.0F};
  std::vector<float> dst{5.0F, 5.0F};
  EXPECT_FALSE(CopyFiniteChecked(dst, src.data(), src.size()));
  EXPECT_FLOAT_EQ(dst[0], 0.0F);
  EXPECT_FLOAT_EQ(dst[1], 0.0F);
}

TEST(PolicyIoCore, CopyFiniteCheckedZeroesOnAShortOrNullSource) {
  // Whatever `dst` holds was written for a step that no longer applies, so a
  // source it cannot trust is the same refusal as a NaN.
  const std::array<float, 2> src{1.0F, 2.0F};
  std::vector<float> dst{4.0F, 4.0F, 4.0F};
  EXPECT_FALSE(CopyFiniteChecked(dst, src.data(), src.size()));
  EXPECT_FLOAT_EQ(dst[2], 0.0F);

  std::vector<float> other{6.0F};
  EXPECT_FALSE(CopyFiniteChecked(other, nullptr, 4));
  EXPECT_FLOAT_EQ(other[0], 0.0F);
}

TEST(PolicyIoCore, CopyFiniteCheckedZeroesOnALongSourceRatherThanTruncatingIt) {
  // The other half of the size refusal, and the one that could pass unnoticed:
  // a source LONGER than `dst` has a valid prefix, so copying it would look
  // like a success and hand the policy a clipped state. #511 D-2 links whole
  // tensors, which is why nothing produces this today — and exactly why the
  // guard has to be here before a slice-level link makes it reachable.
  const std::array<float, 4> src{1.0F, 2.0F, 3.0F, 4.0F};
  std::vector<float> dst{7.0F, 7.0F};
  EXPECT_FALSE(CopyFiniteChecked(dst, src.data(), src.size()));
  for (const float v : dst) {
    EXPECT_FLOAT_EQ(v, 0.0F) << "a truncated state is refused, not silently accepted";
  }
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
