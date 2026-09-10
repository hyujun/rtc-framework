// ── Model I/O comparison: the diagnostic contract ────────────────────────────
//
// This is where the shape check is actually tested. `OnnxEngine::Init` can only
// be exercised with a real .onnx file and this repo's test environment has no
// way to build one (no python `onnx` module), so the comparison was written as
// a pure function precisely so that it could be covered here instead.
//
// The cases are grouped around the two properties that made it worth extracting
// rather than the parser's bookkeeping:
//
//   * EVERY discrepancy is reported, not the first. A retrain moves several
//     tensors at once, and a first-mismatch throw costs one bring-up cycle per
//     tensor to discover the rest.
//   * The numbers reach the operator. The message this replaced said
//     "output[0] shape mismatch vs config" and named neither shape.
//
// One case (PositionalPairingCannotSeeASwapOfTwoIdenticalShapes) pins a KNOWN
// BLIND SPOT rather than a guarantee. It is here so the limitation is a
// recorded property with a name, not something rediscovered later.

#include "rtc_inference/shape_report.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using rtc::CompareModelIo;
using rtc::TensorSpec;
using rtc::TensorVerdict;

/// The shipped-schema shape of things: one input, two heads of differing width.
std::vector<TensorSpec> OneInput() {
  return {{"obs", {1, 34}}};
}

std::vector<TensorSpec> TwoHeads() {
  return {{"actions", {1, 6}}, {"posture", {1, 1}}};
}

// ── Agreement ───────────────────────────────────────────────────────────────

TEST(ShapeReport, MatchingIoPasses) {
  const auto report = CompareModelIo(OneInput(), OneInput(), TwoHeads(), TwoHeads());
  EXPECT_TRUE(report.Ok());
  ASSERT_EQ(report.inputs.size(), 1U);
  ASSERT_EQ(report.outputs.size(), 2U);
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kOk);
  EXPECT_EQ(report.outputs[0].verdict, TensorVerdict::kOk);
  EXPECT_EQ(report.outputs[1].verdict, TensorVerdict::kOk);
}

TEST(ShapeReport, ADynamicModelDimensionAcceptsAnyDeclaredSize) {
  // A model exported with a dynamic batch axis reports -1 there. Refusing it
  // would reject every model exported the normal way; the DECLARED side is what
  // must be static, because it sizes a real buffer.
  const std::vector<TensorSpec> model{{"obs", {-1, 34}}};
  const std::vector<TensorSpec> declared{{"", {1, 34}}};
  const auto report = CompareModelIo(model, declared, TwoHeads(), TwoHeads());
  EXPECT_TRUE(report.Ok());
}

TEST(ShapeReport, AnEmptyComparisonIsVacuouslyOk) {
  const auto report = CompareModelIo({}, {}, {}, {});
  EXPECT_TRUE(report.Ok());
}

// ── Arity ───────────────────────────────────────────────────────────────────

TEST(ShapeReport, AModelInputTheConfigDoesNotDeclareIsReported) {
  // The case this whole change exists for: a multi-input policy against a
  // single-input schema. Before, the input side had NO arity check at all —
  // the extra tensors went unmentioned and the model died later inside ORT.
  const std::vector<TensorSpec> model{
      {"obs", {1, 34}}, {"h_in", {1, 1, 256}}, {"c_in", {1, 1, 256}}};
  const auto report = CompareModelIo(model, OneInput(), TwoHeads(), TwoHeads());

  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.inputs.size(), 3U);
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kOk);
  EXPECT_EQ(report.inputs[1].verdict, TensorVerdict::kNotDeclared);
  EXPECT_EQ(report.inputs[2].verdict, TensorVerdict::kNotDeclared);
  EXPECT_EQ(report.model_inputs, 3U);
  EXPECT_EQ(report.declared_inputs, 1U);

  // The names are what let an operator match the row to what Netron shows.
  EXPECT_EQ(report.inputs[1].name, "h_in");
  EXPECT_EQ(report.inputs[2].name, "c_in");
}

TEST(ShapeReport, AConfigTensorAbsentFromTheModelIsReported) {
  // The declared side is spelled WITHOUT a name here because that is what the
  // engine passes today — the schema has no way to name a tensor yet. With no
  // name on either side the row falls back to a positional label, which is the
  // only thing left to identify it by.
  const std::vector<TensorSpec> declared{{"", {1, 34}}};
  const auto report = CompareModelIo({}, declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.inputs.size(), 1U);
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kNotInModel);
  EXPECT_EQ(report.inputs[0].name, "#0");
}

TEST(ShapeReport, ADeclaredNameIsUsedWhenTheModelSideHasNoneToOffer) {
  // Once the schema can name tensors, a config-only tensor must still be
  // identifiable by the name the operator wrote.
  const std::vector<TensorSpec> declared{{"h_in", {1, 1, 256}}};
  const auto report = CompareModelIo({}, declared, TwoHeads(), TwoHeads());
  ASSERT_EQ(report.inputs.size(), 1U);
  EXPECT_EQ(report.inputs[0].name, "h_in");
}

TEST(ShapeReport, AnExtraOutputHeadIsAMismatchRatherThanBeingDropped) {
  // Extra heads would be silently ignored by positional binding, so "the model
  // has more than we declared" has to fail rather than pass quietly.
  const std::vector<TensorSpec> model{{"actions", {1, 6}}, {"posture", {1, 1}}, {"value", {1, 1}}};
  const auto report = CompareModelIo(OneInput(), OneInput(), model, TwoHeads());
  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.outputs.size(), 3U);
  EXPECT_EQ(report.outputs[2].verdict, TensorVerdict::kNotDeclared);
}

TEST(ShapeReport, AMissingOutputHeadIsAMismatch) {
  const std::vector<TensorSpec> model{{"actions", {1, 6}}};
  const auto report = CompareModelIo(OneInput(), OneInput(), model, TwoHeads());
  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.outputs.size(), 2U);
  EXPECT_EQ(report.outputs[1].verdict, TensorVerdict::kNotInModel);
}

// ── Shape ───────────────────────────────────────────────────────────────────

TEST(ShapeReport, AStaticDimensionDisagreementNamesTheDimension) {
  const std::vector<TensorSpec> declared{{"", {1, 27}}};
  const auto report = CompareModelIo(OneInput(), declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.inputs.size(), 1U);
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kShapeMismatch);
  EXPECT_EQ(report.inputs[0].first_bad_dim, 1);
}

TEST(ShapeReport, ARankDisagreementIsItsOwnVerdict) {
  // Distinguished from a dimension mismatch because the fix is different: a
  // rank change means the export shape changed, not a width.
  const std::vector<TensorSpec> declared{{"", {1, 1, 34}}};
  const auto report = CompareModelIo(OneInput(), declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kRankMismatch);
  EXPECT_EQ(report.inputs[0].first_bad_dim, -1);
}

TEST(ShapeReport, ANonPositiveDeclaredDimensionIsRefusedBeforeAnythingElse) {
  // A declared dim sizes a real allocation: a -1 there would cast to SIZE_MAX
  // in the element-count product and try a catastrophic allocation. It is
  // judged BEFORE rank so the message names the actual operator error rather
  // than a downstream symptom of it.
  const std::vector<TensorSpec> declared{{"", {1, 0, 7}}};
  const auto report = CompareModelIo(OneInput(), declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kBadDeclaredDim);
  EXPECT_EQ(report.inputs[0].first_bad_dim, 1);
}

TEST(ShapeReport, PositionalPairingCannotSeeASwapOfTwoIdenticalShapes) {
  // A KNOWN BLIND SPOT, pinned so it stays a named property.
  //
  // Two heads of the same shape exported in the other order compare as OK,
  // because pairing is positional and the shapes are indistinguishable. This is
  // live in `udp_hand_driver`, whose 3-head FT model declares {{1,1},{1,3},{1,3}}
  // — force and direction are both [1,3]. Name-based pairing is what closes it.
  const std::vector<TensorSpec> model{{"force", {1, 3}}, {"direction", {1, 3}}};
  const std::vector<TensorSpec> swapped{{"direction", {1, 3}}, {"force", {1, 3}}};
  const auto report = CompareModelIo(OneInput(), OneInput(), model, swapped);
  EXPECT_TRUE(report.Ok()) << "if this fails, pairing is no longer positional and the "
                              "blind spot this documents has been closed — update the test";
}

// ── The table ───────────────────────────────────────────────────────────────

TEST(ShapeReport, EveryDiscrepancyIsReportedNotJustTheFirst) {
  // The headline property. Three independent problems across both sides must
  // all appear in one message, because a retrain typically moves several
  // tensors together and a first-mismatch throw costs a bring-up cycle each.
  const std::vector<TensorSpec> model_in{{"obs", {1, 34}}, {"h_in", {1, 1, 256}}};
  const std::vector<TensorSpec> declared_in{{"", {1, 40}}};
  const std::vector<TensorSpec> model_out{{"actions", {1, 6}}, {"posture", {1, 1}}};
  const std::vector<TensorSpec> declared_out{{"", {1, 6}}, {"", {1, 2}}};

  const auto report = CompareModelIo(model_in, declared_in, model_out, declared_out);
  ASSERT_FALSE(report.Ok());

  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kShapeMismatch);
  EXPECT_EQ(report.inputs[1].verdict, TensorVerdict::kNotDeclared);
  EXPECT_EQ(report.outputs[0].verdict, TensorVerdict::kOk);
  EXPECT_EQ(report.outputs[1].verdict, TensorVerdict::kShapeMismatch);

  const std::string text = report.Format("/tmp/policy.onnx");
  EXPECT_NE(text.find("/tmp/policy.onnx"), std::string::npos);
  // Each of the three problems is visible, by name where one exists.
  EXPECT_NE(text.find("obs"), std::string::npos);
  EXPECT_NE(text.find("h_in"), std::string::npos);
  EXPECT_NE(text.find("posture"), std::string::npos);
  // Arity is stated for both sides.
  EXPECT_NE(text.find("(model 2 / config 1)"), std::string::npos);
  EXPECT_NE(text.find("(model 2 / config 2)"), std::string::npos);
}

TEST(ShapeReport, TheTableCarriesBothNumbersOfAMismatch) {
  // The whole point of replacing "shape mismatch vs config": an operator must
  // be able to act on the message without opening the model.
  const std::vector<TensorSpec> declared{{"", {1, 27}}};
  const auto text =
      CompareModelIo(OneInput(), declared, TwoHeads(), TwoHeads()).Format("policy.onnx");
  EXPECT_NE(text.find("[1, 34]"), std::string::npos) << text;
  EXPECT_NE(text.find("[1, 27]"), std::string::npos) << text;
  EXPECT_NE(text.find("dim 1"), std::string::npos) << text;
}

TEST(ShapeReport, ADynamicDimensionIsRenderedAsAQuestionMarkNotAsMinusOne) {
  // "?" says "the model accepts any size here". A raw -1 reads as a corrupt
  // shape and sends the reader looking for the wrong problem.
  const std::vector<TensorSpec> model{{"obs", {-1, 34}}};
  const std::vector<TensorSpec> declared{{"", {1, 33}}};  // forced failure, to get a table
  const auto text = CompareModelIo(model, declared, TwoHeads(), TwoHeads()).Format("m.onnx");
  EXPECT_NE(text.find("[?, 34]"), std::string::npos) << text;
  EXPECT_EQ(text.find("[-1, 34]"), std::string::npos) << text;
}

TEST(ShapeReport, AnAbsentSideRendersAsADash) {
  const std::vector<TensorSpec> model{{"obs", {1, 34}}, {"extra", {1, 5}}};
  const auto text = CompareModelIo(model, OneInput(), TwoHeads(), TwoHeads()).Format("m.onnx");
  EXPECT_NE(text.find("config --"), std::string::npos) << text;
}

}  // namespace
