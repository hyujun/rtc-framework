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
  // UNNAMED on the declared side — that is what selects positional pairing, and
  // it is how `udp_hand_driver` declares its heads today. Naming them is what
  // closes the hole (see NamedPairingCatchesTheSwapPositionalPairingCannotSee).
  const std::vector<TensorSpec> swapped{{"", {1, 3}}, {"", {1, 3}}};
  const auto report = CompareModelIo(OneInput(), OneInput(), model, swapped);
  ASSERT_EQ(report.output_match, rtc::MatchMode::kPositional)
      << "the fixture stopped exercising positional pairing, so the blind spot "
         "this documents is no longer under test";
  EXPECT_TRUE(report.Ok());
}

// ── Name-based pairing ──────────────────────────────────────────────────────

TEST(ShapeReport, NamedPairingIgnoresTheModelsOwnTensorOrder) {
  // The point of naming. The model exports b before a; the config lists a
  // before b because that is the order ITS buffers are indexed in. Both are
  // right, and the pairing must follow the names rather than either order.
  const std::vector<TensorSpec> model{{"b", {1, 5}}, {"a", {1, 3}}};
  const std::vector<TensorSpec> declared{{"a", {1, 3}}, {"b", {1, 5}}};
  const auto report = CompareModelIo(model, declared, TwoHeads(), TwoHeads());
  EXPECT_TRUE(report.Ok());
  EXPECT_EQ(report.input_match, rtc::MatchMode::kByName);
  // Rows come out in DECLARED order — the same order the consumer's buffer
  // indices run in, so row i describes what input_buffer(m, i) will hold.
  ASSERT_EQ(report.inputs.size(), 2U);
  EXPECT_EQ(report.inputs[0].name, "a");
  EXPECT_EQ(report.inputs[1].name, "b");
}

TEST(ShapeReport, NamedPairingCatchesTheSwapPositionalPairingCannotSee) {
  // The counterpart to PositionalPairingCannotSeeASwapOfTwoIdenticalShapes.
  // Same two [1,3] tensors, same swap — but now the config names them, so the
  // pairing follows the name and the shapes are compared against the RIGHT
  // partner. Here the swap is harmless (both [1,3], so still OK) and what the
  // test pins is that each row is matched to its namesake, not to its position.
  const std::vector<TensorSpec> model{{"force", {1, 3}}, {"direction", {1, 3}}};
  const std::vector<TensorSpec> declared{{"direction", {1, 3}}, {"force", {1, 3}}};
  const auto report = CompareModelIo(OneInput(), OneInput(), model, declared);
  EXPECT_TRUE(report.Ok());
  ASSERT_EQ(report.outputs.size(), 2U);
  EXPECT_EQ(report.outputs[0].name, "direction");
  EXPECT_EQ(report.outputs[1].name, "force");
}

TEST(ShapeReport, ANamedTensorTheModelDoesNotExportIsReportedByName) {
  // The most common retrain failure once names are in use: the tensor was
  // renamed upstream. The message has to name what was looked for.
  const std::vector<TensorSpec> model{{"observation", {1, 34}}};
  const std::vector<TensorSpec> declared{{"obs", {1, 34}}};
  const auto report = CompareModelIo(model, declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.inputs.size(), 2U);
  EXPECT_EQ(report.inputs[0].name, "obs");
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kNotInModel);
  // ...and the unclaimed model tensor is listed too, which is what turns
  // "obs is missing" into "you probably meant observation".
  EXPECT_EQ(report.inputs[1].name, "observation");
  EXPECT_EQ(report.inputs[1].verdict, TensorVerdict::kNotDeclared);
}

TEST(ShapeReport, ANamedPairStillHasItsShapeChecked) {
  const std::vector<TensorSpec> model{{"obs", {1, 34}}};
  const std::vector<TensorSpec> declared{{"obs", {1, 40}}};
  const auto report = CompareModelIo(model, declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kShapeMismatch);
  EXPECT_EQ(report.inputs[0].first_bad_dim, 1);
}

TEST(ShapeReport, PartiallyNamedDeclarationsAreRefusedRatherThanGuessedAt) {
  // Naming half the tensors leaves two different bindings equally defensible
  // (name the named ones and position the rest? position everything?). Picking
  // one silently is how a config that reads correct behaves incorrectly.
  const std::vector<TensorSpec> model{{"a", {1, 3}}, {"b", {1, 3}}};
  const std::vector<TensorSpec> declared{{"a", {1, 3}}, {"", {1, 3}}};
  const auto report = CompareModelIo(model, declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  EXPECT_EQ(report.input_match, rtc::MatchMode::kMixedRefused);
  // A refused side is still walked, so the operator can see WHICH one is
  // unnamed rather than getting a bare refusal.
  EXPECT_EQ(report.inputs.size(), 2U);
  EXPECT_NE(report.Format("m.onnx").find("REFUSED"), std::string::npos);
}

TEST(ShapeReport, ARepeatedDeclaredNameIsRefused) {
  // Two buffers would fight over one model tensor, and only one of them could
  // win — which one is an implementation detail nobody should have to know.
  const std::vector<TensorSpec> model{{"a", {1, 3}}};
  const std::vector<TensorSpec> declared{{"a", {1, 3}}, {"a", {1, 3}}};
  const auto report = CompareModelIo(model, declared, TwoHeads(), TwoHeads());
  EXPECT_FALSE(report.Ok());
  ASSERT_EQ(report.inputs.size(), 2U);
  EXPECT_EQ(report.inputs[0].verdict, TensorVerdict::kOk);
  EXPECT_EQ(report.inputs[1].verdict, TensorVerdict::kDuplicateName);
}

TEST(ShapeReport, TheTableSaysHowEachSideWasPaired) {
  // Inputs named, outputs not — the two sides are decided independently, and
  // the table has to say which is which. A reader who cannot tell the modes
  // apart cannot tell a clean row from one that merely looks clean.
  const std::vector<TensorSpec> model{{"obs", {1, 34}}};
  const std::vector<TensorSpec> named{{"obs", {1, 40}}};  // forced failure, to get a table
  const std::vector<TensorSpec> unnamed_heads{{"", {1, 6}}, {"", {1, 1}}};
  const auto text = CompareModelIo(model, named, TwoHeads(), unnamed_heads).Format("m.onnx");
  EXPECT_NE(text.find("paired by name"), std::string::npos) << text;
  EXPECT_NE(text.find("paired positionally"), std::string::npos) << text;
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
  // Arity is stated for both sides, and so is HOW they were paired — an
  // operator reading this has to know whether a same-shape swap could be
  // hiding behind an otherwise clean row.
  EXPECT_NE(text.find("model 2 / config 1"), std::string::npos) << text;
  EXPECT_NE(text.find("model 2 / config 2"), std::string::npos) << text;
  EXPECT_NE(text.find("paired positionally"), std::string::npos) << text;
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

TEST(ShapeReport, APassingReportDoesNotAnnounceAMismatch) {
  // `Format` is documented as safe to call on a passing report for an
  // informational dump. A hardcoded "does not match" header would make that use
  // state the opposite of the table printed directly underneath it.
  const auto text = CompareModelIo(OneInput(), OneInput(), TwoHeads(), TwoHeads()).Format("m.onnx");
  EXPECT_NE(text.find("I/O matches the declared schema"), std::string::npos) << text;
  EXPECT_EQ(text.find("does not match"), std::string::npos) << text;
}

// ── One side at a time ──────────────────────────────────────────────────────

TEST(ShapeReport, AnUndeclaredSideIsNotAVerdictOnTheDeclaredOne) {
  // What `rtc_inference_check MODEL --input obs:1x34` asks. An undeclared side
  // is an EMPTY declaration, which the pairing rules read as "the config claims
  // none of these tensors" — so `Ok()` is false for a model whose inputs are
  // exactly right. The per-side verdict is what lets the caller ask only about
  // what it declared.
  const auto report = CompareModelIo(OneInput(), OneInput(), TwoHeads(), {});
  EXPECT_TRUE(report.InputsOk());
  EXPECT_FALSE(report.OutputsOk());
  EXPECT_FALSE(report.Ok()) << "the configure-time question still fails: it declares both sides";
}

TEST(ShapeReport, AOneSidedReportRendersAndJudgesOnlyThatSide) {
  const auto report = CompareModelIo(OneInput(), OneInput(), TwoHeads(), {});
  const auto text = report.Format("m.onnx", rtc::ReportSides::kInputsOnly);
  EXPECT_NE(text.find("inputs match the declared schema"), std::string::npos) << text;
  EXPECT_NE(text.find("obs"), std::string::npos) << text;
  EXPECT_EQ(text.find("posture"), std::string::npos)
      << "the undeclared side must not appear in a table that judges it: " << text;
}

TEST(ShapeReport, AOneSidedReportStillReportsThatSidesMismatch) {
  const std::vector<TensorSpec> declared{{"obs", {1, 33}}};
  const auto report = CompareModelIo(OneInput(), declared, TwoHeads(), {});
  EXPECT_FALSE(report.InputsOk());
  const auto text = report.Format("m.onnx", rtc::ReportSides::kInputsOnly);
  EXPECT_NE(text.find("inputs do not match the declared schema"), std::string::npos) << text;
  EXPECT_NE(text.find("dim 1"), std::string::npos) << text;
}

}  // namespace
