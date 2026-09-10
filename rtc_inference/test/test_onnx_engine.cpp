// ── OnnxEngine against a real ONNX Runtime session ───────────────────────────
//
// The other half of the shape gate. `test_shape_report.cpp` proves the RULES
// are right by driving a pure function; this file proves they are WIRED right
// by loading an actual .onnx through ORT. Neither substitutes for the other: a
// perfect comparison bolted to the wrong buffers is still a silently wrong
// robot.
//
// THE FIXTURE IS THE ARGUMENT. `two_in_two_out.onnx` has two inputs and two
// outputs and EVERY tensor is [1,2]. With all four shapes identical, shape
// validation provably cannot tell them apart — so any test below that gets the
// right numbers out could only have got them by binding NAMES. And because the
// graph computes sum/diff, a swapped binding is not merely detectable in the
// metadata: it changes the sign of an answer.
//
// The whole file compiles away without ONNX Runtime, exactly like the engine it
// tests. That is honest rather than convenient: with no ORT there is no session
// to bind to and a green here would mean nothing.

#include "rtc_inference/onnx/onnx_engine.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#ifdef HAS_ONNXRUNTIME

namespace rtc {
namespace {

/// Absolute path to the committed fixture, injected by CMake so the test does
/// not depend on the working directory colcon happens to run it from.
std::string FixturePath() {
  return std::string(RTC_INFERENCE_TEST_DATA_DIR) + "/two_in_two_out.onnx";
}

/// Declaration matching the fixture, in a chosen order.
ModelConfig MakeConfig(const std::vector<TensorSpec>& inputs,
                       const std::vector<TensorSpec>& outputs) {
  ModelConfig c;
  c.model_path = FixturePath();
  c.inputs = inputs;
  c.outputs = outputs;
  return c;
}

const std::vector<std::int64_t> kShape{1, 2};

// ── Loading ─────────────────────────────────────────────────────────────────

TEST(OnnxEngineFixture, LoadsAModelWhoseDeclarationMatches) {
  OnnxEngine engine;
  ASSERT_NO_THROW(
      engine.Init(MakeConfig({{"a", kShape}, {"b", kShape}}, {{"sum", kShape}, {"diff", kShape}})));
  EXPECT_TRUE(engine.is_initialized());
  EXPECT_EQ(engine.num_models(), 1);
  EXPECT_EQ(engine.num_inputs(0), 2);
  EXPECT_EQ(engine.num_outputs(0), 2);
  EXPECT_EQ(engine.input_size(0, 0), 2U);
  EXPECT_EQ(engine.input_size(0, 1), 2U);
  // Out of range stays nullptr/0 rather than trapping: the RT tick turns that
  // into a hold, and one branch then covers both this and a stub engine.
  EXPECT_EQ(engine.input_buffer(0, 2), nullptr);
  EXPECT_EQ(engine.input_size(0, 2), 0U);
}

TEST(OnnxEngineFixture, AMultiInputModelIsNoLongerAcceptedAsSingleInput) {
  // The regression this phase exists for. The old check was `GetInputCount() <
  // 1`, so a 2-input model sailed past it, bound only input 0, and died inside
  // ORT during warmup with a message that named neither count.
  OnnxEngine engine;
  try {
    engine.Init(MakeConfig({{"a", kShape}}, {{"sum", kShape}, {"diff", kShape}}));
    FAIL() << "a model with more inputs than declared must be refused";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("model 2 / config 1"), std::string::npos) << msg;
    // Anchored on the row prefix, not on a bare "b" — a single letter matches
    // inside "config" and would make this assertion pass on any message.
    EXPECT_NE(msg.find("] b "), std::string::npos) << msg;
    EXPECT_NE(msg.find("the model has it, the config does not declare it"), std::string::npos)
        << msg;
  }
}

TEST(OnnxEngineFixture, AMisnamedTensorIsRefusedAndBothNamesAppear) {
  OnnxEngine engine;
  try {
    engine.Init(MakeConfig({{"a", kShape}, {"beta", kShape}}, {{"sum", kShape}, {"diff", kShape}}));
    FAIL() << "a declared name the model does not export must be refused";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    // Both halves of the answer must be present: what the config asked for, and
    // what the model actually has — that pairing is what turns "beta is
    // missing" into "you meant b". The second is anchored on the row prefix
    // because a bare "b" also matches inside "beta".
    EXPECT_NE(msg.find("beta"), std::string::npos) << msg;
    EXPECT_NE(msg.find("] b "), std::string::npos) << msg;
    EXPECT_NE(msg.find("declared in the config, absent from the model"), std::string::npos) << msg;
  }
}

TEST(OnnxEngineFixture, AWrongShapeIsRefusedWithBothShapes) {
  OnnxEngine engine;
  try {
    engine.Init(MakeConfig({{"a", kShape}, {"b", {1, 3}}}, {{"sum", kShape}, {"diff", kShape}}));
    FAIL() << "a declared shape the model contradicts must be refused";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("[1, 2]"), std::string::npos) << msg;
    EXPECT_NE(msg.find("[1, 3]"), std::string::npos) << msg;
  }
}

TEST(OnnxEngineFixture, ARegisterOrNothingInitLeavesNoHalfModelBehind) {
  // Init must be atomic: a refused model may not leave a session in models_,
  // or the next Run would drive a model nobody validated.
  OnnxEngine engine;
  EXPECT_ANY_THROW(engine.Init(MakeConfig({{"a", kShape}}, {{"sum", kShape}})));
  EXPECT_EQ(engine.num_models(), 0);
  EXPECT_FALSE(engine.is_initialized());
}

// ── Binding ─────────────────────────────────────────────────────────────────

/// Fill the two inputs and run, returning {sum, diff} as the DECLARED order
/// sees them.
struct RunResult {
  std::vector<float> first;
  std::vector<float> second;
};

RunResult RunWith(OnnxEngine& engine, float a0, float a1, float b0, float b1, int a_slot,
                  int b_slot) {
  float* a = engine.input_buffer(0, a_slot);
  float* b = engine.input_buffer(0, b_slot);
  EXPECT_NE(a, nullptr);
  EXPECT_NE(b, nullptr);
  a[0] = a0;
  a[1] = a1;
  b[0] = b0;
  b[1] = b1;
  EXPECT_TRUE(engine.Run());
  const float* o0 = engine.output_buffer(0, 0);
  const float* o1 = engine.output_buffer(0, 1);
  return {{o0[0], o0[1]}, {o1[0], o1[1]}};
}

TEST(OnnxEngineFixture, BuffersAreBoundToTheTensorTheyName) {
  OnnxEngine engine;
  ASSERT_NO_THROW(
      engine.Init(MakeConfig({{"a", kShape}, {"b", kShape}}, {{"sum", kShape}, {"diff", kShape}})));
  // a = (5, 7), b = (1, 2)  ->  sum = (6, 9), diff = (4, 5)
  const auto r = RunWith(engine, 5.0F, 7.0F, 1.0F, 2.0F, /*a_slot=*/0, /*b_slot=*/1);
  EXPECT_FLOAT_EQ(r.first[0], 6.0F);
  EXPECT_FLOAT_EQ(r.first[1], 9.0F);
  EXPECT_FLOAT_EQ(r.second[0], 4.0F);
  EXPECT_FLOAT_EQ(r.second[1], 5.0F);
}

TEST(OnnxEngineFixture, DeclaringTensorsInTheOtherOrderReordersTheBuffersNotTheMeaning) {
  // THE HEADLINE. The config lists b before a and diff before sum — the
  // opposite of the model's own order, and indistinguishable from it by shape.
  // Slot 0 must now be `b` and slot 0 of the outputs must be `diff`, while the
  // ARITHMETIC is unchanged: diff is still a - b, not b - a.
  //
  // Under positional binding this test computes b - a and fails by sign, which
  // is precisely the silent corruption named-binding exists to prevent.
  OnnxEngine engine;
  ASSERT_NO_THROW(
      engine.Init(MakeConfig({{"b", kShape}, {"a", kShape}}, {{"diff", kShape}, {"sum", kShape}})));
  // a is slot 1 now, b is slot 0.
  const auto r = RunWith(engine, 5.0F, 7.0F, 1.0F, 2.0F, /*a_slot=*/1, /*b_slot=*/0);
  EXPECT_FLOAT_EQ(r.first[0], 4.0F) << "output slot 0 should be diff = a - b";
  EXPECT_FLOAT_EQ(r.first[1], 5.0F);
  EXPECT_FLOAT_EQ(r.second[0], 6.0F) << "output slot 1 should be sum = a + b";
  EXPECT_FLOAT_EQ(r.second[1], 9.0F);
}

TEST(OnnxEngineFixture, UnnamedDeclarationsStillBindPositionally) {
  // The compatibility path `udp_hand_driver` is on: no names, so slots follow
  // the model's own order. It works, and it is exactly what cannot detect the
  // reordering the previous test survives.
  OnnxEngine engine;
  ASSERT_NO_THROW(
      engine.Init(MakeConfig({{"", kShape}, {"", kShape}}, {{"", kShape}, {"", kShape}})));
  const auto r = RunWith(engine, 5.0F, 7.0F, 1.0F, 2.0F, /*a_slot=*/0, /*b_slot=*/1);
  EXPECT_FLOAT_EQ(r.first[0], 6.0F);
  EXPECT_FLOAT_EQ(r.second[0], 4.0F);
}

TEST(OnnxEngineFixture, PartiallyNamedDeclarationsAreRefused) {
  OnnxEngine engine;
  try {
    engine.Init(MakeConfig({{"a", kShape}, {"", kShape}}, {{"sum", kShape}, {"diff", kShape}}));
    FAIL() << "a half-named side must be refused rather than guessed at";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("REFUSED"), std::string::npos) << e.what();
  }
}

TEST(OnnxEngineFixture, ResetMakesInitIdempotent) {
  OnnxEngine engine;
  const auto config =
      MakeConfig({{"a", kShape}, {"b", kShape}}, {{"sum", kShape}, {"diff", kShape}});
  ASSERT_NO_THROW(engine.Init(config));
  ASSERT_NO_THROW(engine.Init(config));
  EXPECT_EQ(engine.num_models(), 2) << "Init registers an additional model by design";
  engine.Reset();
  EXPECT_EQ(engine.num_models(), 0);
  EXPECT_FALSE(engine.is_initialized());
  ASSERT_NO_THROW(engine.Init(config));
  EXPECT_EQ(engine.num_models(), 1);
}

TEST(OnnxEngineFixture, RunModelsDrivesTheDirectSessionPath) {
  // RunModels() bypasses IoBinding and marshals names/values itself — a
  // separate code path from Run(), and the one the RT tick uses. It must reach
  // the same numbers.
  OnnxEngine engine;
  ASSERT_NO_THROW(
      engine.Init(MakeConfig({{"a", kShape}, {"b", kShape}}, {{"sum", kShape}, {"diff", kShape}})));
  float* a = engine.input_buffer(0, 0);
  float* b = engine.input_buffer(0, 1);
  a[0] = 5.0F;
  a[1] = 7.0F;
  b[0] = 1.0F;
  b[1] = 2.0F;
  const int idx = 0;
  ASSERT_TRUE(engine.RunModels(&idx, 1));
  EXPECT_FLOAT_EQ(engine.output_buffer(0, 0)[0], 6.0F);
  EXPECT_FLOAT_EQ(engine.output_buffer(0, 1)[0], 4.0F);
}

}  // namespace
}  // namespace rtc

#endif  // HAS_ONNXRUNTIME
