// ── test_inference_engine.cpp ────────────────────────────────────────────────
// Pure-logic unit tests for the rtc_inference INTERFACE library:
//   - InferenceEngine base-class default RunModels/RunModel delegation
//   - OnnxEngine pre-Init guard paths (no .onnx model required)
//   - ModelConfig defaults
//
// The OnnxEngine Init→Run happy-path (ONNX Runtime session load + buffer
// sizing) is integration-scope: it requires a real .onnx model fixture and
// exercises ORT itself, not rtc logic. It is intentionally NOT covered here.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_inference/onnx/onnx_engine.hpp"

#include <gtest/gtest.h>

#include <cstddef>

namespace rtc {
namespace {

// Minimal concrete engine that records Run() calls and can be told to fail on a
// specific (1-based) call. RunModels/RunModel are NOT overridden, so they
// exercise the InferenceEngine base-class defaults.
class CountingEngine : public InferenceEngine {
 public:
  int run_calls{0};
  int fail_on_call{-1};  // 1-based call number that returns false; -1 = never

  void Init(const ModelConfig&) override {}

  [[nodiscard]] bool Run() noexcept override {
    ++run_calls;
    return run_calls != fail_on_call;
  }

  float* input_buffer(int = 0) noexcept override { return nullptr; }

  const float* output_buffer(int = 0, int = 0) const noexcept override { return nullptr; }

  [[nodiscard]] std::size_t input_size(int = 0) const noexcept override { return 0; }

  [[nodiscard]] std::size_t output_size(int = 0, int = 0) const noexcept override { return 0; }

  [[nodiscard]] int num_outputs(int = 0) const noexcept override { return 0; }

  [[nodiscard]] bool is_initialized() const noexcept override { return true; }

  [[nodiscard]] int num_models() const noexcept override { return 0; }
};

// ── InferenceEngine base-class default methods ──────────────────────────────

// RunModel() default delegates to Run() once.
TEST(InferenceEngineDefaults, RunModelDelegatesToRun) {
  CountingEngine e;
  EXPECT_TRUE(e.RunModel(7));  // index ignored by the default impl
  EXPECT_EQ(e.run_calls, 1);
}

// RunModels() default runs each index once, returning true when all succeed.
TEST(InferenceEngineDefaults, RunModelsAllSucceed) {
  CountingEngine e;
  const int idx[] = {0, 1, 2};
  EXPECT_TRUE(e.RunModels(idx, 3));
  EXPECT_EQ(e.run_calls, 3);
}

// RunModels() stops at the first failing model and reports false.
TEST(InferenceEngineDefaults, RunModelsStopsAtFirstFailure) {
  CountingEngine e;
  e.fail_on_call = 2;  // 2nd Run() returns false
  const int idx[] = {0, 1, 2};
  EXPECT_FALSE(e.RunModels(idx, 3));
  EXPECT_EQ(e.run_calls, 2);  // 3rd model never reached
}

// Zero count is a vacuous success — the loop body (and pointer deref) is skipped.
TEST(InferenceEngineDefaults, RunModelsZeroCountIsVacuousTrue) {
  CountingEngine e;
  EXPECT_TRUE(e.RunModels(nullptr, 0));
  EXPECT_EQ(e.run_calls, 0);
}

// ── OnnxEngine pre-Init guards (no model loaded) ────────────────────────────

TEST(OnnxEnginePreInit, NotInitializedByDefault) {
  OnnxEngine e;
  EXPECT_FALSE(e.is_initialized());
  EXPECT_EQ(e.num_models(), 0);
}

TEST(OnnxEnginePreInit, RunFailsBeforeInit) {
  OnnxEngine e;
  EXPECT_FALSE(e.Run());
}

TEST(OnnxEnginePreInit, RunModelRejectsBeforeInit) {
  OnnxEngine e;
  EXPECT_FALSE(e.RunModel(0));
  EXPECT_FALSE(e.RunModel(-1));
}

// Out-of-range accessors are RT-safe: nullptr / 0 instead of UB (no model loaded).
TEST(OnnxEnginePreInit, AccessorsOutOfRangeAreSafe) {
  OnnxEngine e;
  EXPECT_EQ(e.input_buffer(0), nullptr);
  EXPECT_EQ(e.output_buffer(0, 0), nullptr);
  EXPECT_EQ(e.input_size(0), 0U);
  EXPECT_EQ(e.output_size(0, 0), 0U);
  EXPECT_EQ(e.num_outputs(0), 0);
}

// Reset() is safe on a fresh engine and leaves it un-initialized (idempotent re-Init support).
TEST(OnnxEnginePreInit, ResetOnFreshEngineIsSafe) {
  OnnxEngine e;
  e.Reset();
  EXPECT_FALSE(e.is_initialized());
  EXPECT_EQ(e.num_models(), 0);
}

#ifdef HAS_ONNXRUNTIME
// The real engine overrides RunModels with explicit !initialized_ and count<=0
// guards (the stub inherits the base default, whose count<=0 is vacuously true,
// so these assertions only hold for the ORT-backed build).
TEST(OnnxEnginePreInit, RunModelsGuardsBeforeInit) {
  OnnxEngine e;
  const int idx[] = {0};
  EXPECT_FALSE(e.RunModels(idx, 1));  // !initialized_
  EXPECT_FALSE(e.RunModels(idx, 0));  // count <= 0
  EXPECT_FALSE(e.RunModels(nullptr, -1));
}
#endif  // HAS_ONNXRUNTIME

// ── ModelConfig defaults ────────────────────────────────────────────────────

TEST(ModelConfigDefaults, FieldsZeroInitializedExceptThreads) {
  ModelConfig c;
  EXPECT_EQ(c.intra_op_threads, 1);  // RT: single-threaded inference default
  EXPECT_TRUE(c.model_path.empty());
  EXPECT_TRUE(c.optimized_model_path.empty());
  EXPECT_TRUE(c.input_shape.empty());
  EXPECT_TRUE(c.output_shapes.empty());
}

}  // namespace
}  // namespace rtc
