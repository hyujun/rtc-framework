// ── Deterministic stand-in for rtc::OnnxEngine (DemoInferenceController tests) ─
//
// Holds real buffers so the controller's pack/unpack path is the production
// one; only the "model" is fake. `run_result` and the output values are
// writable so a case can make a single tick fail without touching the
// controller.
//
// Shared by the roster-fixture suite (test_demo_inference_controller.cpp) and
// the real-URDF suite (test_demo_inference_urdf.cpp). NOT by the allocation
// gate: `Run()` here snapshots every input into `last_inputs`, which allocates,
// and that suite needs an engine whose Run() is itself allocation-free.
#pragma once

#include "rtc_inference/inference_engine.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

class FakeEngine final : public rtc::InferenceEngine {
 public:
  FakeEngine(const std::vector<std::size_t>& in_sizes, const std::vector<std::size_t>& out_sizes) {
    for (const auto n : in_sizes) {
      inputs_.emplace_back(n, 0.0F);
    }
    for (const auto n : out_sizes) {
      outputs_.emplace_back(n, 0.0F);
    }
  }

  void Init(const rtc::ModelConfig& config) override {
    // `stub` reproduces exactly what a build without ONNX Runtime does: Init()
    // returns normally, throws nothing, and leaves is_initialized() false.
    last_model_path = config.model_path;
    ++init_count;
    initialized_ = !stub;
  }

  [[nodiscard]] bool Run() noexcept override {
    ++run_count;
    last_inputs = inputs_;
    for (std::size_t h = 0; h < outputs_.size(); ++h) {
      for (std::size_t i = 0; i < outputs_[h].size(); ++i) {
        outputs_[h][i] =
            (h < next_output.size() && i < next_output[h].size()) ? next_output[h][i] : 0.0F;
      }
    }
    return run_result;
  }

  // Out-of-range answers nullptr/0 rather than folding onto slot 0: a binding
  // that addressed a tensor this "model" does not have must hold, not silently
  // re-read another one.
  float* input_buffer(int /*model_idx*/, int input_idx) noexcept override {
    const auto t = static_cast<std::size_t>(input_idx);
    return (t < inputs_.size()) ? inputs_[t].data() : nullptr;
  }

  [[nodiscard]] const float* output_buffer(int /*model_idx*/,
                                           int output_idx) const noexcept override {
    const auto h = static_cast<std::size_t>(output_idx);
    return (h < outputs_.size()) ? outputs_[h].data() : nullptr;
  }

  [[nodiscard]] std::size_t input_size(int /*model_idx*/, int input_idx) const noexcept override {
    const auto t = static_cast<std::size_t>(input_idx);
    return (t < inputs_.size()) ? inputs_[t].size() : 0;
  }

  [[nodiscard]] int num_inputs(int /*model_idx*/) const noexcept override {
    return static_cast<int>(inputs_.size());
  }

  [[nodiscard]] std::size_t output_size(int /*model_idx*/, int output_idx) const noexcept override {
    const auto h = static_cast<std::size_t>(output_idx);
    return (h < outputs_.size()) ? outputs_[h].size() : 0;
  }

  [[nodiscard]] int num_outputs(int /*model_idx*/) const noexcept override {
    return static_cast<int>(outputs_.size());
  }

  [[nodiscard]] bool is_initialized() const noexcept override { return initialized_; }

  [[nodiscard]] int num_models() const noexcept override { return 1; }

  /// Overwrite every input buffer with @p v — what an engine's allocation, or a
  /// previous evaluation, might have left there. A filler that is applied on
  /// every pack must overwrite this; one applied once at configure would not.
  void PoisonInputs(float v) {
    for (auto& buf : inputs_) {
      for (auto& x : buf) {
        x = v;
      }
    }
  }

  // ── Knobs ────────────────────────────────────────────────────────────────
  bool stub{false};
  bool run_result{true};
  int run_count{0};
  int init_count{0};
  std::string last_model_path;
  std::vector<std::vector<float>> next_output;
  /// Snapshot of every input tensor as the last Run() saw it. Per tensor and
  /// not concatenated: the whole point of the multi-input path is that the
  /// tensors are separate buffers, and a flattened copy would let a case that
  /// wrote into the wrong one still look right.
  std::vector<std::vector<float>> last_inputs;

 private:
  bool initialized_{false};
  std::vector<std::vector<float>> inputs_;
  std::vector<std::vector<float>> outputs_;
};

}  // namespace integrated_bringup::testfx
