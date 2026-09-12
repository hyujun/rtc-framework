#ifndef RTC_INFERENCE_INFERENCE_ENGINE_HPP_
#define RTC_INFERENCE_INFERENCE_ENGINE_HPP_

#include "rtc_inference/inference_types.hpp"

#include <cstddef>

namespace rtc {

/// Backend-agnostic tensor runner.
///
/// ── TWO INDEPENDENT "MULTI" AXES ───────────────────────────────────────────
/// Do not confuse them:
///   * `model_idx` — several SEPARATE models, each its own ORT session. This is
///     what `udp_hand_driver` uses: one F/T model per fingertip.
///   * `input_idx` / `output_idx` — several tensors of ONE model. This is what
///     a multi-input policy uses.
/// A consumer can use either, both, or neither.
///
/// Every index is bounds-checked and out-of-range returns nullptr / 0 rather
/// than trapping: the RT tick treats a null buffer as "hold", which is the same
/// response it needs for a stub engine, so one branch covers both.
class InferenceEngine {
 public:
  virtual ~InferenceEngine() = default;

  InferenceEngine(const InferenceEngine&) = delete;
  InferenceEngine& operator=(const InferenceEngine&) = delete;
  InferenceEngine(InferenceEngine&&) = delete;
  InferenceEngine& operator=(InferenceEngine&&) = delete;

  /// non-RT: Load model, allocate tensors, warmup
  virtual void Init(const ModelConfig& config) = 0;

  /// Run inference on pre-filled input buffers.
  ///
  /// noexcept and lock-free on this side, but NOT allocation-free: ONNX
  /// Runtime allocates inside every Run(), whatever the binding style. Calling
  /// it on an RT path is a recorded RT-1 exception with conditions
  /// (agent_docs/invariants.md, RT 절) — not a property of this interface.
  [[nodiscard]] virtual bool Run() noexcept = 0;

  /// Access pre-allocated I/O buffers. Out-of-range indices return nullptr.
  ///
  /// `input_idx` / `output_idx` index the ORDER THE CONFIG DECLARED, not the
  /// model's own tensor order. When the config names its tensors the two need
  /// not agree — that indirection is exactly what name-based binding buys.
  virtual float* input_buffer(int model_idx = 0, int input_idx = 0) noexcept = 0;
  [[nodiscard]] virtual const float* output_buffer(int model_idx = 0,
                                                   int output_idx = 0) const noexcept = 0;

  /// Buffer sizes (float element counts). Out-of-range indices return 0.
  [[nodiscard]] virtual std::size_t input_size(int model_idx = 0,
                                               int input_idx = 0) const noexcept = 0;
  [[nodiscard]] virtual std::size_t output_size(int model_idx = 0,
                                                int output_idx = 0) const noexcept = 0;

  /// Tensor counts for a model. Out-of-range model_idx returns 0.
  [[nodiscard]] virtual int num_inputs(int model_idx = 0) const noexcept = 0;
  [[nodiscard]] virtual int num_outputs(int model_idx = 0) const noexcept = 0;

  /// Run multiple models by index in a single batch call. Same allocation
  /// caveat as Run().
  /// Default implementation delegates to RunModel() sequentially.
  [[nodiscard]] virtual bool RunModels(const int* model_indices, int count) noexcept {
    for (int i = 0; i < count; ++i) {
      if (!RunModel(model_indices[i])) {
        return false;
      }
    }
    return true;
  }

  /// Run a single model by index. Same allocation caveat as Run().
  [[nodiscard]] virtual bool RunModel(int /*model_idx*/) noexcept { return Run(); }

  [[nodiscard]] virtual bool is_initialized() const noexcept = 0;
  [[nodiscard]] virtual int num_models() const noexcept = 0;

 protected:
  InferenceEngine() = default;
};

}  // namespace rtc

#endif  // RTC_INFERENCE_INFERENCE_ENGINE_HPP_
