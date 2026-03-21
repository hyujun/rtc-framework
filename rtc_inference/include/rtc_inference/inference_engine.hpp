#ifndef RTC_INFERENCE_INFERENCE_ENGINE_HPP_
#define RTC_INFERENCE_INFERENCE_ENGINE_HPP_

#include "rtc_inference/inference_types.hpp"

namespace rtc {

class InferenceEngine {
 public:
  virtual ~InferenceEngine() = default;

  InferenceEngine(const InferenceEngine&) = delete;
  InferenceEngine& operator=(const InferenceEngine&) = delete;
  InferenceEngine(InferenceEngine&&) = delete;
  InferenceEngine& operator=(InferenceEngine&&) = delete;

  /// non-RT: Load model, allocate tensors, warmup
  virtual void Init(const ModelConfig& config) = 0;

  /// RT-safe: Run inference on pre-filled input buffer
  [[nodiscard]] virtual bool Run() noexcept = 0;

  /// Access pre-allocated I/O buffers
  virtual float* input_buffer(int model_idx = 0) noexcept = 0;
  virtual const float* output_buffer(int model_idx = 0) const noexcept = 0;

  /// Buffer sizes
  [[nodiscard]] virtual std::size_t input_size(int model_idx = 0) const noexcept = 0;
  [[nodiscard]] virtual std::size_t output_size(int model_idx = 0) const noexcept = 0;

  /// RT-safe: Run multiple models by index in a single batch call.
  /// Default implementation delegates to RunModel() sequentially.
  [[nodiscard]] virtual bool RunModels(const int* model_indices,
                                       int count) noexcept {
    for (int i = 0; i < count; ++i) {
      if (!RunModel(model_indices[i])) return false;
    }
    return true;
  }

  /// RT-safe: Run a single model by index.
  [[nodiscard]] virtual bool RunModel(int /*model_idx*/) noexcept {
    return Run();
  }

  [[nodiscard]] virtual bool is_initialized() const noexcept = 0;
  [[nodiscard]] virtual int num_models() const noexcept = 0;

 protected:
  InferenceEngine() = default;
};

}  // namespace rtc

#endif  // RTC_INFERENCE_INFERENCE_ENGINE_HPP_
