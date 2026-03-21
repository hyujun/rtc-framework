#ifndef RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_
#define RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_

#include "rtc_inference/inference_engine.hpp"

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

#include <memory>
#include <string>
#include <vector>

namespace rtc {

#ifdef HAS_ONNXRUNTIME

class OnnxEngine : public InferenceEngine {
 public:
  OnnxEngine() = default;
  ~OnnxEngine() override = default;

  void Init(const ModelConfig& config) override {
    configs_.push_back(config);
    const int idx = static_cast<int>(configs_.size()) - 1;

    if (idx == 0) {
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "rtc_inference");
    }

    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(config.intra_op_threads);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    sessions_.emplace_back(*env_, config.model_path.c_str(), opts);
    auto& session = sessions_.back();

    // Compute buffer sizes
    std::size_t in_size = 1;
    for (auto d : config.input_shape) in_size *= static_cast<std::size_t>(d);
    std::size_t out_size = 1;
    for (auto d : config.output_shape) out_size *= static_cast<std::size_t>(d);

    input_sizes_.push_back(in_size);
    output_sizes_.push_back(out_size);

    // Pre-allocate buffers
    input_buffers_.emplace_back(in_size, 0.0f);
    output_buffers_.emplace_back(out_size, 0.0f);

    // Create IoBinding
    io_bindings_.emplace_back(session);
    auto& binding = io_bindings_.back();

    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    auto input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, input_buffers_.back().data(), in_size,
        config.input_shape.data(), config.input_shape.size());
    auto output_tensor = Ort::Value::CreateTensor<float>(
        mem_info, output_buffers_.back().data(), out_size,
        config.output_shape.data(), config.output_shape.size());

    // Get input/output names
    Ort::AllocatorWithDefaultOptions alloc;
    auto in_name = session.GetInputNameAllocated(0, alloc);
    auto out_name = session.GetOutputNameAllocated(0, alloc);
    input_names_.emplace_back(in_name.get());
    output_names_.emplace_back(out_name.get());

    binding.BindInput(input_names_.back().c_str(), input_tensor);
    binding.BindOutput(output_names_.back().c_str(), output_tensor);

    // Store tensors to keep them alive
    input_tensors_.push_back(std::move(input_tensor));
    output_tensors_.push_back(std::move(output_tensor));

    // Warmup
    session.Run(Ort::RunOptions{nullptr},
                input_names_.back().c_str(), &input_tensors_.back(), 1,
                output_names_.back().c_str(), &output_tensors_.back(), 1);

    initialized_ = true;
  }

  [[nodiscard]] bool Run() noexcept override {
    if (!initialized_) return false;
    try {
      for (std::size_t i = 0; i < sessions_.size(); ++i) {
        io_bindings_[i].SynchronizeInputs();
        sessions_[i].Run(Ort::RunOptions{nullptr}, io_bindings_[i]);
        io_bindings_[i].SynchronizeOutputs();
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  // Also provide per-model Run
  [[nodiscard]] bool RunModel(int model_idx) noexcept {
    if (!initialized_ || model_idx < 0 ||
        static_cast<std::size_t>(model_idx) >= sessions_.size()) return false;
    try {
      io_bindings_[static_cast<std::size_t>(model_idx)].SynchronizeInputs();
      sessions_[static_cast<std::size_t>(model_idx)].Run(
          Ort::RunOptions{nullptr},
          io_bindings_[static_cast<std::size_t>(model_idx)]);
      io_bindings_[static_cast<std::size_t>(model_idx)].SynchronizeOutputs();
      return true;
    } catch (...) {
      return false;
    }
  }

  float* input_buffer(int model_idx = 0) noexcept override {
    return input_buffers_[static_cast<std::size_t>(model_idx)].data();
  }

  const float* output_buffer(int model_idx = 0) const noexcept override {
    return output_buffers_[static_cast<std::size_t>(model_idx)].data();
  }

  [[nodiscard]] std::size_t input_size(int model_idx = 0) const noexcept override {
    return input_sizes_[static_cast<std::size_t>(model_idx)];
  }

  [[nodiscard]] std::size_t output_size(int model_idx = 0) const noexcept override {
    return output_sizes_[static_cast<std::size_t>(model_idx)];
  }

  [[nodiscard]] bool is_initialized() const noexcept override { return initialized_; }
  [[nodiscard]] int num_models() const noexcept override {
    return static_cast<int>(sessions_.size());
  }

 private:
  bool initialized_{false};
  std::unique_ptr<Ort::Env> env_;
  std::vector<ModelConfig> configs_;
  std::vector<Ort::Session> sessions_;
  std::vector<Ort::IoBinding> io_bindings_;
  std::vector<Ort::Value> input_tensors_;
  std::vector<Ort::Value> output_tensors_;
  std::vector<std::vector<float>> input_buffers_;
  std::vector<std::vector<float>> output_buffers_;
  std::vector<std::size_t> input_sizes_;
  std::vector<std::size_t> output_sizes_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
};

#else  // !HAS_ONNXRUNTIME

// Stub implementation when ONNX Runtime is not available
class OnnxEngine : public InferenceEngine {
 public:
  void Init(const ModelConfig&) override { /* no-op */ }
  [[nodiscard]] bool Run() noexcept override { return false; }
  float* input_buffer(int = 0) noexcept override { return nullptr; }
  const float* output_buffer(int = 0) const noexcept override { return nullptr; }
  [[nodiscard]] std::size_t input_size(int = 0) const noexcept override { return 0; }
  [[nodiscard]] std::size_t output_size(int = 0) const noexcept override { return 0; }
  [[nodiscard]] bool is_initialized() const noexcept override { return false; }
  [[nodiscard]] int num_models() const noexcept override { return 0; }
};

#endif  // HAS_ONNXRUNTIME

}  // namespace rtc

#endif  // RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_
