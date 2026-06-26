#ifndef RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_
#define RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_

#include "rtc_inference/inference_engine.hpp"

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_session_options_config_keys.h>
#endif

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rtc {

#ifdef HAS_ONNXRUNTIME

class OnnxEngine : public InferenceEngine {
 public:
  OnnxEngine() = default;
  ~OnnxEngine() override = default;

  OnnxEngine(const OnnxEngine&) = delete;
  OnnxEngine& operator=(const OnnxEngine&) = delete;
  OnnxEngine(OnnxEngine&&) = delete;
  OnnxEngine& operator=(OnnxEngine&&) = delete;

  /// Register one model (single input, N output heads). Call once per model.
  /// Element addresses are stable (models held via unique_ptr), so the tensors
  /// and IoBindings created here keep referencing the right buffers/sessions.
  /// Throws std::runtime_error if the model's actual I/O arity/shape does not
  /// match `config` (non-RT setup path — fail loudly here, not at runtime).
  void Init(const ModelConfig& config) override {
    if (!env_) {
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "rtc_inference");
    }
    if (!run_options_) {
      run_options_ = std::make_unique<Ort::RunOptions>();
    }

    Ort::SessionOptions opts;
    opts.SetExecutionMode(ORT_SEQUENTIAL);  // RT: no inter-op scheduling
    opts.SetIntraOpNumThreads(config.intra_op_threads);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    // RT determinism: stop intra-op worker threads from busy-spinning between
    // runs (otherwise they burn a core and add scheduling jitter on the RT loop).
    opts.AddConfigEntry(kOrtSessionOptionsConfigAllowIntraOpSpinning, "0");
    if (!config.optimized_model_path.empty()) {
      opts.SetOptimizedModelFilePath(config.optimized_model_path.c_str());
    }

    auto model = std::make_unique<Model>(*env_, config.model_path.c_str(), opts);

    // ── Validate model I/O against config (loud failure on drift) ───────────
    // Catches output-head count/shape mismatch from a retrained/wrong .onnx
    // before any buffer is sized to the declared (possibly wrong) shape.
    // NOTE: outputs are bound positionally (head o ↔ config.output_shapes[o]),
    // so shape validation cannot detect a reorder of two heads with identical
    // shapes (e.g. two [1,3] heads swapped) — that is the model's contract.
    const std::size_t n_out = config.output_shapes.size();
    if (model->session.GetInputCount() < 1)
      throw std::runtime_error("rtc_inference: model '" + config.model_path +
                               "' has no input tensor");
    // Exact arity: a model with fewer OR more outputs than declared is a
    // mismatch (extra heads would be silently dropped by positional binding).
    if (model->session.GetOutputCount() != n_out)
      throw std::runtime_error("rtc_inference: model '" + config.model_path + "' exposes " +
                               std::to_string(model->session.GetOutputCount()) +
                               " outputs but config declares " + std::to_string(n_out));
    CheckShape(model->session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
               config.input_shape, "input", 0, config.model_path);
    for (std::size_t o = 0; o < n_out; ++o)
      CheckShape(model->session.GetOutputTypeInfo(o).GetTensorTypeAndShapeInfo().GetShape(),
                 config.output_shapes[o], "output", static_cast<int>(o), config.model_path);

    const auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // ── Input (single) ──────────────────────────────────────────────────────
    const std::size_t in_size = Numel(config.input_shape);
    model->input_size = in_size;
    model->input_buffer.assign(in_size, 0.0F);

    Ort::AllocatorWithDefaultOptions alloc;
    model->input_name = model->session.GetInputNameAllocated(0, alloc).get();
    model->input_tensor =
        Ort::Value::CreateTensor<float>(mem_info, model->input_buffer.data(), in_size,
                                        config.input_shape.data(), config.input_shape.size());

    // ── Outputs (N heads) ───────────────────────────────────────────────────
    model->output_buffers.resize(n_out);
    model->output_sizes.resize(n_out);
    model->output_names.resize(n_out);
    model->output_tensors.reserve(n_out);
    model->output_name_ptrs.resize(n_out);
    for (std::size_t o = 0; o < n_out; ++o) {
      const auto& shape = config.output_shapes[o];
      const std::size_t out_size = Numel(shape);
      model->output_sizes[o] = out_size;
      model->output_buffers[o].assign(out_size, 0.0F);
      model->output_names[o] = model->session.GetOutputNameAllocated(o, alloc).get();
      model->output_tensors.push_back(Ort::Value::CreateTensor<float>(
          mem_info, model->output_buffers[o].data(), out_size, shape.data(), shape.size()));
    }

    // ── IoBinding (input + all outputs) ─────────────────────────────────────
    model->io_binding = std::make_unique<Ort::IoBinding>(model->session);
    model->io_binding->BindInput(model->input_name.c_str(), model->input_tensor);
    for (std::size_t o = 0; o < n_out; ++o) {
      model->io_binding->BindOutput(model->output_names[o].c_str(), model->output_tensors[o]);
      model->output_name_ptrs[o] = model->output_names[o].c_str();  // stable: name owned by model
    }

    // ── Warmup via the exact production path (RunModelDirect) ────────────────
    // Run BEFORE registering so Init is atomic: a warmup throw propagates out
    // without leaving a half-initialized model in models_ (register-or-nothing).
    // Re-uses the same marshalling RunModels() uses, so warmup exercises the
    // real inference path (no separate code that could drift / leave it cold).
    RunModelDirect(*model);  // throws on ORT error → propagates out of Init

    models_.push_back(std::move(model));
    initialized_ = true;
  }

  /// Drop all registered models so Init() can re-register from scratch. Makes
  /// repeated Init cycles idempotent (env_/run_options_ are reused). Non-RT.
  void Reset() noexcept {
    models_.clear();
    initialized_ = false;
  }

  [[nodiscard]] bool Run() noexcept override {
    if (!initialized_)
      return false;
    try {
      for (auto& model : models_)
        RunOneBinding(*model);
      return true;
    } catch (...) {
      return false;
    }
  }

  /// Per-model inference via IoBinding (backward compatible).
  [[nodiscard]] bool RunModel(int model_idx) noexcept override {
    if (!initialized_ || model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return false;
    try {
      RunOneBinding(*models_[static_cast<std::size_t>(model_idx)]);
      return true;
    } catch (...) {
      return false;
    }
  }

  /// RT-optimized batch run: direct Session::Run bypassing IoBinding overhead.
  /// Pre-allocated RunOptions reused across calls. SynchronizeInputs/Outputs
  /// skipped (CPU-only, no device transfer needed).
  [[nodiscard]] bool RunModels(const int* model_indices, int count) noexcept override {
    if (!initialized_ || count <= 0)
      return false;
    try {
      for (int i = 0; i < count; ++i) {
        const int idx = model_indices[i];
        if (idx < 0 || static_cast<std::size_t>(idx) >= models_.size())
          return false;
        RunModelDirect(*models_[static_cast<std::size_t>(idx)]);
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  float* input_buffer(int model_idx = 0) noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return nullptr;
    return models_[static_cast<std::size_t>(model_idx)]->input_buffer.data();
  }

  const float* output_buffer(int model_idx = 0, int output_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return nullptr;
    const auto& outs = models_[static_cast<std::size_t>(model_idx)]->output_buffers;
    if (output_idx < 0 || static_cast<std::size_t>(output_idx) >= outs.size())
      return nullptr;
    return outs[static_cast<std::size_t>(output_idx)].data();
  }

  [[nodiscard]] std::size_t input_size(int model_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return 0;
    return models_[static_cast<std::size_t>(model_idx)]->input_size;
  }

  [[nodiscard]] std::size_t output_size(int model_idx = 0,
                                        int output_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return 0;
    const auto& sizes = models_[static_cast<std::size_t>(model_idx)]->output_sizes;
    if (output_idx < 0 || static_cast<std::size_t>(output_idx) >= sizes.size())
      return 0;
    return sizes[static_cast<std::size_t>(output_idx)];
  }

  [[nodiscard]] int num_outputs(int model_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return 0;
    return static_cast<int>(models_[static_cast<std::size_t>(model_idx)]->output_sizes.size());
  }

  [[nodiscard]] bool is_initialized() const noexcept override { return initialized_; }

  [[nodiscard]] int num_models() const noexcept override {
    return static_cast<int>(models_.size());
  }

 private:
  // One registered model. Held via unique_ptr so its address (and therefore the
  // tensors referencing its buffers and the IoBinding referencing its session)
  // stays stable across models_ growth.
  struct Model {
    Model(Ort::Env& env, const char* path, const Ort::SessionOptions& opts)
        : session(env, path, opts) {}

    Ort::Session session;
    std::unique_ptr<Ort::IoBinding> io_binding;
    std::vector<float> input_buffer;
    std::size_t input_size{0};
    std::string input_name;
    Ort::Value input_tensor{nullptr};
    std::vector<std::vector<float>> output_buffers;
    std::vector<std::size_t> output_sizes;
    std::vector<std::string> output_names;
    std::vector<const char*> output_name_ptrs;  // pre-built for RT Session::Run
    std::vector<Ort::Value> output_tensors;
  };

  // Flat element count of a tensor shape (used to size input + each output buffer).
  static std::size_t Numel(const std::vector<int64_t>& shape) noexcept {
    std::size_t n = 1;
    for (auto d : shape)
      n *= static_cast<std::size_t>(d);
    return n;
  }

  // Throw if the model's actual tensor shape is incompatible with the declared
  // one. The declared shape sizes a real buffer, so every declared dim must be
  // static positive (a dynamic -1 would make Numel() cast to SIZE_MAX and try a
  // catastrophic allocation). A model dim < 0 is dynamic (e.g. dynamic batch/
  // seq) and matches any declared size; static model dims must match exactly.
  // Non-RT (Init only).
  static void CheckShape(const std::vector<int64_t>& model_shape,
                         const std::vector<int64_t>& declared, const char* what, int idx,
                         const std::string& model_path) {
    for (auto d : declared)
      if (d <= 0)
        throw std::runtime_error(
            "rtc_inference: model '" + model_path + "' " + what + "[" + std::to_string(idx) +
            "] declared shape must be static positive (got " + std::to_string(d) + ")");
    bool ok = model_shape.size() == declared.size();
    for (std::size_t i = 0; ok && i < declared.size(); ++i)
      ok = (model_shape[i] < 0) || (model_shape[i] == declared[i]);
    if (!ok)
      throw std::runtime_error("rtc_inference: model '" + model_path + "' " + what + "[" +
                               std::to_string(idx) + "] shape mismatch vs config");
  }

  // IoBinding inference for one model (SynchronizeInputs → Run → Synchronize
  // Outputs), shared by Run() and RunModel(). Single-sources the sync triplet.
  void RunOneBinding(Model& m) {
    m.io_binding->SynchronizeInputs();
    m.session.Run(*run_options_, *m.io_binding);
    m.io_binding->SynchronizeOutputs();
  }

  // Direct Session::Run marshalling shared by RunModels() (RT, wrapped in
  // try/catch) and Init() warmup (non-RT, allowed to throw). RT-safe: only a
  // stack name array, no heap allocation.
  void RunModelDirect(Model& m) {
    const char* in_names[] = {m.input_name.c_str()};
    m.session.Run(*run_options_, in_names, &m.input_tensor, 1, m.output_name_ptrs.data(),
                  m.output_tensors.data(), m.output_tensors.size());
  }

  bool initialized_{false};
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::RunOptions> run_options_;
  std::vector<std::unique_ptr<Model>> models_;
};

#else  // !HAS_ONNXRUNTIME

// Stub implementation when ONNX Runtime is not available
class OnnxEngine : public InferenceEngine {
 public:
  void Init(const ModelConfig&) override { /* no-op */ }

  void Reset() noexcept { /* no-op */ }

  [[nodiscard]] bool Run() noexcept override { return false; }

  float* input_buffer(int = 0) noexcept override { return nullptr; }

  const float* output_buffer(int = 0, int = 0) const noexcept override { return nullptr; }

  [[nodiscard]] std::size_t input_size(int = 0) const noexcept override { return 0; }

  [[nodiscard]] std::size_t output_size(int = 0, int = 0) const noexcept override { return 0; }

  [[nodiscard]] int num_outputs(int = 0) const noexcept override { return 0; }

  [[nodiscard]] bool is_initialized() const noexcept override { return false; }

  [[nodiscard]] int num_models() const noexcept override { return 0; }
};

#endif  // HAS_ONNXRUNTIME

}  // namespace rtc

#endif  // RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_
