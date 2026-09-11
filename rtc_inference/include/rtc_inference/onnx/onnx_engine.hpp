#ifndef RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_
#define RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_

#include "rtc_inference/inference_engine.hpp"
#include "rtc_inference/shape_report.hpp"

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
    // Both sides are collected and compared in ONE pass so that a retrained
    // .onnx which moved several tensors reports all of them at once. Throwing
    // on the first mismatch — what this replaces — costs one whole bring-up
    // cycle per tensor to discover the rest.
    //
    // The comparison and the table live in shape_report.hpp and know nothing
    // about ORT. That buys exhaustive rule coverage cheaply (a pure function
    // over two lists of shapes); the end-to-end half — that those rules are
    // actually WIRED to a session — is covered by test_onnx_engine.cpp against
    // a committed .onnx fixture.
    Ort::AllocatorWithDefaultOptions alloc;
    std::vector<TensorSpec> model_inputs;
    std::vector<TensorSpec> model_outputs;
    model_inputs.reserve(model->session.GetInputCount());
    model_outputs.reserve(model->session.GetOutputCount());
    for (std::size_t i = 0; i < model->session.GetInputCount(); ++i)
      model_inputs.push_back(
          {model->session.GetInputNameAllocated(i, alloc).get(),
           model->session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape()});
    for (std::size_t o = 0; o < model->session.GetOutputCount(); ++o)
      model_outputs.push_back(
          {model->session.GetOutputNameAllocated(o, alloc).get(),
           model->session.GetOutputTypeInfo(o).GetTensorTypeAndShapeInfo().GetShape()});

    // Exact arity in BOTH directions falls out of the report: a tensor present
    // on only one side gets a row, and any such row fails it. The input side
    // had no arity check at all before — a multi-input model reached the warmup
    // below and died there with an ORT-internal message naming neither count.
    const auto report = CompareModelIo(model_inputs, config.inputs, model_outputs, config.outputs);
    if (!report.Ok())
      throw std::runtime_error(report.Format(config.model_path));

    const auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const std::size_t n_in = config.inputs.size();
    const std::size_t n_out = config.outputs.size();

    // ── Buffers, in DECLARED order ──────────────────────────────────────────
    // Slot i is `config.inputs[i]`, which is what the consumer will pass to
    // input_buffer(model_idx, i). When the config names its tensors that need
    // not be the model's own order, and keeping the consumer's order here is
    // what makes the indirection invisible to the tick.
    //
    // The name ORT is asked to bind slot i to:
    //
    //   named config    → the declared name. No lookup is needed to find the
    //                     model's tensor: a passing report has already proven
    //                     the model exports exactly this name, so the declared
    //                     spelling IS the model's. THIS is where the reordering
    //                     happens — slot i asks ORT for its own tensor, not for
    //                     the model's i-th one.
    //   unnamed config  → the model's i-th name, i.e. positional binding.
    //
    // A passing report is load-bearing here, not decorative: it is what makes
    // the indexing below safe without a second search.
    const auto bind_name = [](const std::vector<TensorSpec>& from_model, const TensorSpec& declared,
                              std::size_t slot) {
      return declared.name.empty() ? from_model[slot].name : declared.name;
    };

    model->input_buffers.resize(n_in);
    model->input_sizes.resize(n_in);
    model->input_names.resize(n_in);
    model->input_name_ptrs.resize(n_in);
    model->input_tensors.reserve(n_in);
    for (std::size_t i = 0; i < n_in; ++i) {
      const auto& shape = config.inputs[i].shape;
      const std::size_t in_size = Numel(shape);
      model->input_sizes[i] = in_size;
      model->input_buffers[i].assign(in_size, 0.0F);
      model->input_names[i] = bind_name(model_inputs, config.inputs[i], i);
      model->input_tensors.push_back(Ort::Value::CreateTensor<float>(
          mem_info, model->input_buffers[i].data(), in_size, shape.data(), shape.size()));
    }

    model->output_buffers.resize(n_out);
    model->output_sizes.resize(n_out);
    model->output_names.resize(n_out);
    model->output_name_ptrs.resize(n_out);
    model->output_tensors.reserve(n_out);
    for (std::size_t o = 0; o < n_out; ++o) {
      const auto& shape = config.outputs[o].shape;
      const std::size_t out_size = Numel(shape);
      model->output_sizes[o] = out_size;
      model->output_buffers[o].assign(out_size, 0.0F);
      model->output_names[o] = bind_name(model_outputs, config.outputs[o], o);
      model->output_tensors.push_back(Ort::Value::CreateTensor<float>(
          mem_info, model->output_buffers[o].data(), out_size, shape.data(), shape.size()));
    }

    // ── IoBinding (all inputs + all outputs) ────────────────────────────────
    // The name pointers are cached here and not rebuilt per Run: they are what
    // the RT path hands to Session::Run, and they stay valid because the
    // strings are owned by this Model, whose address is stable (unique_ptr).
    model->io_binding = std::make_unique<Ort::IoBinding>(model->session);
    for (std::size_t i = 0; i < n_in; ++i) {
      model->io_binding->BindInput(model->input_names[i].c_str(), model->input_tensors[i]);
      model->input_name_ptrs[i] = model->input_names[i].c_str();
    }
    for (std::size_t o = 0; o < n_out; ++o) {
      model->io_binding->BindOutput(model->output_names[o].c_str(), model->output_tensors[o]);
      model->output_name_ptrs[o] = model->output_names[o].c_str();
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

  float* input_buffer(int model_idx = 0, int input_idx = 0) noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return nullptr;
    auto& ins = models_[static_cast<std::size_t>(model_idx)]->input_buffers;
    if (input_idx < 0 || static_cast<std::size_t>(input_idx) >= ins.size())
      return nullptr;
    return ins[static_cast<std::size_t>(input_idx)].data();
  }

  const float* output_buffer(int model_idx = 0, int output_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return nullptr;
    const auto& outs = models_[static_cast<std::size_t>(model_idx)]->output_buffers;
    if (output_idx < 0 || static_cast<std::size_t>(output_idx) >= outs.size())
      return nullptr;
    return outs[static_cast<std::size_t>(output_idx)].data();
  }

  [[nodiscard]] std::size_t input_size(int model_idx = 0,
                                       int input_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return 0;
    const auto& sizes = models_[static_cast<std::size_t>(model_idx)]->input_sizes;
    if (input_idx < 0 || static_cast<std::size_t>(input_idx) >= sizes.size())
      return 0;
    return sizes[static_cast<std::size_t>(input_idx)];
  }

  [[nodiscard]] int num_inputs(int model_idx = 0) const noexcept override {
    if (model_idx < 0 || static_cast<std::size_t>(model_idx) >= models_.size())
      return 0;
    return static_cast<int>(models_[static_cast<std::size_t>(model_idx)]->input_sizes.size());
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
    // Inputs and outputs are held symmetrically: every slot below is indexed by
    // the position in ModelConfig::inputs / ::outputs, not by the model's own
    // tensor order. `*_name_ptrs` are pre-built so the RT Session::Run path
    // allocates nothing.
    std::vector<std::vector<float>> input_buffers;
    std::vector<std::size_t> input_sizes;
    std::vector<std::string> input_names;
    std::vector<const char*> input_name_ptrs;
    std::vector<Ort::Value> input_tensors;
    std::vector<std::vector<float>> output_buffers;
    std::vector<std::size_t> output_sizes;
    std::vector<std::string> output_names;
    std::vector<const char*> output_name_ptrs;
    std::vector<Ort::Value> output_tensors;
  };

  // Flat element count of a tensor shape (used to size input + each output buffer).
  static std::size_t Numel(const std::vector<int64_t>& shape) noexcept {
    std::size_t n = 1;
    for (auto d : shape)
      n *= static_cast<std::size_t>(d);
    return n;
  }

  // (Shape compatibility lives in shape_report.hpp, as a pure function over two
  // lists of shapes. Keeping it out of here is what lets every rule be driven
  // exhaustively without a session; do not inline it back.)

  // IoBinding inference for one model (SynchronizeInputs → Run → Synchronize
  // Outputs), shared by Run() and RunModel(). Single-sources the sync triplet.
  void RunOneBinding(Model& m) {
    m.io_binding->SynchronizeInputs();
    m.session.Run(*run_options_, *m.io_binding);
    m.io_binding->SynchronizeOutputs();
  }

  // Direct Session::Run marshalling shared by RunModels() (RT, wrapped in
  // try/catch) and Init() warmup (non-RT, allowed to throw). This wrapper adds
  // no allocation — the name and value arrays are prebuilt — but Session::Run
  // itself allocates on every call (see InferenceEngine::Run()); measured by
  // integrated_bringup's test_demo_inference_real_model.
  void RunModelDirect(Model& m) {
    m.session.Run(*run_options_, m.input_name_ptrs.data(), m.input_tensors.data(),
                  m.input_tensors.size(), m.output_name_ptrs.data(), m.output_tensors.data(),
                  m.output_tensors.size());
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

  float* input_buffer(int = 0, int = 0) noexcept override { return nullptr; }

  [[nodiscard]] const float* output_buffer(int = 0, int = 0) const noexcept override {
    return nullptr;
  }

  [[nodiscard]] std::size_t input_size(int = 0, int = 0) const noexcept override { return 0; }

  [[nodiscard]] std::size_t output_size(int = 0, int = 0) const noexcept override { return 0; }

  [[nodiscard]] int num_inputs(int = 0) const noexcept override { return 0; }

  [[nodiscard]] int num_outputs(int = 0) const noexcept override { return 0; }

  [[nodiscard]] bool is_initialized() const noexcept override { return false; }

  [[nodiscard]] int num_models() const noexcept override { return 0; }
};

#endif  // HAS_ONNXRUNTIME

}  // namespace rtc

#endif  // RTC_INFERENCE_ONNX_ONNX_ENGINE_HPP_
