#ifndef RTC_INFERENCE_INFERENCE_TYPES_HPP_
#define RTC_INFERENCE_INFERENCE_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rtc {

/// One tensor, as either a model or a config describes it.
///
/// `name` is how the tensor is BOUND when the config supplies one — see
/// ModelConfig below for why that is not merely cosmetic. A model-side name
/// comes from ORT; a config-side name is whatever the operator wrote, and may
/// be left empty to fall back to positional binding.
struct TensorSpec {
  std::string name;
  std::vector<std::int64_t> shape;
};

/// What a consumer declares about the model it is about to load.
///
/// ── WHY TENSORS CARRY NAMES ────────────────────────────────────────────────
/// Binding tensors by POSITION cannot distinguish two tensors of the same
/// shape. That is not a hypothetical: an LSTM policy's `h_out`/`c_out` are the
/// same shape by definition, and `udp_hand_driver`'s fingertip F/T model
/// declares {{1,1},{1,3},{1,3}} where force and direction are both [1,3]. If
/// such a model is exported with those two in the other order, every shape
/// check passes and the values are silently swapped.
///
/// So each side may be declared EITHER fully named (bound by name, which
/// detects the swap) or fully unnamed (bound positionally, preserving the
/// original behaviour for a consumer that does not know its model's names).
/// A partially-named side is REFUSED rather than guessed at: it is the one
/// case where the intended binding is genuinely ambiguous.
struct ModelConfig {
  std::string model_path;
  std::string optimized_model_path;  ///< ORT graph-optimized cache path (empty = disabled)

  /// Input tensors, in the order the consumer will index them via
  /// `InferenceEngine::input_buffer(model_idx, input_idx)`. That index is the
  /// position in THIS list, not the model's own input order — when names are
  /// used the two need not agree, which is the whole point.
  std::vector<TensorSpec> inputs;

  /// Output heads, indexed the same way by `output_buffer(model_idx, head)`.
  std::vector<TensorSpec> outputs;

  int intra_op_threads{1};  ///< RT: single-threaded inference
};

}  // namespace rtc

#endif  // RTC_INFERENCE_INFERENCE_TYPES_HPP_
