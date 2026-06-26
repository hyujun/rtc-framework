#ifndef RTC_INFERENCE_INFERENCE_TYPES_HPP_
#define RTC_INFERENCE_INFERENCE_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rtc {

struct ModelConfig {
  std::string model_path;
  std::string optimized_model_path;  // ORT graph-optimized cache path (empty = disabled)
  std::vector<int64_t> input_shape;  // single input, e.g., {1, 12, 16}
  // One shape per output head, e.g., {{1, 1}, {1, 3}, {1, 3}} for a 3-head model.
  // A single-output model is just {{1, 1, 13}}.
  std::vector<std::vector<int64_t>> output_shapes;
  int intra_op_threads{1};  // RT: single-threaded inference
};

}  // namespace rtc

#endif  // RTC_INFERENCE_INFERENCE_TYPES_HPP_
