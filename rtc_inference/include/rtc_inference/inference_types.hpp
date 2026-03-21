#ifndef RTC_INFERENCE_INFERENCE_TYPES_HPP_
#define RTC_INFERENCE_INFERENCE_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rtc {

struct ModelConfig {
  std::string model_path;
  std::string optimized_model_path;   // ORT graph-optimized cache path (empty = disabled)
  std::vector<int64_t> input_shape;   // e.g., {1, 12, 16}
  std::vector<int64_t> output_shape;  // e.g., {1, 1, 13}
  int intra_op_threads{1};            // RT: single-threaded inference
};

}  // namespace rtc

#endif  // RTC_INFERENCE_INFERENCE_TYPES_HPP_
