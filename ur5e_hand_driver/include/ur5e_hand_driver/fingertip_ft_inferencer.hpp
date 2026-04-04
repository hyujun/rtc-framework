#ifndef UR5E_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_
#define UR5E_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_

// Per-fingertip ONNX Runtime 기반 Force/Torque 추론기.
//
// 각 fingertip마다 개별 ONNX 모델을 로드하여 barometer + delta → 접촉/힘 추론.
//   Input:  float32[1, H, 16]  (H=history_length, barometer 8ch + barometer_delta 8ch, 정규화됨)
//   Outputs (3 heads):
//     output0: float32[1, 1]  (contact logit → sigmoid → probability)
//     output1: float32[1, 3]  (F: force vector)
//     output2: float32[1, 3]  (u: direction vector, filtered when no contact)
//
// RT safety:
//   - InitFT()에서 모든 동적 할당 수행 (non-RT 컨텍스트)
//   - Infer()/FeedCalibration()는 noexcept + allocation-free
//   - 사전 할당된 I/O 버퍼 + Ort::IoBinding으로 zero-alloc 추론
//
// Baseline Offset Calibration:
//   - InitFT() 이후 FeedCalibration()으로 센서 baseline 자동 측정
//   - 정규화 공식: (raw - baseline_offset) / input_max → [-1, +1]

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "rtc_base/types/types.hpp"

namespace rtc {

#ifndef HAS_ONNXRUNTIME

/// Stub implementation when ONNX Runtime is not available.
class FingertipFTInferencer {
 public:
  struct Config {
    bool enabled{false};
    int  num_fingertips{kDefaultNumFingertips};
    int  history_length{kFTHistoryLength};
    std::vector<std::string> model_paths;
    std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_max{};
    bool calibration_enabled{true};
    int  calibration_samples{500};
  };

  void InitFT(const Config& /*config*/) {
    RCLCPP_ERROR(rclcpp::get_logger("FT-Inferencer"),
                 "STUB: HAS_ONNXRUNTIME not defined! "
                 "ONNX Runtime unavailable — FT inference disabled.");
  }
  [[nodiscard]] bool FeedCalibration(
      const std::array<int32_t, kMaxHandSensors>& /*sensor_data*/,
      int /*num_fingertips*/) noexcept { return true; }
  [[nodiscard]] FingertipFTState Infer(
      const std::array<int32_t, kMaxHandSensors>& /*sensor_data*/,
      int /*num_fingertips*/) noexcept { return {}; }
  [[nodiscard]] bool is_initialized() const noexcept { return false; }
  [[nodiscard]] bool is_calibrated() const noexcept { return false; }
  [[nodiscard]] int  num_models() const noexcept { return 0; }
  [[nodiscard]] int  calibration_count() const noexcept { return 0; }
  [[nodiscard]] int  calibration_target() const noexcept { return 0; }
  [[nodiscard]] std::array<std::array<float, kBarometerCount>, kMaxFingertips>
      baseline_offset() const noexcept { return {}; }
};

#else  // HAS_ONNXRUNTIME

class FingertipFTInferencer {
 public:
  // ── Configuration ─────────────────────────────────────────────────────────
  struct Config {
    bool enabled{false};
    int  num_fingertips{kDefaultNumFingertips};
    int  history_length{kFTHistoryLength};      // FIFO history rows (default: 12)

    // Per-fingertip ONNX 모델 경로 (빈 문자열 → 해당 finger 비활성)
    std::vector<std::string> model_paths;

    // Per-fingertip 정규화 파라미터: input_max [fingertip][input_channel]
    // input_channel: baro[0..7] + delta[0..7] = 16
    // 정규화 공식: value / input_max → [-1, +1]
    std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_max{};

    // Baseline Offset Calibration
    bool calibration_enabled{true};
    int  calibration_samples{500};    // sensor cycle 기준 (500Hz/decimation)
  };

  FingertipFTInferencer() = default;
  ~FingertipFTInferencer() = default;

  FingertipFTInferencer(const FingertipFTInferencer&) = delete;
  FingertipFTInferencer& operator=(const FingertipFTInferencer&) = delete;
  FingertipFTInferencer(FingertipFTInferencer&&) = delete;
  FingertipFTInferencer& operator=(FingertipFTInferencer&&) = delete;

  // ── Lifecycle (non-RT) ────────────────────────────────────────────────────

  /// 모델 로드, Ort::Session 생성, 텐서 사전 할당, warmup 실행.
  /// non-RT 컨텍스트에서만 호출. 실패 시 예외.
  void InitFT(const Config& config) {
    config_ = config;
    const int n = std::min(config_.num_fingertips, kMaxFingertips);
    num_active_ = 0;

    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "InitFT: num_fingertips=%d, history_length=%d, model_paths.size=%zu, calibration=%s(%d samples)",
                n, config_.history_length, config_.model_paths.size(),
                config_.calibration_enabled ? "ON" : "OFF",
                config_.calibration_samples);

    // Ort::SessionOptions (모든 모델 공유)
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options.EnableCpuMemArena();

    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    for (int f = 0; f < n; ++f) {
      auto& model = models_[static_cast<std::size_t>(f)];

      // 모델 경로 없으면 해당 finger 비활성
      if (f >= static_cast<int>(config_.model_paths.size()) ||
          config_.model_paths[static_cast<std::size_t>(f)].empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FT-Inferencer"),
                    "finger[%d]: SKIPPED (empty path)", f);
        model.valid = false;
        continue;
      }

      const auto& path = config_.model_paths[static_cast<std::size_t>(f)];
      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "finger[%d]: loading \"%s\"", f, path.c_str());

      // Session 생성
      model.session = std::make_unique<Ort::Session>(
          env_, path.c_str(), session_options);

      // Input/Output 이름 쿼리 (3 output heads)
#if ORT_API_VERSION >= 13
      auto input_name_alloc = model.session->GetInputNameAllocated(0, allocator_);
      auto output0_name_alloc = model.session->GetOutputNameAllocated(0, allocator_);
      auto output1_name_alloc = model.session->GetOutputNameAllocated(1, allocator_);
      auto output2_name_alloc = model.session->GetOutputNameAllocated(2, allocator_);
      model.input_name = input_name_alloc.get();
      model.output0_name = output0_name_alloc.get();
      model.output1_name = output1_name_alloc.get();
      model.output2_name = output2_name_alloc.get();
#else
      {
        char* in_name = model.session->GetInputName(0, allocator_);
        char* out0_name = model.session->GetOutputName(0, allocator_);
        char* out1_name = model.session->GetOutputName(1, allocator_);
        char* out2_name = model.session->GetOutputName(2, allocator_);
        model.input_name = in_name;
        model.output0_name = out0_name;
        model.output1_name = out1_name;
        model.output2_name = out2_name;
        allocator_.Free(in_name);
        allocator_.Free(out0_name);
        allocator_.Free(out1_name);
        allocator_.Free(out2_name);
      }
#endif
      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "finger[%d]: input='%s', outputs=['%s','%s','%s']",
                  f, model.input_name.c_str(),
                  model.output0_name.c_str(),
                  model.output1_name.c_str(),
                  model.output2_name.c_str());

      // 사전 할당된 버퍼 위에 Ort::Value 텐서 생성
      const int64_t H = static_cast<int64_t>(config_.history_length);
      const int64_t input_shape[] = {1, H, kFTInputSize};             // [1, 12, 16]
      constexpr int64_t output0_shape[] = {1, 1};                     // [1, 1] contact logit
      constexpr int64_t output1_shape[] = {1, 3};                     // [1, 3] F
      constexpr int64_t output2_shape[] = {1, 3};                     // [1, 3] u

      model.input_tensor = Ort::Value::CreateTensor<float>(
          memory_info_, model.input_buffer.data(),
          static_cast<std::size_t>(config_.history_length) * kFTInputSize,
          input_shape, 3);

      model.output0_tensor = Ort::Value::CreateTensor<float>(
          memory_info_, model.output0_buffer.data(), 1, output0_shape, 2);
      model.output1_tensor = Ort::Value::CreateTensor<float>(
          memory_info_, model.output1_buffer.data(), 3, output1_shape, 2);
      model.output2_tensor = Ort::Value::CreateTensor<float>(
          memory_info_, model.output2_buffer.data(), 3, output2_shape, 2);

      model.history_count = 0;

      // IoBinding 생성 + 바인딩 (1 input, 3 outputs)
      model.io_binding = std::make_unique<Ort::IoBinding>(*model.session);
      model.io_binding->BindInput(model.input_name.c_str(), model.input_tensor);
      model.io_binding->BindOutput(model.output0_name.c_str(), model.output0_tensor);
      model.io_binding->BindOutput(model.output1_name.c_str(), model.output1_tensor);
      model.io_binding->BindOutput(model.output2_name.c_str(), model.output2_tensor);

      model.valid = true;
      ++num_active_;
      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "finger[%d]: loaded OK (input=%s, outputs=[%s,%s,%s])",
                  f, model.input_name.c_str(),
                  model.output0_name.c_str(),
                  model.output1_name.c_str(),
                  model.output2_name.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "num_active=%d / %d", num_active_, n);

    // prev_barometer 초기화 (delta 계산용)
    for (auto& p : prev_barometer_) p.fill(0.0f);

    // Calibration 초기화
    if (config_.calibration_enabled) {
      for (auto& s : calibration_sum_) s.fill(0.0);
      calibration_count_ = 0;
      calibrated_ = false;
    } else {
      for (auto& b : baseline_offset_) b.fill(0.0f);
      calibrated_ = true;
    }

    // input_max 역수 사전 계산 (Infer()에서 div → mul 최적화)
    for (int f = 0; f < n; ++f) {
      const auto fi = static_cast<std::size_t>(f);
      for (int ch = 0; ch < kFTInputSize; ++ch) {
        const auto ci = static_cast<std::size_t>(ch);
        const float m = config_.input_max[fi][ci];
        input_max_reciprocal_[fi][ci] = (m == 0.0f) ? 1.0f : (1.0f / m);
      }
    }

    // Warmup: 각 모델에 더미 추론 1회 (JIT 오버헤드 제거)
    for (int f = 0; f < n; ++f) {
      auto& model = models_[static_cast<std::size_t>(f)];
      if (!model.valid) continue;
      model.input_buffer.fill(0.0f);
      model.session->Run(Ort::RunOptions{nullptr}, *model.io_binding);
    }

    initialized_ = (num_active_ > 0);
    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "InitFT done: initialized=%d, calibrated=%d",
                initialized_ ? 1 : 0, calibrated_ ? 1 : 0);
  }

  // ── Calibration (noexcept — EventLoop hot path) ───────────────────────────

  /// 캘리브레이션 데이터 축적. sensor cycle마다 호출.
  /// @return true: 캘리브레이션 완료
  [[nodiscard]] bool FeedCalibration(
      const std::array<int32_t, kMaxHandSensors>& sensor_data,
      int num_fingertips) noexcept {
    if (calibrated_) return true;

    const int n = std::min(num_fingertips,
                           std::min(config_.num_fingertips, kMaxFingertips));
    for (int f = 0; f < n; ++f) {
      const int base = f * kSensorValuesPerFingertip;
      for (int b = 0; b < kBarometerCount; ++b) {
        calibration_sum_[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] +=
            static_cast<double>(sensor_data[static_cast<std::size_t>(base + b)]);
      }
    }
    ++calibration_count_;

    if (calibration_count_ >= config_.calibration_samples) {
      const double inv_count = 1.0 / static_cast<double>(calibration_count_);
      for (int f = 0; f < n; ++f) {
        for (int b = 0; b < kBarometerCount; ++b) {
          baseline_offset_[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] =
              static_cast<float>(
                  calibration_sum_[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] * inv_count);
        }
      }
      calibrated_ = true;
      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "Calibration COMPLETE (%d samples)", calibration_count_);
      return true;
    }
    return false;
  }

  // ── Inference (noexcept, allocation-free) ──────────────────────────────────

  /// Per-fingertip 순차 추론. sensor_data에서 barometer만 추출 → 정규화 → history FIFO → 추론.
  /// 3-head output: sigmoid(contact_logit) + u 필터링 + 직렬화.
  /// history가 history_length만큼 채워지지 않으면 추론을 수행하지 않고 invalid 반환.
  [[nodiscard]] FingertipFTState Infer(
      const std::array<int32_t, kMaxHandSensors>& sensor_data,
      int num_fingertips) noexcept {
    FingertipFTState result{};
    if (!initialized_ || !calibrated_) return result;

    try {
      const int n = std::min(num_fingertips,
                             std::min(config_.num_fingertips, kMaxFingertips));
      const int H = config_.history_length;
      bool all_ready = true;

      for (int f = 0; f < n; ++f) {
        auto& model = models_[static_cast<std::size_t>(f)];
        if (!model.valid) continue;

        const int sensor_base = f * kSensorValuesPerFingertip;
        const auto fi = static_cast<std::size_t>(f);

        // ── 새 row 계산: baro(8) + delta(8) = 16 float ──────────────────
        std::array<float, kFTInputSize> new_row{};

        // barometer[0..7]: baseline-offset → reciprocal 정규화 (mul)
        std::array<float, kBarometerCount> cur_baro{};
        for (int b = 0; b < kBarometerCount; ++b) {
          const auto bi = static_cast<std::size_t>(b);
          const float raw = static_cast<float>(
              sensor_data[static_cast<std::size_t>(sensor_base + b)]);
          cur_baro[bi] = raw - baseline_offset_[fi][bi];
          new_row[bi] = cur_baro[bi] * input_max_reciprocal_[fi][bi];
        }

        // barometer_delta[8..15]: (current - previous) × reciprocal
        for (int b = 0; b < kBarometerCount; ++b) {
          const auto bi = static_cast<std::size_t>(b);
          const auto di = static_cast<std::size_t>(kBarometerCount + b);
          new_row[di] = (cur_baro[bi] - prev_barometer_[fi][bi])
                        * input_max_reciprocal_[fi][di];
        }

        prev_barometer_[fi] = cur_baro;

        // ── FIFO shift ──────────────────────────────────────────────────
        const auto row_bytes = static_cast<std::size_t>(kFTInputSize) * sizeof(float);
        if (H > 1) {
          std::memmove(model.input_buffer.data(),
                       model.input_buffer.data() + kFTInputSize,
                       static_cast<std::size_t>(H - 1) * row_bytes);
        }
        std::memcpy(model.input_buffer.data() +
                         static_cast<std::size_t>(H - 1) * kFTInputSize,
                     new_row.data(), row_bytes);

        // History count 증가 (최대 H)
        if (model.history_count < H) {
          ++model.history_count;
        }

        if (model.history_count < H) {
          all_ready = false;
          continue;
        }

        // ── 추론 (IoBinding → 3 output buffers에 직접 기록) ──────────────
        model.session->Run(Ort::RunOptions{nullptr}, *model.io_binding);

        // ── 후처리: sigmoid + 필터링 + 직렬화 ──────────────────────────
        const int ft_base = f * kFTValuesPerFingertip;

        // 1. Sigmoid 적용 (contact logit → 확률)
        const float contact_logit = model.output0_buffer[0];
        const float contact_prob = 1.0f / (1.0f + std::exp(-contact_logit));

        // 2. F와 u 복사
        float F[3] = { model.output1_buffer[0], model.output1_buffer[1], model.output1_buffer[2] };
        float u[3] = { model.output2_buffer[0], model.output2_buffer[1], model.output2_buffer[2] };

        // 3. 필터링: 비접촉 시 u 벡터 전체를 0으로 (센서 노이즈 차단)
        if (contact_prob < 0.1f) {
          u[0] = 0.0f;
          u[1] = 0.0f;
          u[2] = 0.0f;
        }

        // 4. ft_data에 직렬화: [contact_prob, F(3), u(3)] = 7 values
        result.ft_data[static_cast<std::size_t>(ft_base + 0)] = contact_prob;
        result.ft_data[static_cast<std::size_t>(ft_base + 1)] = F[0];
        result.ft_data[static_cast<std::size_t>(ft_base + 2)] = F[1];
        result.ft_data[static_cast<std::size_t>(ft_base + 3)] = F[2];
        result.ft_data[static_cast<std::size_t>(ft_base + 4)] = u[0];
        result.ft_data[static_cast<std::size_t>(ft_base + 5)] = u[1];
        result.ft_data[static_cast<std::size_t>(ft_base + 6)] = u[2];
        result.per_fingertip_valid[static_cast<std::size_t>(f)] = true;
      }

      result.num_fingertips = n;
      result.valid = all_ready;
    } catch (const std::exception& e) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("FT-Inferencer"),
                            steady_clock, 5000,
                            "FT inference exception: %s", e.what());
      result.valid = false;
    } catch (...) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("FT-Inferencer"),
                            steady_clock, 5000,
                            "FT inference unknown exception (result invalidated)");
      result.valid = false;
    }

    return result;
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
  [[nodiscard]] bool is_calibrated() const noexcept { return calibrated_; }
  [[nodiscard]] int  num_models() const noexcept { return num_active_; }
  [[nodiscard]] int  calibration_count() const noexcept { return calibration_count_; }
  [[nodiscard]] int  calibration_target() const noexcept {
    return config_.calibration_samples;
  }

  /// 캘리브레이션으로 측정된 baseline offset (per-fingertip × per-channel)
  [[nodiscard]] const auto& baseline_offset() const noexcept {
    return baseline_offset_;
  }

 private:
  // Per-fingertip 모델 데이터 (사전 할당)
  struct PerFingertipModel {
    std::unique_ptr<Ort::Session> session;
    std::unique_ptr<Ort::IoBinding> io_binding;
    // History buffer: [history_length × kFTInputSize] = [12 × 16] = 192 floats
    std::array<float, kFTHistoryLength * kFTInputSize> input_buffer{};
    std::array<float, 1> output0_buffer{};  // contact logit
    std::array<float, 3> output1_buffer{};  // F: force vector
    std::array<float, 3> output2_buffer{};  // u: direction vector
    Ort::Value input_tensor{nullptr};
    Ort::Value output0_tensor{nullptr};
    Ort::Value output1_tensor{nullptr};
    Ort::Value output2_tensor{nullptr};
    std::string input_name;
    std::string output0_name;
    std::string output1_name;
    std::string output2_name;
    int  history_count{0};
    bool valid{false};
  };

  Config config_{};
  bool   initialized_{false};
  int    num_active_{0};

  // ONNX Runtime 공유 객체
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "fingertip_ft"};
  Ort::AllocatorWithDefaultOptions allocator_;
  Ort::MemoryInfo memory_info_{nullptr};

  // Per-fingertip 모델 배열
  std::array<PerFingertipModel, kMaxFingertips> models_;

  // Baseline Offset Calibration
  std::array<std::array<double, kBarometerCount>, kMaxFingertips> calibration_sum_{};
  std::array<std::array<float, kBarometerCount>, kMaxFingertips>  baseline_offset_{};
  int  calibration_count_{0};
  bool calibrated_{false};

  // input_max 역수 (div → mul 최적화, InitFT()에서 사전 계산)
  std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_max_reciprocal_{};

  // 이전 barometer 값 (delta 계산용)
  std::array<std::array<float, kBarometerCount>, kMaxFingertips> prev_barometer_{};
};

#endif  // HAS_ONNXRUNTIME

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_
