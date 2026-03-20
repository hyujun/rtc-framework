#ifndef UR5E_HAND_UDP_FINGERTIP_FT_INFERENCER_HPP_
#define UR5E_HAND_UDP_FINGERTIP_FT_INFERENCER_HPP_

// Per-fingertip ONNX Runtime 기반 Force/Torque 추론기.
//
// 각 fingertip마다 개별 ONNX 모델을 로드하여 barometer + delta → 접촉/힘 추론.
//   Input:  float32[1, 16]  (barometer 8ch + barometer_delta 8ch, 정규화됨)
//   Output: float32[1, 13]  ([contact(1), F(3), u(3), Fn(3), Fx(1), Fy(1), Fz(1)])
//
// RT safety:
//   - Init()에서 모든 동적 할당 수행 (non-RT 컨텍스트)
//   - Infer()/FeedCalibration()는 noexcept + allocation-free
//   - 사전 할당된 I/O 버퍼 + Ort::IoBinding으로 zero-alloc 추론
//
// Baseline Offset Calibration:
//   - Init() 이후 FeedCalibration()으로 센서 baseline 자동 측정
//   - 정규화 공식: (raw - baseline_offset - yaml_mean) / yaml_std

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

#include <rclcpp/logging.hpp>

#include "ur5e_rt_base/types/types.hpp"

namespace ur5e_rt_controller {

#ifndef HAS_ONNXRUNTIME

/// Stub implementation when ONNX Runtime is not available.
class FingertipFTInferencer {
 public:
  struct Config {
    bool enabled{false};
    int  num_fingertips{kDefaultNumFingertips};
    std::vector<std::string> model_paths;
    // 정규화 파라미터: [fingertip][input_channel] (baro 8 + delta 8 = 16)
    std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_mean{};
    std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_std{};
    bool calibration_enabled{true};
    int  calibration_samples{500};
  };

  void Init(const Config& /*config*/) {
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
  [[nodiscard]] int  num_active_models() const noexcept { return 0; }
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

    // Per-fingertip ONNX 모델 경로 (빈 문자열 → 해당 finger 비활성)
    std::vector<std::string> model_paths;

    // Per-fingertip 정규화 파라미터 [fingertip][input_channel]
    // input_channel: baro[0..7] + delta[0..7] = 16
    std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_mean{};
    std::array<std::array<float, kFTInputSize>, kMaxFingertips> input_std{};

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
  void Init(const Config& config) {
    config_ = config;
    const int n = std::min(config_.num_fingertips, kMaxFingertips);
    num_active_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "Init: num_fingertips=%d, model_paths.size=%zu, calibration=%s(%d samples)",
                n, config_.model_paths.size(),
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

      // Input/Output 이름 쿼리
      // ORT_API_VERSION >= 13 (v1.13+): GetInputNameAllocated 사용
      // 이전 버전 (Ubuntu 22.04 apt v1.11): GetInputName 사용
#if ORT_API_VERSION >= 13
      auto input_name_alloc = model.session->GetInputNameAllocated(0, allocator_);
      auto output_name_alloc = model.session->GetOutputNameAllocated(0, allocator_);
      model.input_name = input_name_alloc.get();
      model.output_name = output_name_alloc.get();
#else
      {
        char* in_name = model.session->GetInputName(0, allocator_);
        char* out_name = model.session->GetOutputName(0, allocator_);
        model.input_name = in_name;
        model.output_name = out_name;
        allocator_.Free(in_name);
        allocator_.Free(out_name);
      }
#endif

      // 사전 할당된 버퍼 위에 Ort::Value 텐서 생성
      constexpr int64_t input_shape[] = {1, kFTInputSize};           // [1, 16]
      constexpr int64_t output_shape[] = {1, kFTValuesPerFingertip}; // [1, 13]

      model.input_tensor = Ort::Value::CreateTensor<float>(
          memory_info_, model.input_buffer.data(),
          model.input_buffer.size(), input_shape, 2);

      model.output_tensor = Ort::Value::CreateTensor<float>(
          memory_info_, model.output_buffer.data(),
          model.output_buffer.size(), output_shape, 2);

      // IoBinding 생성 + 바인딩
      model.io_binding = std::make_unique<Ort::IoBinding>(*model.session);
      model.io_binding->BindInput(model.input_name.c_str(), model.input_tensor);
      model.io_binding->BindOutput(model.output_name.c_str(), model.output_tensor);

      model.valid = true;
      ++num_active_;
      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "finger[%d]: loaded OK (input=%s, output=%s)",
                  f, model.input_name.c_str(), model.output_name.c_str());
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
      // 캘리브레이션 비활성: baseline_offset = 0, 즉시 calibrated
      for (auto& b : baseline_offset_) b.fill(0.0f);
      calibrated_ = true;
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
                "Init done: initialized=%d, calibrated=%d",
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

  /// Per-fingertip 순차 추론. sensor_data에서 barometer만 추출 → 정규화 → 추론.
  [[nodiscard]] FingertipFTState Infer(
      const std::array<int32_t, kMaxHandSensors>& sensor_data,
      int num_fingertips) noexcept {
    FingertipFTState result{};
    if (!initialized_ || !calibrated_) return result;

    try {
      const int n = std::min(num_fingertips,
                             std::min(config_.num_fingertips, kMaxFingertips));

      for (int f = 0; f < n; ++f) {
        auto& model = models_[static_cast<std::size_t>(f)];
        if (!model.valid) continue;

        const int sensor_base = f * kSensorValuesPerFingertip;
        const auto fi = static_cast<std::size_t>(f);

        // ── barometer[0..7]: baseline-offset 정규화 ────────────────────
        std::array<float, kBarometerCount> cur_baro{};
        for (int b = 0; b < kBarometerCount; ++b) {
          const auto bi = static_cast<std::size_t>(b);
          const float raw = static_cast<float>(
              sensor_data[static_cast<std::size_t>(sensor_base + b)]);
          cur_baro[bi] = raw - baseline_offset_[fi][bi];

          const float mean = config_.input_mean[fi][bi];
          float std_val = config_.input_std[fi][bi];
          if (std_val == 0.0f) std_val = 1.0f;
          model.input_buffer[bi] = (cur_baro[bi] - mean) / std_val;
        }

        // ── barometer_delta[8..15]: current - previous ─────────────────
        for (int b = 0; b < kBarometerCount; ++b) {
          const auto bi = static_cast<std::size_t>(b);
          const auto di = static_cast<std::size_t>(kBarometerCount + b);
          const float delta = cur_baro[bi] - prev_barometer_[fi][bi];

          const float mean = config_.input_mean[fi][di];
          float std_val = config_.input_std[fi][di];
          if (std_val == 0.0f) std_val = 1.0f;
          model.input_buffer[di] = (delta - mean) / std_val;
        }

        // 현재 값을 prev에 저장 (다음 사이클의 delta 계산용)
        prev_barometer_[fi] = cur_baro;

        // 추론 (IoBinding → output_buffer에 직접 기록)
        model.session->Run(Ort::RunOptions{nullptr}, *model.io_binding);

        // 결과 복사: output_buffer[13] → ft_data[f*13 .. f*13+12]
        const int ft_base = f * kFTValuesPerFingertip;
        std::memcpy(&result.ft_data[static_cast<std::size_t>(ft_base)],
                     model.output_buffer.data(),
                     sizeof(float) * kFTValuesPerFingertip);
      }

      result.num_fingertips = n;
      result.valid = true;
    } catch (...) {
      // 예외 발생 시 invalid 반환 (noexcept 보장)
      result.valid = false;
    }

    return result;
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
  [[nodiscard]] bool is_calibrated() const noexcept { return calibrated_; }
  [[nodiscard]] int  num_active_models() const noexcept { return num_active_; }
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
    std::array<float, kFTInputSize>          input_buffer{};   // baro(8) + delta(8) = 16
    std::array<float, kFTValuesPerFingertip> output_buffer{};  // 13 outputs
    Ort::Value input_tensor{nullptr};
    Ort::Value output_tensor{nullptr};
    std::string input_name;
    std::string output_name;
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

  // 이전 barometer 값 (delta 계산용)
  std::array<std::array<float, kBarometerCount>, kMaxFingertips> prev_barometer_{};
};

#endif  // HAS_ONNXRUNTIME

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_FINGERTIP_FT_INFERENCER_HPP_
