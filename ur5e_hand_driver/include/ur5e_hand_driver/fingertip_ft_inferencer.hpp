#ifndef UR5E_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_
#define UR5E_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_

// Per-fingertip Force/Torque 추론기 — OnnxEngine 상속.
//
// 각 fingertip마다 개별 ONNX 모델을 OnnxEngine에 등록하여 접촉/힘 추론.
//   Input:  float32[1, H, 16]  (H=history_length, barometer 8ch + barometer_delta 8ch, 정규화됨)
//   Output: float32[1, 1, 13]  ([contact(1), F(3), u(3), Fn(3), Fx(1), Fy(1), Fz(1)])
//
// RT safety:
//   - InitFT()에서 모든 동적 할당 수행 (non-RT 컨텍스트)
//   - Infer()/FeedCalibration()는 noexcept + allocation-free
//   - OnnxEngine의 사전 할당된 I/O 버퍼 + IoBinding으로 zero-alloc 추론
//
// Baseline Offset Calibration:
//   - InitFT() 이후 FeedCalibration()으로 센서 baseline 자동 측정
//   - 정규화 공식: (raw - baseline_offset) / input_max → [-1, +1]

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>

#include "rtc_base/types/types.hpp"
#include "rtc_inference/onnx/onnx_engine.hpp"

namespace rtc {

class FingertipFTInferencer : public OnnxEngine {
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
  ~FingertipFTInferencer() override = default;

  FingertipFTInferencer(const FingertipFTInferencer&) = delete;
  FingertipFTInferencer& operator=(const FingertipFTInferencer&) = delete;
  FingertipFTInferencer(FingertipFTInferencer&&) = delete;
  FingertipFTInferencer& operator=(FingertipFTInferencer&&) = delete;

  // ── Lifecycle (non-RT) ────────────────────────────────────────────────────

  /// 도메인 초기화: OnnxEngine::Init(ModelConfig)를 fingertip별로 호출,
  /// 캘리브레이션/history 상태 초기화.
  /// non-RT 컨텍스트에서만 호출. 실패 시 예외.
  void InitFT(const Config& config) {
    config_ = config;
    const int n = std::min(config_.num_fingertips, kMaxFingertips);
    finger_to_model_.fill(-1);

    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "InitFT: num_fingertips=%d, history_length=%d, model_paths.size=%zu, calibration=%s(%d samples)",
                n, config_.history_length, config_.model_paths.size(),
                config_.calibration_enabled ? "ON" : "OFF",
                config_.calibration_samples);

    for (int f = 0; f < n; ++f) {
      // 모델 경로 없으면 해당 finger 비활성
      if (f >= static_cast<int>(config_.model_paths.size()) ||
          config_.model_paths[static_cast<std::size_t>(f)].empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FT-Inferencer"),
                    "finger[%d]: SKIPPED (empty path)", f);
        continue;
      }

      const auto& path = config_.model_paths[static_cast<std::size_t>(f)];
      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "finger[%d]: loading \"%s\"", f, path.c_str());

      // OnnxEngine에 모델 등록
      ModelConfig mc;
      mc.model_path = path;
      mc.input_shape = {1, static_cast<int64_t>(config_.history_length),
                        static_cast<int64_t>(kFTInputSize)};
      mc.output_shape = {1, 1, static_cast<int64_t>(kFTValuesPerFingertip)};
      mc.intra_op_threads = 1;

      OnnxEngine::Init(mc);
      finger_to_model_[static_cast<std::size_t>(f)] = num_models() - 1;

      RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                  "finger[%d]: loaded OK (model_idx=%d)", f, num_models() - 1);
    }

    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "num_active=%d / %d", num_models(), n);

    // prev_barometer 초기화 (delta 계산용)
    for (auto& p : prev_barometer_) p.fill(0.0f);

    // history_count 초기화
    history_count_.fill(0);

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

    // input_max 역수 사전 계산 (Infer()에서 div → mul 최적화)
    for (int f = 0; f < n; ++f) {
      const auto fi = static_cast<std::size_t>(f);
      for (int ch = 0; ch < kFTInputSize; ++ch) {
        const auto ci = static_cast<std::size_t>(ch);
        const float m = config_.input_max[fi][ci];
        input_max_reciprocal_[fi][ci] = (m == 0.0f) ? 1.0f : (1.0f / m);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("FT-Inferencer"),
                "InitFT done: initialized=%d, calibrated=%d",
                is_initialized() ? 1 : 0, calibrated_ ? 1 : 0);
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

  // ── Inference (noexcept, allocation-free, 3-phase pipeline) ────────────────

  /// 3-phase pipeline 추론:
  ///   Phase 1: 전처리 — 모든 fingertip의 정규화 + delta + FIFO shift (L1i cache 유지)
  ///   Phase 2: 추론 — RunModels()로 모든 ready 모델 일괄 실행 (ONNX runtime L1i 유지)
  ///   Phase 3: 결과 복사 — output_buffer → FingertipFTState
  [[nodiscard]] FingertipFTState Infer(
      const std::array<int32_t, kMaxHandSensors>& sensor_data,
      int num_fingertips) noexcept {
    FingertipFTState result{};
    if (!is_initialized() || !calibrated_) return result;

    try {
      const int n = std::min(num_fingertips,
                             std::min(config_.num_fingertips, kMaxFingertips));
      const int H = config_.history_length;
      bool all_ready = true;

      // Ready 모델 인덱스 수집 + finger→model 매핑 (Phase 2, 3에서 사용)
      std::array<int, kMaxFingertips> ready_models{};
      std::array<int, kMaxFingertips> ready_fingers{};
      int num_ready = 0;

      // ── Phase 1: 전처리 (모든 fingertip) ─────────────────────────────────
      for (int f = 0; f < n; ++f) {
        const auto fi = static_cast<std::size_t>(f);
        const int model_idx = finger_to_model_[fi];
        if (model_idx < 0) continue;

        const int sensor_base = f * kSensorValuesPerFingertip;

        // 새 row 계산: baro(8) + delta(8) = 16 float
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

        // FIFO shift: OnnxEngine의 input_buffer에 직접 조작
        float* buf = input_buffer(model_idx);
        const auto row_bytes = static_cast<std::size_t>(kFTInputSize) * sizeof(float);
        if (H > 1) {
          std::memmove(buf,
                       buf + kFTInputSize,
                       static_cast<std::size_t>(H - 1) * row_bytes);
        }
        std::memcpy(buf + static_cast<std::size_t>(H - 1) * kFTInputSize,
                     new_row.data(), row_bytes);

        // History count 증가 (최대 H)
        if (history_count_[fi] < H) {
          ++history_count_[fi];
        }

        if (history_count_[fi] < H) {
          all_ready = false;
        } else {
          ready_models[static_cast<std::size_t>(num_ready)] = model_idx;
          ready_fingers[static_cast<std::size_t>(num_ready)] = f;
          ++num_ready;
        }
      }

      // ── Phase 2: 일괄 추론 (direct Session::Run, IoBinding 우회) ─────────
      if (num_ready > 0) {
        (void)RunModels(ready_models.data(), num_ready);
      }

      // ── Phase 3: 결과 복사 ───────────────────────────────────────────────
      for (int i = 0; i < num_ready; ++i) {
        const int f = ready_fingers[static_cast<std::size_t>(i)];
        const int model_idx = ready_models[static_cast<std::size_t>(i)];
        const int ft_base = f * kFTValuesPerFingertip;
        std::memcpy(&result.ft_data[static_cast<std::size_t>(ft_base)],
                     output_buffer(model_idx),
                     sizeof(float) * kFTValuesPerFingertip);
      }

      result.num_fingertips = n;
      result.valid = all_ready;
    } catch (...) {
      result.valid = false;
    }

    return result;
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] bool is_calibrated() const noexcept { return calibrated_; }
  [[nodiscard]] int  calibration_count() const noexcept { return calibration_count_; }
  [[nodiscard]] int  calibration_target() const noexcept {
    return config_.calibration_samples;
  }

  /// 캘리브레이션으로 측정된 baseline offset (per-fingertip × per-channel)
  [[nodiscard]] const auto& baseline_offset() const noexcept {
    return baseline_offset_;
  }

 private:
  Config config_{};

  // fingertip index → OnnxEngine model index 매핑 (-1 = inactive)
  std::array<int, kMaxFingertips> finger_to_model_{};

  // Per-fingertip history count (0 ~ history_length)
  std::array<int, kMaxFingertips> history_count_{};

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

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_
