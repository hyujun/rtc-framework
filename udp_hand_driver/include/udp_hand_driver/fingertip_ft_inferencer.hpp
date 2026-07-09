#ifndef UDP_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_
#define UDP_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_

// Per-fingertip Force/Torque 추론기. ONNX 실행은 rtc::OnnxEngine (single-input,
// 3-output) 에 위임하고, 이 클래스는 fingertip 별 history FIFO / 정규화 /
// baseline calibration / post-processing 만 소유한다.
//
// 각 fingertip마다 개별 ONNX 모델 (engine 의 1 model) 을 로드하여 barometer + delta → 접촉/힘 추론.
//   Input:  float32[1, H, 16]  (H=history_length, barometer 8ch + barometer_delta 8ch, 정규화됨)
//   Outputs (3 heads):
//     output0: float32[1, 1]  (contact logit → sigmoid → probability)
//     output1: float32[1, 3]  (F: force vector)
//     output2: float32[1, 3]  (u: direction vector, filtered when no contact)
//
// RT safety:
//   - InitFT()에서 모든 동적 할당 수행 (non-RT 컨텍스트; session/tensor/warmup 은 engine 소유)
//   - Infer()/FeedCalibration()는 noexcept + allocation-free
//   - engine_.input_buffer()/output_buffer() 사전 할당 버퍼 위 zero-alloc 추론
//
// Baseline Offset Calibration:
//   - InitFT() 이후 FeedCalibration()으로 센서 baseline 자동 측정
//   - 정규화 공식: (raw - baseline_offset) / input_max → [-1, +1]

#include "rtc_base/types/types.hpp"
#include "rtc_inference/onnx/onnx_engine.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_logging.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace udp_hand_driver {

#ifndef HAS_ONNXRUNTIME

/// Stub implementation when ONNX Runtime is not available.
class FingertipFTInferencer {
 public:
  struct Config {
    bool enabled{false};
    int num_fingertips{udp_hand_driver::kDefaultNumFingertips};
    int history_length{udp_hand_driver::kFTHistoryLength};
    std::vector<std::string> model_paths;
    std::array<std::array<float, udp_hand_driver::kFTInputSize>, kMaxFingertips> input_max{};
    bool calibration_enabled{true};
    int calibration_samples{500};
  };

  void InitFT(const Config& /*config*/) {
    RCLCPP_ERROR(::udp_hand_driver::logging::FtLogger(),
                 "STUB: HAS_ONNXRUNTIME not defined! "
                 "ONNX Runtime unavailable — FT inference disabled.");
  }

  [[nodiscard]] bool FeedCalibration(
      const std::array<int32_t, udp_hand_driver::kMaxHandSensors>& /*sensor_data*/,
      int /*num_fingertips*/) noexcept {
    return true;
  }

  void ResetCalibration(uint16_t /*sample_count*/ = 0) noexcept {}

  [[nodiscard]] udp_hand_driver::FingertipFTState Infer(
      const std::array<int32_t, udp_hand_driver::kMaxHandSensors>& /*sensor_data*/,
      int /*num_fingertips*/) noexcept {
    return {};
  }

  [[nodiscard]] bool is_initialized() const noexcept { return false; }

  [[nodiscard]] bool is_calibrated() const noexcept { return false; }

  [[nodiscard]] int num_models() const noexcept { return 0; }

  [[nodiscard]] int calibration_count() const noexcept { return 0; }

  [[nodiscard]] int calibration_target() const noexcept { return 0; }

  [[nodiscard]] std::array<std::array<float, udp_hand_driver::kBarometerCount>, kMaxFingertips>
  baseline_offset() const noexcept {
    return {};
  }
};

#else  // HAS_ONNXRUNTIME

class FingertipFTInferencer {
 public:
  // ── Configuration ─────────────────────────────────────────────────────────
  struct Config {
    bool enabled{false};
    int num_fingertips{udp_hand_driver::kDefaultNumFingertips};
    int history_length{udp_hand_driver::kFTHistoryLength};  // FIFO history rows (default: 12)

    // Per-fingertip ONNX 모델 경로 (빈 문자열 → 해당 finger 비활성)
    std::vector<std::string> model_paths;

    // Per-fingertip 정규화 파라미터: input_max [fingertip][input_channel]
    // input_channel: baro[0..7] + delta[0..7] = 16
    // 정규화 공식: value / input_max → [-1, +1]
    std::array<std::array<float, udp_hand_driver::kFTInputSize>, kMaxFingertips> input_max{};

    // Baseline Offset Calibration
    bool calibration_enabled{true};
    int calibration_samples{500};  // sensor cycle 기준 (500Hz/decimation)
  };

  FingertipFTInferencer() = default;
  ~FingertipFTInferencer() = default;

  FingertipFTInferencer(const FingertipFTInferencer&) = delete;
  FingertipFTInferencer& operator=(const FingertipFTInferencer&) = delete;
  FingertipFTInferencer(FingertipFTInferencer&&) = delete;
  FingertipFTInferencer& operator=(FingertipFTInferencer&&) = delete;

  // ── Lifecycle (non-RT) ────────────────────────────────────────────────────

  /// 모델 로드 + 텐서 사전 할당 + warmup 은 rtc::OnnxEngine 에 위임.
  /// 이 클래스는 per-fingertip history/normalization/calibration/post-proc 만 소유.
  /// non-RT 컨텍스트에서만 호출. 실패 시 예외.
  void InitFT(const Config& config) {
    config_ = config;
    // history_length 는 ROS 파라미터(하한 미검증)이므로 방어적으로 clamp.
    // H<1 이면 input_shape {1,0,16} → engine 입력 버퍼 크기 0, FIFO memcpy 가
    // 버퍼 앞쪽 OOB write 를 일으킨다.
    if (config_.history_length < 1) {
      RCLCPP_WARN(::udp_hand_driver::logging::FtLogger(),
                  "history_length=%d invalid; clamping to 1", config_.history_length);
      config_.history_length = 1;
    }
    const int n = std::min(config_.num_fingertips, kMaxFingertips);
    engine_.Reset();  // InitFT 재호출 시 이전 모델 중복 등록 방지 (idempotent)
    ft_to_model_.fill(-1);
    history_count_.fill(0);

    RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(),
                "InitFT: num_fingertips=%d, history_length=%d, model_paths.size=%zu, "
                "calibration=%s(%d samples)",
                n, config_.history_length, config_.model_paths.size(),
                config_.calibration_enabled ? "ON" : "OFF", config_.calibration_samples);

    // 3-head output: contact logit [1,1], F [1,3], u [1,3]
    const int64_t H = static_cast<int64_t>(config_.history_length);
    rtc::ModelConfig model_config;
    model_config.input_shape = {1, H, udp_hand_driver::kFTInputSize};  // [1, H, 16]
    model_config.output_shapes = {{1, 1}, {1, 3}, {1, 3}};
    model_config.intra_op_threads = 1;

    for (int f = 0; f < n; ++f) {
      // 모델 경로 없으면 해당 finger 비활성
      if (f >= static_cast<int>(config_.model_paths.size()) ||
          config_.model_paths[static_cast<std::size_t>(f)].empty()) {
        RCLCPP_WARN(::udp_hand_driver::logging::FtLogger(), "finger[%d]: SKIPPED (empty path)", f);
        continue;
      }

      const auto& path = config_.model_paths[static_cast<std::size_t>(f)];
      RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(), "finger[%d]: loading \"%s\"", f,
                  path.c_str());

      // Session/IoBinding/tensor/warmup 은 engine_ 이 처리.
      // model arity/shape 가 config 와 다르면 engine_.Init 이 throw (loud failure).
      model_config.model_path = path;
      engine_.Init(model_config);  // 등록 순서대로 model index 부여
      ft_to_model_[static_cast<std::size_t>(f)] = engine_.num_models() - 1;
      RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(), "finger[%d]: loaded OK (model_idx=%d)", f,
                  ft_to_model_[static_cast<std::size_t>(f)]);
    }

    RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(), "num_active=%d / %d", engine_.num_models(),
                n);

    // prev_barometer 초기화 (delta 계산용)
    for (auto& p : prev_barometer_)
      p.fill(0.0f);

    // Calibration 초기화
    if (config_.calibration_enabled) {
      for (auto& s : calibration_sum_)
        s.fill(0.0);
      calibration_count_.store(0, std::memory_order_relaxed);
      calibrated_.store(false, std::memory_order_relaxed);
    } else {
      for (auto& b : baseline_offset_)
        b.fill(0.0f);
      calibrated_.store(true, std::memory_order_relaxed);
    }

    // input_max 역수 사전 계산 (Infer()에서 div → mul 최적화)
    for (int f = 0; f < n; ++f) {
      const auto fi = static_cast<std::size_t>(f);
      for (int ch = 0; ch < udp_hand_driver::kFTInputSize; ++ch) {
        const auto ci = static_cast<std::size_t>(ch);
        const float m = config_.input_max[fi][ci];
        input_max_reciprocal_[fi][ci] = (m == 0.0f) ? 1.0f : (1.0f / m);
      }
    }

    initialized_ = (engine_.num_models() > 0);
    RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(),
                "InitFT done: initialized=%d, calibrated=%d", initialized_ ? 1 : 0,
                calibrated_.load(std::memory_order_relaxed) ? 1 : 0);
  }

  // ── Calibration (noexcept — EventLoop hot path) ───────────────────────────

  /// 캘리브레이션 데이터 축적. sensor cycle마다 호출.
  /// @return true: 캘리브레이션 완료
  [[nodiscard]] bool FeedCalibration(
      const std::array<int32_t, udp_hand_driver::kMaxHandSensors>& sensor_data,
      int num_fingertips) noexcept {
    if (calibrated_.load(std::memory_order_relaxed))
      return true;

    const int n = std::min(num_fingertips, std::min(config_.num_fingertips, kMaxFingertips));
    for (int f = 0; f < n; ++f) {
      const int base = f * udp_hand_driver::kSensorValuesPerFingertip;
      for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
        calibration_sum_[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] +=
            static_cast<double>(sensor_data[static_cast<std::size_t>(base + b)]);
      }
    }
    const int count = calibration_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    if (count >= config_.calibration_samples) {
      const double inv_count = 1.0 / static_cast<double>(count);
      for (int f = 0; f < n; ++f) {
        for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
          baseline_offset_[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] =
              static_cast<float>(
                  calibration_sum_[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] *
                  inv_count);
        }
      }
      calibrated_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(), "Calibration COMPLETE (%d samples)",
                  count);
      return true;
    }
    return false;
  }

  /// 캘리브레이션 재시작: 누적 버퍼 + baseline_offset_ 리셋.
  /// baseline_offset_를 0으로 초기화하여 재수집 중 FT inference가
  /// 오래된/무효 baseline으로 동작하지 않도록 보장 (is_calibrated()==false
  /// 동안 Infer()는 일찍 리턴함).
  /// @param sample_count 0이면 기존 target 유지, >0이면 이번 요청에 한해 override.
  void ResetCalibration(uint16_t sample_count = 0) noexcept {
    if (sample_count > 0) {
      config_.calibration_samples = static_cast<int>(sample_count);
    }
    for (auto& s : calibration_sum_)
      s.fill(0.0);
    for (auto& b : baseline_offset_)
      b.fill(0.0f);
    calibration_count_.store(0, std::memory_order_relaxed);
    calibrated_.store(false, std::memory_order_relaxed);
    RCLCPP_INFO(::udp_hand_driver::logging::FtLogger(), "Calibration RESET (target=%d samples)",
                config_.calibration_samples);
  }

  // ── Inference (noexcept, allocation-free) ──────────────────────────────────

  /// Per-fingertip 순차 추론. sensor_data에서 barometer만 추출 → 정규화 → history FIFO → 추론.
  /// 3-head output: sigmoid(contact_logit) + u 필터링 + 직렬화.
  /// history가 history_length만큼 채워지지 않으면 추론을 수행하지 않고 invalid 반환.
  [[nodiscard]] udp_hand_driver::FingertipFTState Infer(
      const std::array<int32_t, udp_hand_driver::kMaxHandSensors>& sensor_data,
      int num_fingertips) noexcept {
    udp_hand_driver::FingertipFTState result{};
    if (!initialized_ || !calibrated_.load(std::memory_order_relaxed))
      return result;

    // No try/catch: engine_.RunModels() is noexcept (it catches ORT exceptions
    // internally and returns false), and the remaining ops (memmove/memcpy/exp/
    // array writes) cannot throw — so this whole body is non-throwing.
    const int n = std::min(num_fingertips, std::min(config_.num_fingertips, kMaxFingertips));
    const int H = config_.history_length;
    bool all_ready = true;

    for (int f = 0; f < n; ++f) {
      const auto fi = static_cast<std::size_t>(f);
      const int mi = ft_to_model_[fi];
      if (mi < 0)  // skipped finger (empty model path)
        continue;

      const int sensor_base = f * udp_hand_driver::kSensorValuesPerFingertip;

      // ── 새 row 계산: baro(8) + delta(8) = 16 float ──────────────────
      std::array<float, udp_hand_driver::kFTInputSize> new_row{};

      // barometer[0..7]: baseline-offset → reciprocal 정규화 (mul)
      std::array<float, udp_hand_driver::kBarometerCount> cur_baro{};
      for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
        const auto bi = static_cast<std::size_t>(b);
        const float raw =
            static_cast<float>(sensor_data[static_cast<std::size_t>(sensor_base + b)]);
        cur_baro[bi] = raw - baseline_offset_[fi][bi];
        new_row[bi] = cur_baro[bi] * input_max_reciprocal_[fi][bi];
      }

      // barometer_delta[8..15]: (current - previous) × reciprocal
      for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
        const auto bi = static_cast<std::size_t>(b);
        const auto di = static_cast<std::size_t>(udp_hand_driver::kBarometerCount + b);
        new_row[di] = (cur_baro[bi] - prev_barometer_[fi][bi]) * input_max_reciprocal_[fi][di];
      }

      prev_barometer_[fi] = cur_baro;

      // ── FIFO shift (engine 이 소유한 입력 버퍼 위에서) ───────────────
      float* in_buf = engine_.input_buffer(mi);  // [H × 16] floats
      const auto row_bytes =
          static_cast<std::size_t>(udp_hand_driver::kFTInputSize) * sizeof(float);
      if (H > 1) {
        std::memmove(in_buf, in_buf + udp_hand_driver::kFTInputSize,
                     static_cast<std::size_t>(H - 1) * row_bytes);
      }
      std::memcpy(in_buf + static_cast<std::size_t>(H - 1) * udp_hand_driver::kFTInputSize,
                  new_row.data(), row_bytes);

      // History count 증가 (최대 H)
      if (history_count_[fi] < H) {
        ++history_count_[fi];
      }

      if (history_count_[fi] < H) {
        all_ready = false;
        continue;
      }

      // ── 추론 (engine: direct Session::Run, 3 output buffers에 직접 기록) ──
      if (!engine_.RunModels(&mi, 1)) {
        // ORT 추론 실패 (engine 이 예외를 삼키고 false 반환). hot path 라 throttle.
        // throttle_clock_ 은 멤버 (생성 시 1회 init) — static-local 의 first-call
        // 생성이 noexcept Infer 밖으로 throw 해 terminate 되는 경로를 차단.
        RCLCPP_ERROR_THROTTLE(::udp_hand_driver::logging::FtLogger(), throttle_clock_,
                              ::udp_hand_driver::logging::kThrottleHotMs,
                              "FT inference failed (finger=%d, model_idx=%d)", f, mi);
        all_ready = false;
        continue;
      }
      const float* out0 = engine_.output_buffer(mi, 0);  // contact logit [1,1]
      const float* out1 = engine_.output_buffer(mi, 1);  // F [1,3]
      const float* out2 = engine_.output_buffer(mi, 2);  // u [1,3]

      // ── 후처리: sigmoid + 필터링 + 직렬화 ──────────────────────────
      const int ft_base = f * udp_hand_driver::kFTValuesPerFingertip;

      // 1. Sigmoid 적용 (contact logit → 확률)
      const float contact_logit = out0[0];
      const float contact_prob = 1.0f / (1.0f + std::exp(-contact_logit));

      // 2. F와 u 복사
      float F[3] = {out1[0], out1[1], out1[2]};
      float u[3] = {out2[0], out2[1], out2[2]};

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
    return result;
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

  [[nodiscard]] bool is_calibrated() const noexcept {
    return calibrated_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] int num_models() const noexcept { return engine_.num_models(); }

  [[nodiscard]] int calibration_count() const noexcept {
    return calibration_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] int calibration_target() const noexcept { return config_.calibration_samples; }

  /// 캘리브레이션으로 측정된 baseline offset (per-fingertip × per-channel)
  [[nodiscard]] const auto& baseline_offset() const noexcept { return baseline_offset_; }

 private:
  Config config_{};
  bool initialized_{false};

  // 모든 ONNX session/iobinding/tensor/buffer/warmup 를 소유 (per-fingertip = model).
  // 등록된 model 수 = engine_.num_models() (별도 카운터 불필요).
  rtc::OnnxEngine engine_;

  // noexcept Infer() hot path 의 throttle 로깅용 clock. 멤버로 1회 생성하여
  // static-local first-call 생성이 Infer 밖으로 throw 하는 경로를 제거.
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // fingertip index → engine model index 매핑 (-1 = 모델 미로드/skip).
  // 빈 경로 finger 가 건너뛰어지므로 fingertip↔model 은 identity 가 아니다.
  std::array<int, kMaxFingertips> ft_to_model_{};

  // Per-fingertip FIFO history 채움 카운트 (engine 입력 버퍼의 row 수).
  std::array<int, kMaxFingertips> history_count_{};

  // Baseline Offset Calibration
  std::array<std::array<double, udp_hand_driver::kBarometerCount>, kMaxFingertips>
      calibration_sum_{};
  std::array<std::array<float, udp_hand_driver::kBarometerCount>, kMaxFingertips>
      baseline_offset_{};
  // Writer = comm/event-loop thread (FeedCalibration/InitFT/ResetCalibration);
  // readers = ROS callback + status timer threads (is_calibrated/calibration_count).
  // relaxed atomics: no cross-field ordering required, just tear-free scalar access.
  std::atomic<int> calibration_count_{0};
  std::atomic<bool> calibrated_{false};

  // input_max 역수 (div → mul 최적화, InitFT()에서 사전 계산)
  std::array<std::array<float, udp_hand_driver::kFTInputSize>, kMaxFingertips>
      input_max_reciprocal_{};

  // 이전 barometer 값 (delta 계산용)
  std::array<std::array<float, udp_hand_driver::kBarometerCount>, kMaxFingertips> prev_barometer_{};
};

#endif  // HAS_ONNXRUNTIME

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_FINGERTIP_FT_INFERENCER_HPP_
