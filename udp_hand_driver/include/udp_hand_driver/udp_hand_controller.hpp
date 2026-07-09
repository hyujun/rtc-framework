#ifndef UDP_HAND_DRIVER_UDP_HAND_CONTROLLER_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_CONTROLLER_HPP_

// High-level hand controller: self-clocked UDP driver.
//
// Uses a single UDP socket for both send and receive. A dedicated
// rtc::PeriodicRtThread (CommLoop) drives one RunCommCycle() per period at a
// fixed `loop_rate_hz` (default 500 Hz) — the read/state-publish cadence is
// autonomous and does NOT depend on how often commands arrive.
//
// Per comm cycle (bulk mode):
//   1. Write position (0x01) + recv echo — ONLY when a command is pending
//   2. Read all motors (0x10)            — every cycle (state)
//   3. Read all sensors (0x19)           — every sensor cycle (state)
// Per comm cycle (individual mode):
//   1. Write position (0x01) + recv echo — ONLY when a command is pending
//   2. Read position  (0x11)
//   3. Read velocity  (0x12)
//   4. Read sensor 0-3 (0x14..0x17) x 4
//
// Write gating: SendCommandAndRequestStates(cmd) stages the command
// (SeqLock) and sets event_pending_. The next cycle latches it, writes once,
// and clears the pending flag (per-command, not a permanent latch). When no
// command has been staged since the last write, cycles are read-only — the
// firmware holds its last commanded position. Under integrated_bringup the
// controller-manager RT loop publishes /…/joint_command only while a
// controller is active, so writes track control_rate and stop on deactivate
// while reads/publishes continue at loop_rate_hz.
//
// RT safety:
//   - All hot-path operations are allocation-free.
//   - No printf/stdout on the CommLoop thread (THROTTLE-only, RT-safe msgs).
//   - Shared state uses rtc::SeqLock (lock-free) instead of mutex.
//   - Command handoff is a lock-free SeqLock store + release-ordered flag;
//     no eventfd / condition_variable on the RT producer path.

#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/types/types.hpp"
#include "udp_hand_driver/fingertip_ft_inferencer.hpp"
#include "udp_hand_driver/protocol/sensor_protocol.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"
#include "udp_hand_driver/udp_hand_sensor_processor.hpp"
#include "udp_hand_driver/udp_hand_state.hpp"
#include "udp_hand_driver/udp_hand_timing_profiler.hpp"
#include "udp_hand_driver/udp_hand_transport.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace udp_hand_driver {

// ── Communication mode
// ────────────────────────────────────────────────────────
enum class HandCommunicationMode {
  kIndividual,  // 0x11 pos + 0x12 vel + 0x14~0x17 sensors
  kBulk,        // 0x10 all motors + 0x19 all sensors
};

// ── Sensor calibration (matches rtc_msgs::msg::CalibrationCommand/Status) ─
// Kept independent of rtc_msgs to avoid adding a message dependency in this
// header. The ROS node is responsible for translating msg enums to these.
namespace calibration {
// Sensor types
constexpr uint8_t kSensorBarometer = 0;
constexpr uint8_t kSensorFtZero = 1;   // reserved
constexpr uint8_t kSensorTofDark = 2;  // reserved
// Actions
constexpr uint8_t kActionStart = 0;
constexpr uint8_t kActionAbort = 1;
// States
constexpr uint8_t kStateIdle = 0;
constexpr uint8_t kStateRunning = 1;
constexpr uint8_t kStateComplete = 2;
constexpr uint8_t kStateFailed = 3;
}  // namespace calibration

struct CalibrationStatusSnapshot {
  uint8_t sensor_type{0};
  uint8_t state{calibration::kStateIdle};
  uint16_t progress_count{0};
  uint16_t target_count{0};
};

// ── Cycle outcome ring (link-down forensics) ─────────────────────────────
// One entry per CommLoop cycle: which request kinds were attempted and which
// succeeded (bit i = RequestKindBit(RequestKind(i))). POD, single writer
// (CommLoop), exposed as a rtc::SeqLock snapshot so the non-RT failure
// detector can decode the last kCapacity cycles at link-down time.
struct CycleOutcomeEntry {
  uint32_t cycle_seq{0};
  uint8_t attempted_mask{0};
  uint8_t ok_mask{0};
};

struct CycleOutcomeRing {
  static constexpr uint32_t kCapacity = 64;
  std::array<CycleOutcomeEntry, kCapacity> entries{};
  uint32_t count{0};  ///< total cycles recorded; latest = entries[(count-1) % kCapacity]
};

// Hand-private UDP receive thread config (Phase 5 — was rtc::kUdpRecvConfig
// in SystemThreadConfigs prior to the SSoT cleanup). cpu_core = -1 means the
// receive thread inherits whatever affinity the launch script's process-level
// `taskset` set on the hand_driver process; SCHED_FIFO 65 preserves the prior
// priority hierarchy (rt_callback 70 > hand UDP 65 > mpc_main 60).
inline const rtc::ThreadConfig kHandUdpRecvConfig{.cpu_core = -1,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 65,
                                                  .nice_value = 0,
                                                  .name = "hand_udp_recv"};

// Construction parameters for UdpHandController (same pattern as
// UdpHandSensorProcessorConfig). Field defaults preserve the historical
// constructor defaults; callers use designated initializers and set only
// what differs.
struct UdpHandControllerConfig {
  std::string target_ip{};
  int target_port{0};
  rtc::ThreadConfig thread_cfg{kHandUdpRecvConfig};
  double recv_timeout_ms{10.0};
  int sensor_decimation{1};
  int num_fingertips{udp_hand_driver::kDefaultNumFingertips};
  bool use_fake_hand{false};
  std::vector<std::string> fingertip_names{};
  HandCommunicationMode communication_mode{HandCommunicationMode::kIndividual};
  bool tof_lpf_enabled{false};
  double tof_lpf_cutoff_hz{15.0};
  bool baro_lpf_enabled{false};
  double baro_lpf_cutoff_hz{30.0};
  FingertipFTInferencer::Config ft_config{};
  bool drift_detection_enabled{false};
  double drift_threshold{5.0};
  int drift_window_size{2500};
  int comm_decimation{1};
  double loop_rate_hz{500.0};
};

class UdpHandController {
 public:
  using StateCallback =
      std::function<void(const UdpHandState&, const udp_hand_driver::FingertipFTState&)>;

  explicit UdpHandController(UdpHandControllerConfig cfg) noexcept
      : thread_cfg_(cfg.thread_cfg),
        loop_rate_hz_(cfg.loop_rate_hz),
        sensor_decimation_(cfg.sensor_decimation < 1 ? 1 : cfg.sensor_decimation),
        comm_decimation_(ClampCommDecimation(cfg.comm_decimation)),
        num_fingertips_(cfg.num_fingertips > kMaxFingertips
                            ? kMaxFingertips
                            : (cfg.num_fingertips < 0 ? 0 : cfg.num_fingertips)),
        use_fake_hand_(cfg.use_fake_hand),
        fingertip_names_(cfg.fingertip_names.empty() ? kDefaultFingertipNames
                                                     : cfg.fingertip_names),
        communication_mode_(cfg.communication_mode),
        ft_config_(std::move(cfg.ft_config)),
        transport_(std::move(cfg.target_ip), cfg.target_port, cfg.recv_timeout_ms),
        sensor_processor_(UdpHandSensorProcessorConfig{
            num_fingertips_, sensor_decimation_, cfg.tof_lpf_enabled, cfg.tof_lpf_cutoff_hz,
            cfg.baro_lpf_enabled, cfg.baro_lpf_cutoff_hz, cfg.drift_detection_enabled,
            cfg.drift_threshold, cfg.drift_window_size}) {
    const int name_count = static_cast<int>(fingertip_names_.size());
    if (name_count < num_fingertips_) {
      num_fingertips_ = name_count;
    }
  }

  ~UdpHandController() { Stop(); }

  UdpHandController(const UdpHandController&) = delete;
  UdpHandController& operator=(const UdpHandController&) = delete;
  UdpHandController(UdpHandController&&) = delete;
  UdpHandController& operator=(UdpHandController&&) = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  [[nodiscard]] bool Start() {
    // Self-clock rate must be positive. PeriodicRtThread::Start() is a silent
    // no-op on a non-positive frequency, so validate explicitly here (off the
    // RT path) and fail Start() loudly rather than launch a dead loop.
    if (!(loop_rate_hz_ > 0.0)) {
      RCLCPP_ERROR(::udp_hand_driver::logging::ControllerLogger(),
                   "loop_rate_hz must be > 0 (got %.3f)", loop_rate_hz_);
      return false;
    }

    // Default to the 1a sensor protocol when the node did not inject one
    // (e.g. direct construction in unit tests). Off the RT path.
    if (!sensor_protocol_) {
      sensor_protocol_ = CreateSensorProtocol("1a");
    }

    // Propagate the protocol's MODE-echo capabilities to the transport before
    // any request runs (InitializeSensors below, then the CommLoop thread).
    // The joint / set-mode gate stays strict on both 1a and 1b; the bulk-sensor
    // gate is separate because 1b's 0x19 response echoes an arbitrary MODE byte
    // (see VerifiesBulkSensorResponseMode()). Set here (off the RT path) so there
    // is no race with the CommLoop thread started further down.
    transport_.set_verify_response_mode(sensor_protocol_->VerifiesResponseMode());
    transport_.set_verify_bulk_sensor_mode(sensor_protocol_->VerifiesBulkSensorResponseMode());

    // Hoist per-loop-lifetime constants off the RunCommCycle hot path. Both are
    // fixed once sensor_protocol_ is set (above) and read every cycle:
    //   is_bulk_       — communication-mode branch (bulk 0x10/0x19 vs individual)
    //   joint_io_mode_ — joint-space I/O MODE (1a=kJoint, 1b=kMotor); drives the
    //                    position write, the joint read, and the E-Stop zero-write
    is_bulk_ = (communication_mode_ == HandCommunicationMode::kBulk);
    joint_io_mode_ = sensor_protocol_->JointIoMode();

    // F/T inferencer initialization. Non-RT init path — verbose diagnostics
    // here are intentional: when FT inference misbehaves at runtime, this
    // dump is the first thing the operator inspects.
    RCLCPP_INFO(::udp_hand_driver::logging::ControllerLogger(),
                "FT config: enabled=%d fingertips=%d paths=%zu cal=%s(%d samples)",
                ft_config_.enabled ? 1 : 0, num_fingertips_, ft_config_.model_paths.size(),
                ft_config_.calibration_enabled ? "on" : "off", ft_config_.calibration_samples);
    if (ft_config_.enabled && num_fingertips_ > 0) {
      for (std::size_t i = 0; i < ft_config_.model_paths.size(); ++i) {
        const auto& p = ft_config_.model_paths[i];
        RCLCPP_INFO(::udp_hand_driver::logging::ControllerLogger(), "FT model[%zu]=%s", i,
                    p.empty() ? "<skipped>" : p.c_str());
      }
      ft_inferencer_ = std::make_unique<FingertipFTInferencer>();
      try {
        ft_inferencer_->InitFT(ft_config_);
        ft_enabled_ = ft_inferencer_->is_initialized();
        RCLCPP_INFO(::udp_hand_driver::logging::ControllerLogger(),
                    "FT ready: active=%d/%d initialized=%d calibrated=%d",
                    ft_inferencer_->num_models(), num_fingertips_,
                    ft_inferencer_->is_initialized() ? 1 : 0,
                    ft_inferencer_->is_calibrated() ? 1 : 0);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(::udp_hand_driver::logging::ControllerLogger(), "FT init FAILED: %s",
                     e.what());
        ft_enabled_ = false;
        ft_inferencer_.reset();
      } catch (...) {
        RCLCPP_ERROR(::udp_hand_driver::logging::ControllerLogger(),
                     "FT init FAILED: unknown exception");
        ft_enabled_ = false;
        ft_inferencer_.reset();
      }
    } else {
      RCLCPP_WARN(::udp_hand_driver::logging::ControllerLogger(),
                  "FT inference SKIPPED: enabled=%d, num_fingertips=%d", ft_config_.enabled ? 1 : 0,
                  num_fingertips_);
    }

    if (use_fake_hand_) {
      running_.store(true, std::memory_order_release);
      start_time_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(::udp_hand_driver::logging::ControllerLogger(),
                  "UdpHandController started in FAKE mode (num_fingertips=%d)", num_fingertips_);
      return true;
    }

    if (!transport_.Open()) {
      RCLCPP_ERROR(::udp_hand_driver::logging::ControllerLogger(), "UDP socket open failed");
      return false;
    }

    // Sensor initialization: NN -> RAW mode
    if (num_fingertips_ > 0) {
      sensor_init_ok_ = transport_.InitializeSensors();
      if (!sensor_init_ok_) {
        RCLCPP_WARN(::udp_hand_driver::logging::ControllerLogger(),
                    "Sensor initialization failed (sensor_init_ok=false)");
      }
    }

    // Sensor processor initialization (rate estimator, filters, drift detector)
    if (num_fingertips_ > 0) {
      sensor_processor_.Init();
      RCLCPP_DEBUG(::udp_hand_driver::logging::ControllerLogger(), "Sensor processor initialized");
    }

    // Reset per-run cycle state so a deactivate→activate restart begins clean:
    // no stale pending write, fresh decimation phase, write re-gated until the
    // first state read (startup no-jump contract).
    has_pending_write_ = false;
    event_pending_.store(false, std::memory_order_release);
    state_read_once_ = false;
    // Seed so the FIRST tick is a comm cycle (++counter reaches comm_decimation_
    // immediately), preserving the "first cycle always communicates" contract —
    // the initial state read is not delayed by comm_decimation_ periods. For
    // decimation=1 this is 0 (every cycle communicates).
    comm_cycle_counter_ = comm_decimation_ - 1;
    sensor_cycle_counter_ = 0;
    outcome_ring_working_ = CycleOutcomeRing{};
    cached_sensor_data_ = {};
    cached_sensor_force_ = {};

    // Reset per-run link/telemetry counters so a deactivate→activate restart
    // (including after an E-Stop self-exit) does not carry stale failure or
    // cycle history into the fresh run — otherwise the failure detector could
    // observe a non-zero consecutive_recv_failures_ / recv_error_count before
    // the first cycle and false-trigger a link-down E-STOP. Off the RT path.
    consecutive_recv_failures_.store(0, std::memory_order_relaxed);
    for (auto& c : channel_consec_fail_) {
      c.store(0, std::memory_order_relaxed);
    }
    cycle_count_.store(0, std::memory_order_relaxed);
    comm_skip_count_.store(0, std::memory_order_relaxed);
    sensor_seq_ = 0;  // CommLoop-thread-only; restart begins fresh at 0 so the
                      // detector's prev_sensor_seq_ (also 0) sees the first read.
    transport_.ResetCommStats();

    running_.store(true, std::memory_order_release);
    start_time_ = std::chrono::steady_clock::now();

    // Self-clocked comm loop: PeriodicRtThread ticks RunCommCycle() at
    // loop_rate_hz_ (clock_nanosleep TIMER_ABSTIME + overrun recovery). The
    // per-tick RtTickTimingPayload is pushed by the base after each OnTick
    // (see MarkStateAcquired/MarkComputeDone in RunCommCycle).
    comm_loop_.SetTimingProducer(timing_producer_);
    comm_loop_.Start({.thread_config = thread_cfg_, .frequency_hz = loop_rate_hz_});
    RCLCPP_INFO(::udp_hand_driver::logging::ControllerLogger(),
                "CommLoop thread started (mode=%s, loop_rate=%.1f Hz)",
                is_bulk_ ? "bulk" : "individual", loop_rate_hz_);

    return true;
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    if (use_fake_hand_) {
      return;
    }
    // Join() requests stop (cooperative stop_token) and waits for the CommLoop
    // to exit its current tick + clock_nanosleep. Idempotent; safe on a loop
    // that already self-stopped via the E-Stop path (RequestStop in RunCommCycle).
    comm_loop_.Join();
    transport_.Close();
    const auto cycles = cycle_count_.load(std::memory_order_relaxed);
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    const double avg_rate_hz =
        (elapsed_sec > 0.0) ? static_cast<double>(cycles) / elapsed_sec : 0.0;
    RCLCPP_INFO(::udp_hand_driver::logging::ControllerLogger(),
                "UdpHandController stopped: %zu cycles in %.1fs (avg=%.1f Hz)", cycles, elapsed_sec,
                avg_rate_hz);
  }

  // ── Sensor protocol injection (non-RT — call before Start()) ───────────

  /// Inject the firmware sensor protocol (1a/1b). Must be called before Start();
  /// if left unset, Start() defaults to 1a for backward compatibility. Start()
  /// hoists its capabilities (JointIoMode, MODE-verify) before launching the
  /// CommLoop, so it must not change while running.
  void SetSensorProtocol(std::unique_ptr<SensorProtocol> proto) noexcept {
    sensor_protocol_ = std::move(proto);
  }

  // ── Callback ───────────────────────────────────────────────────────────

  void SetCallback(StateCallback cb) noexcept { callback_ = std::move(cb); }

  // ── Timing producer (per-tick CSV output, optional) ────────────────────

  /// Inject a producer for the unified per-tick timing CSV (see
  /// rtc/timing/rt_tick_timing_sample.hpp). Owned by the caller; must
  /// outlive this controller. Pass nullptr to disable. Call before Start()
  /// (Start() forwards it to the CommLoop base). Jitter is computed by the
  /// CommLoop base against its own loop_rate_hz_ period budget.
  void SetTimingProducer(rtc::HandUdpTimingBuffer* producer) noexcept {
    timing_producer_ = producer;
  }

  [[nodiscard]] rtc::HandUdpTimingBuffer* TimingProducer() const noexcept {
    return timing_producer_;
  }

  // ── E-Stop flag (shared with RtControllerNode) ─────────────────────────

  void SetEstopFlag(std::atomic<bool>* flag) noexcept { estop_flag_ = flag; }

  // ── State readiness ─────────────────────────────────────────────────────

  /// True after at least one successful state read from the hand hardware.
  /// Write commands are suppressed until this returns true (startup no-jump
  /// contract — see the write gate in RunCommCycle). Reset to false on each
  /// Start() so a restart re-reads before writing.
  [[nodiscard]] bool HasStateBeenRead() const noexcept { return state_read_once_; }

  // ── Command API (ROS command sub / CM ControlLoop → self-clocked loop) ──

  void SendCommandAndRequestStates(const std::array<float, kNumHandMotors>& cmd) noexcept {
    // Fake mode: echo-back
    if (use_fake_hand_) {
      UdpHandState fake_state{};
      std::copy(cmd.begin(), cmd.end(), fake_state.motor_positions.begin());
      std::copy(cmd.begin(), cmd.end(), fake_state.joint_positions.begin());
      fake_state.num_fingertips = num_fingertips_;
      fake_state.valid = true;
      fake_state.joint_valid = true;
      fake_state.motor_valid = true;

      // Mirror the command into the 1b force channel (fx,fy,fz per fingertip) so
      // the force pipeline — sensor publish + failure detector — is exercisable
      // in standalone fake mode (protocol_version "1b" + use_fake_hand). Harmless
      // on 1a, where sensor_force is unused. Consumers read only fx,fy,fz.
      for (int f = 0; f < num_fingertips_ && f < udp_hand_driver::kMaxFingertips; ++f) {
        for (int j = 0; j < 3; ++j) {
          const auto src = static_cast<std::size_t>((f * 3 + j) % kNumHandMotors);
          const auto dst = static_cast<std::size_t>(f) *
                               static_cast<std::size_t>(udp_hand_driver::kP1bValuesPerFingertip) +
                           static_cast<std::size_t>(j);
          fake_state.sensor_force[dst] = cmd[src];
        }
      }

      if (ft_enabled_ && ft_inferencer_) {
        if (!ft_inferencer_->is_calibrated()) {
          static_cast<void>(
              ft_inferencer_->FeedCalibration(fake_state.sensor_data, num_fingertips_));
        } else {
          auto ft_result = ft_inferencer_->Infer(fake_state.sensor_data, num_fingertips_);
          ft_seqlock_.Store(ft_result);
        }
      }

      // Each fake tick mirrors a fresh sensor read, so advance the freshness
      // sequence — otherwise the detector's freshness gate would treat every
      // fake snapshot as a stale republish and never evaluate the sensor path.
      ++sensor_seq_;
      fake_state.sensor_seq = sensor_seq_;
      state_seqlock_.Store(fake_state);
      if (callback_) {
        callback_(fake_state, ft_seqlock_.Load());
      }
      cycle_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    // RT-safe handoff to the self-clocked CommLoop: lock-free SeqLock store +
    // release-ordered pending flag. No wake primitive — the loop is already
    // ticking at loop_rate_hz_ and latches the flag at the top of its next
    // cycle (last-command-wins; a burst before the next tick collapses to the
    // latest). Per-command, not a permanent latch: the cycle clears it after
    // one write attempt, so an uncommanded interval is read-only.
    staged_cmd_seqlock_.Store(cmd);
    event_pending_.store(true, std::memory_order_release);
  }

  // ── Sensor calibration trigger (ROS thread -> CommLoop handoff) ────────

  /// Request a sensor calibration action. Safe to call from ROS subscriber
  /// threads. The actual reset/state mutation happens on the CommLoop
  /// thread, so existing sensor data structures stay single-threaded.
  /// Only one request is in-flight at a time; additional requests before the
  /// CommLoop consumes the pending one will overwrite it (last-wins).
  void RequestCalibration(uint8_t sensor_type, uint8_t action, uint16_t sample_count) noexcept {
    pending_calib_.sensor_type = sensor_type;
    pending_calib_.action = action;
    pending_calib_.sample_count = sample_count;
    calib_request_pending_.store(true, std::memory_order_release);
  }

  /// Snapshot of the latest calibration progress for a given sensor.
  /// Safe to call from any thread (reads CommLoop-owned state via snapshot
  /// accessors that are either atomic or only mutated in known contexts).
  [[nodiscard]] CalibrationStatusSnapshot GetCalibrationStatus(uint8_t sensor_type) const noexcept {
    CalibrationStatusSnapshot snap{};
    snap.sensor_type = sensor_type;
    switch (sensor_type) {
      case calibration::kSensorBarometer: {
        if (!ft_enabled_ || !ft_inferencer_) {
          snap.state = calibration::kStateFailed;
          break;
        }
        const int cnt = ft_inferencer_->calibration_count();
        const int target = ft_inferencer_->calibration_target();
        snap.progress_count = static_cast<uint16_t>(cnt < 0 ? 0 : cnt);
        snap.target_count = static_cast<uint16_t>(target < 0 ? 0 : target);
        if (ft_inferencer_->is_calibrated()) {
          snap.state = calibration::kStateComplete;
        } else if (cnt > 0) {
          snap.state = calibration::kStateRunning;
        } else {
          snap.state = calibration::kStateIdle;
        }
        break;
      }
      default:
        snap.state = calibration::kStateFailed;
        break;
    }
    return snap;
  }

  // ── State access ───────────────────────────────────────────────────────

  [[nodiscard]] UdpHandState GetLatestState() const noexcept { return state_seqlock_.Load(); }

  [[nodiscard]] std::array<float, kNumHandMotors> GetLatestPositions() const noexcept {
    return state_seqlock_.Load().motor_positions;
  }

  [[nodiscard]] bool IsRunning() const noexcept { return running_.load(std::memory_order_acquire); }

  // ── F/T inference accessors ──────────────────────────────────────────────

  [[nodiscard]] udp_hand_driver::FingertipFTState GetLatestFTState() const noexcept {
    return ft_seqlock_.Load();
  }

  [[nodiscard]] bool ft_inference_enabled() const noexcept { return ft_enabled_; }

  [[nodiscard]] bool IsSensorInitialized() const noexcept { return sensor_init_ok_; }

  [[nodiscard]] std::size_t cycle_count() const noexcept {
    return cycle_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] uint64_t recv_error_count() const noexcept { return transport_.recv_error_count(); }

  [[nodiscard]] int comm_decimation() const noexcept { return comm_decimation_; }

  [[nodiscard]] uint64_t comm_skip_count() const noexcept {
    return comm_skip_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] uint64_t consecutive_recv_failures() const noexcept {
    return consecutive_recv_failures_.load(std::memory_order_relaxed);
  }

  // Per-channel consecutive-failure streak (attempts of `kind` that failed in a
  // row). For forensics / detector messages. Reader-thread safe (relaxed load).
  [[nodiscard]] uint32_t channel_consec_fail(RequestKind kind) const noexcept {
    return channel_consec_fail_[static_cast<std::size_t>(kind)].load(std::memory_order_relaxed);
  }

  // Strict per-channel link-down decision (#1): true when ANY periodically-polled
  // channel's consecutive-failure streak has reached its threshold, so a single
  // dead channel latches even while others stay healthy. Single source of the
  // decision so detector / publish / stats cannot drift. Snapshots the atomics
  // and delegates to the pure LinkDownDecision (directly unit-tested).
  [[nodiscard]] bool LinkDown(uint32_t comm_threshold, uint32_t sensor_threshold) const noexcept {
    std::array<uint32_t, kNumRequestKinds> fails{};
    for (int k = 0; k < kNumRequestKinds; ++k) {
      fails[static_cast<std::size_t>(k)] =
          channel_consec_fail_[static_cast<std::size_t>(k)].load(std::memory_order_relaxed);
    }
    return LinkDownDecision(fails, comm_threshold, sensor_threshold);
  }

  // Pure per-channel link-down policy. Motor/joint are polled every comm cycle →
  // compared to comm_threshold; sensor (individual kSensorRead or bulk
  // kBulkSensor) is polled every sensor_decimation-th cycle → compared to the
  // shorter sensor_threshold. write-echo / set-mode are command/config-driven
  // (not periodic health polls) and are EXCLUDED — their cadence is unpredictable,
  // so a cycles-based timeout would false-latch during a legitimately idle
  // interval; a genuinely dead link still latches via the read channels. A
  // threshold of 0 disables that channel class. Static + input-only so it can be
  // unit-tested with crafted streak vectors.
  [[nodiscard]] static bool LinkDownDecision(const std::array<uint32_t, kNumRequestKinds>& fails,
                                             uint32_t comm_threshold,
                                             uint32_t sensor_threshold) noexcept {
    const auto latched = [&fails](RequestKind kind, uint32_t thr) noexcept {
      return thr > 0 && fails[static_cast<std::size_t>(kind)] >= thr;
    };
    return latched(RequestKind::kMotorRead, comm_threshold) ||
           latched(RequestKind::kJointRead, comm_threshold) ||
           latched(RequestKind::kSensorRead, sensor_threshold) ||
           latched(RequestKind::kBulkSensor, sensor_threshold);
  }

  [[nodiscard]] UdpHandCommStats comm_stats() const noexcept {
    UdpHandCommStats stats = transport_.comm_stats();
    stats.comm_decimation_skip_count = comm_skip_count_.load(std::memory_order_relaxed);
    return stats;
  }

  /// Snapshot of the last CycleOutcomeRing::kCapacity CommLoop cycle outcomes
  /// (attempted/ok request-kind masks). Safe from any thread (SeqLock).
  [[nodiscard]] CycleOutcomeRing GetCycleOutcomeRing() const noexcept {
    return outcome_ring_seqlock_.Load();
  }

  [[nodiscard]] UdpHandTimingProfiler::Stats timing_stats() const noexcept {
    return timing_profiler_.GetStats();
  }

  [[nodiscard]] std::string TimingSummary() const noexcept { return timing_profiler_.Summary(); }

  [[nodiscard]] HandCommunicationMode communication_mode() const noexcept {
    return communication_mode_;
  }

  [[nodiscard]] double recv_timeout_ms() const noexcept { return transport_.recv_timeout_ms(); }

  [[nodiscard]] double actual_sensor_rate_hz() const noexcept {
    return sensor_processor_.actual_sensor_rate_hz();
  }

 private:
  // ── Self-clocked comm loop (PeriodicRtThread subclass) ───────────────────
  // CommLoop ticks OnTick() -> owner_->RunCommCycle() once per loop_rate_hz_
  // period (clock_nanosleep TIMER_ABSTIME + overrun recovery in the base). The
  // base captures t0 before OnTick and t3 after; MarkState()/MarkCompute()
  // forward the intra-tick breakpoints so the base can push one
  // RtTickTimingPayload per tick. Nested so it can reach the owner's private
  // members through the back-pointer.
  class CommLoop final : public rtc::PeriodicRtThread {
   public:
    explicit CommLoop(UdpHandController* owner) noexcept : owner_(owner) {}

    // Forwarders: RunCommCycle lives on the owner (not this subclass), so it
    // cannot reach the base's protected MarkStateAcquired/MarkComputeDone
    // directly. These expose them for the state / compute breakpoints.
    void MarkState() noexcept { MarkStateAcquired(); }

    void MarkCompute() noexcept { MarkComputeDone(); }

    // Forwarders for the base's protected RT-safe controls, so RunCommCycle
    // (which lives on the owner) can drive them: RequestLoopExit() is the
    // pure-atomic self-exit used by the E-Stop path (no pause-mutex / cv);
    // SuppressTiming() drops the current tick's timing row (skip / E-Stop).
    void RequestLoopExit() noexcept { PeriodicRtThread::RequestLoopExit(); }

    void SuppressTiming() noexcept { SuppressTimingThisTick(); }

   protected:
    void OnTick() override { owner_->RunCommCycle(); }

    // Terminal-exit hook: the base invokes this once on the loop thread after
    // RunCommCycle requested RequestLoopExit() (E-Stop). Performs the zero-write
    // off the RT hot path (moved out of the E-Stop branch per review #2-A).
    void OnLoopAborted() noexcept override { owner_->OnCommLoopAborted(); }

   private:
    UdpHandController* owner_;
  };

  // One self-clocked comm cycle — the CommLoop tick body. Runs on the CommLoop
  // SCHED_FIFO thread every loop_rate_hz_ period (RT hot path). Order:
  //   decimation decision → E-Stop → (skip? return) → latch command →
  //   command-gated write → reads → sensor post-process → publish → forensics.
  // Allocation-free; logging is THROTTLE-only with RT-safe (primitive-arg) msgs.
  void RunCommCycle() noexcept {
    // Whole-cycle UDP decimation: only every comm_decimation_-th cycle runs the
    // UDP transaction (default 1 = every cycle). Decided first so the E-Stop
    // check below still runs on every tick, decimated or not.
    ++comm_cycle_counter_;
    const bool is_comm_cycle = (comm_cycle_counter_ >= comm_decimation_);
    if (is_comm_cycle) {
      comm_cycle_counter_ = 0;
    }

    // E-Stop (checked every tick, before the decimation skip): zero-write once
    // and self-stop the loop. Terminal — semantically the old EventLoop `break`.
    if (estop_flag_ && estop_flag_->load(std::memory_order_acquire)) {
      static rclcpp::Clock estop_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(::udp_hand_driver::logging::ControllerLogger(), estop_clock,
                           ::udp_hand_driver::logging::kThrottleFastMs,
                           "RunCommCycle: E-Stop active, sending zero and stopping loop");
      // #4 Mark not-running so command / calibration subscriber callbacks (which
      // gate on IsRunning()) stop feeding a loop that is about to exit.
      running_.store(false, std::memory_order_release);
      // #5 This terminal cycle does no representative I/O — keep its degenerate
      // phase timings out of the per-tick timing CSV.
      comm_loop_.SuppressTiming();
      // #2-A RT-safe self-exit: a pure atomic store (no pause-mutex / condition
      // variable, unlike RequestStop()). The base drops out of its loop after
      // this tick and runs OnLoopAborted() → OnCommLoopAborted(), which performs
      // the zero-write off this RT hot path.
      comm_loop_.RequestLoopExit();
      return;
    }

    // Decimated skip cycle: no UDP I/O, no publish, no cycle accounting. The
    // staged command (if any) stays latched in event_pending_ for the next real
    // comm cycle.
    if (!is_comm_cycle) {
      comm_skip_count_.fetch_add(1, std::memory_order_relaxed);
      // #5 A decimated skip cycle does no UDP I/O — keep its degenerate phase
      // timings out of the per-tick timing CSV.
      comm_loop_.SuppressTiming();
      return;
    }

    // Drain any datagram left queued from a prior cycle before this cycle's
    // first request. A 1-cycle desync parks the previous request's late response
    // in the recv buffer; without draining it, the first read below would
    // consume that stale packet (cmd_mismatch) and burn retries. Bounded
    // non-blocking drain — the count is observed via comm_stats().stale_drained
    // only (no hot-path logging). Runs only on real comm cycles (after the
    // decimation skip return above); a no-op in fake mode (socket closed).
    transport_.DrainStaleDatagrams();

    // Latch the most recent staged command (last-command-wins). Per-command:
    // cleared after one write attempt below, so an uncommanded interval is
    // read-only and the firmware holds its last position.
    if (event_pending_.exchange(false, std::memory_order_acquire)) {
      pending_cmd_ = staged_cmd_seqlock_.Load();
      has_pending_write_ = true;
    }

    UdpHandState state{};
    bool any_recv_ok = false;
    uint8_t attempted_mask = 0;
    uint8_t ok_mask = 0;
    std::array<int32_t, udp_hand_driver::kMaxHandSensors> cached_sensor_data_raw{};

    const auto t0 = std::chrono::steady_clock::now();

    // 1. Write position + recv echo (joint_io_mode_: 1a=kJoint, 1b=kMotor).
    // Gated on BOTH the first state read (startup no-jump contract) AND a
    // pending command. has_pending_write_ is cleared only here, inside the open
    // gate, so a command staged before the first state read is held (not lost)
    // until state_read_once_ flips true. Success/failure both clear — a dropped
    // write is not re-sent stale (control_rate delivers a fresh one).
    if (state_read_once_ && has_pending_write_) {
      attempted_mask |= RequestKindBit(RequestKind::kWriteEcho);
      if (transport_.WritePositionWithEcho(pending_cmd_, send_buf_, echo_buf_, joint_io_mode_)) {
        any_recv_ok = true;
        ok_mask |= RequestKindBit(RequestKind::kWriteEcho);
      }
      has_pending_write_ = false;
    }

    const auto t1 = std::chrono::steady_clock::now();

    // Sensor decimation
    ++sensor_cycle_counter_;
    const bool is_sensor_cycle = (sensor_cycle_counter_ >= sensor_decimation_);
    if (is_sensor_cycle)
      sensor_cycle_counter_ = 0;

    if (is_bulk_) {
      // ── Bulk mode ─────────────────────────────────────────────────────
      // 2. Read all motors (motor-space: kMotor). Skipped when the protocol
      //    has no motor-space read (1b) — motor_states is then not published.
      if (sensor_protocol_->HasMotorSpaceRead()) {
        packets::JointMode received_mode{};
        attempted_mask |= RequestKindBit(RequestKind::kMotorRead);
        if (transport_.RequestAllMotorRead(motor_pos_buf_, motor_vel_buf_, motor_cur_buf_,
                                           packets::JointMode::kMotor, &received_mode,
                                           RequestKind::kMotorRead)) {
          std::copy_n(motor_pos_buf_.begin(), kNumHandMotors, state.motor_positions.begin());
          std::copy_n(motor_vel_buf_.begin(), kNumHandMotors, state.motor_velocities.begin());
          std::copy_n(motor_cur_buf_.begin(), kNumHandMotors, state.motor_currents.begin());
          state.received_joint_mode = static_cast<uint8_t>(received_mode);
          state.motor_valid = true;
          any_recv_ok = true;
          ok_mask |= RequestKindBit(RequestKind::kMotorRead);
        }
      }

      const auto t2 = std::chrono::steady_clock::now();

      // 3. Read all motors (joint-space: joint_io_mode_) — pos/vel/cur. 1b
      //    issues this in kMotor (its only joint-serving mode) and still
      //    publishes the result as joint_states.
      {
        std::array<float, packets::kMotorDataCount> jp_buf{}, jv_buf{}, jc_buf{};
        attempted_mask |= RequestKindBit(RequestKind::kJointRead);
        if (transport_.RequestAllMotorRead(jp_buf, jv_buf, jc_buf, joint_io_mode_, nullptr,
                                           RequestKind::kJointRead)) {
          std::copy_n(jp_buf.begin(), kNumHandMotors, state.joint_positions.begin());
          std::copy_n(jv_buf.begin(), kNumHandMotors, state.joint_velocities.begin());
          std::copy_n(jc_buf.begin(), kNumHandMotors, state.joint_currents.begin());
          state.joint_valid = true;
          any_recv_ok = true;
          ok_mask |= RequestKindBit(RequestKind::kJointRead);
        }
      }

      const auto t2j = std::chrono::steady_clock::now();

      // 4. Read all sensors — raw recv, decode via the injected protocol
      //    (version seam). The request is identical across versions; only the
      //    response size/layout and decode differ.
      if (is_sensor_cycle) {
        state.num_fingertips = num_fingertips_;  // decode input
        attempted_mask |= RequestKindBit(RequestKind::kBulkSensor);
        const ssize_t recvd = transport_.RequestBulkSensorRaw(
            sensor_recv_buf_.data(), sensor_recv_buf_.size(), sensor_protocol_->ResponseSize(),
            packets::SensorMode::kRaw);
        if (recvd >= 0 && sensor_protocol_->DecodeAllSensors(
                              sensor_recv_buf_.data(), static_cast<std::size_t>(recvd), state)) {
          any_recv_ok = true;
          ok_mask |= RequestKindBit(RequestKind::kBulkSensor);
          cached_sensor_data_ = state.sensor_data;    // 1a raw decode (0 for 1b)
          cached_sensor_force_ = state.sensor_force;  // 1b force (0 for 1a)
          ++sensor_seq_;  // fresh validated sensor read (freshness gate for detector)
        }
      }

      const auto t3 = std::chrono::steady_clock::now();
      comm_loop_.MarkState();  // end of UDP read phase (base t1)

      // Bulk-only state field (1b firmware-computed force; 0 for 1a). Set before
      // the shared tail so the SeqLock store captures it.
      state.sensor_force = cached_sensor_force_;
      const CommCycleTailResult tail =
          RunCommCycleTail(state, cached_sensor_data_raw, any_recv_ok, is_sensor_cycle);

      UdpHandTimingProfiler::PhaseTiming pt;
      pt.is_bulk_mode = true;
      pt.write_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      pt.read_all_motor_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
      pt.read_all_joint_motor_us = std::chrono::duration<double, std::micro>(t2j - t2).count();
      pt.read_all_sensor_us = std::chrono::duration<double, std::micro>(t3 - t2j).count();
      FinalizePhaseTiming(pt, tail, t3, t0, is_sensor_cycle);

    } else {
      // ── Individual mode ───────────────────────────────────────────────
      // 2. Read motor position (kMotor)
      packets::JointMode received_mode{};
      attempted_mask |= RequestKindBit(RequestKind::kMotorRead);
      if (transport_.RequestMotorRead(packets::Command::kReadPosition, motor_pos_buf_,
                                      packets::JointMode::kMotor, &received_mode,
                                      RequestKind::kMotorRead)) {
        std::copy_n(motor_pos_buf_.begin(), kNumHandMotors, state.motor_positions.begin());
        state.received_joint_mode = static_cast<uint8_t>(received_mode);
        state.motor_valid = true;
        any_recv_ok = true;
        ok_mask |= RequestKindBit(RequestKind::kMotorRead);
      }

      const auto t2 = std::chrono::steady_clock::now();

      // 3. Read joint position (joint-space: joint_io_mode_). Individual mode is
      //    1a-only (1b requires bulk), so this resolves to kJoint in practice.
      {
        std::array<float, packets::kMotorDataCount> jp_buf{};
        attempted_mask |= RequestKindBit(RequestKind::kJointRead);
        if (transport_.RequestMotorRead(packets::Command::kReadPosition, jp_buf, joint_io_mode_,
                                        nullptr, RequestKind::kJointRead)) {
          std::copy_n(jp_buf.begin(), kNumHandMotors, state.joint_positions.begin());
          state.joint_valid = true;
          any_recv_ok = true;
          ok_mask |= RequestKindBit(RequestKind::kJointRead);
        }
      }

      const auto t2j = std::chrono::steady_clock::now();

      // 4. Read motor velocity (kMotor)
      attempted_mask |= RequestKindBit(RequestKind::kMotorRead);
      if (transport_.RequestMotorRead(packets::Command::kReadVelocity, motor_vel_buf_,
                                      packets::JointMode::kMotor, nullptr,
                                      RequestKind::kMotorRead)) {
        std::copy_n(motor_vel_buf_.begin(), kNumHandMotors, state.motor_velocities.begin());
        any_recv_ok = true;
        ok_mask |= RequestKindBit(RequestKind::kMotorRead);
      }

      const auto t3 = std::chrono::steady_clock::now();

      // 5. Read sensors
      if (is_sensor_cycle) {
        if (num_fingertips_ > 0) {
          attempted_mask |= RequestKindBit(RequestKind::kSensorRead);
        }
        bool sensor_validated = false;
        for (int i = 0; i < num_fingertips_; ++i) {
          auto cmd = packets::SensorCommand(i);
          if (transport_.RequestSensorRead(cmd, sensor_raw_buf_, packets::SensorMode::kRaw)) {
            std::copy_n(
                sensor_raw_buf_.begin(), udp_hand_driver::kSensorValuesPerFingertip,
                cached_sensor_data_.begin() + i * udp_hand_driver::kSensorValuesPerFingertip);
            any_recv_ok = true;
            ok_mask |= RequestKindBit(RequestKind::kSensorRead);
            sensor_validated = true;
          }
        }
        if (sensor_validated)
          ++sensor_seq_;  // one bump per cycle with ≥1 fresh finger read
      }

      const auto t4 = std::chrono::steady_clock::now();
      comm_loop_.MarkState();  // end of UDP read phase (base t1)

      const CommCycleTailResult tail =
          RunCommCycleTail(state, cached_sensor_data_raw, any_recv_ok, is_sensor_cycle);

      UdpHandTimingProfiler::PhaseTiming pt;
      pt.write_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      pt.read_pos_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
      pt.read_joint_pos_us = std::chrono::duration<double, std::micro>(t2j - t2).count();
      pt.read_vel_us = std::chrono::duration<double, std::micro>(t3 - t2j).count();
      pt.read_sensor_us = std::chrono::duration<double, std::micro>(t4 - t3).count();
      FinalizePhaseTiming(pt, tail, t4, t0, is_sensor_cycle);
    }

    // cycle_count_ is the SSoT comm-cycle counter (read by the rate detector).
    // comm_stats.total_cycles mirrors it for the forensic JSON — derive it from
    // the fetch_add result instead of maintaining a second independent counter
    // that could drift. cycle_seq is the pre-increment value; total_cycles wants
    // the post-increment count.
    const auto cycle_seq = cycle_count_.fetch_add(1, std::memory_order_relaxed);
    transport_.comm_stats_mut().total_cycles = cycle_seq + 1;

    // Record this cycle's outcome into the working forensic ring (cheap in-place
    // write, every cycle).
    outcome_ring_working_.entries[outcome_ring_working_.count % CycleOutcomeRing::kCapacity] = {
        static_cast<uint32_t>(cycle_seq), attempted_mask, ok_mask};
    ++outcome_ring_working_.count;

    // Per-channel consecutive-failure tracking (strict link-down, #1). A channel
    // attempted-but-failed this cycle increments; attempted-and-ok resets; a
    // channel NOT attempted this cycle (sensor on a non-sensor cycle, motor-space
    // on 1b, write-echo with no pending command) is left unchanged so its streak
    // counts only real attempts. any_channel_failing feeds the forensic-ring
    // publish gate so a partial (single-channel) loss still captures forensics —
    // strictly a superset of the old aggregate-only gate. RT-safe: bounded loop,
    // relaxed atomics, no alloc.
    bool any_channel_failing = false;
    for (int k = 0; k < kNumRequestKinds; ++k) {
      const auto bit = static_cast<uint8_t>(1u << static_cast<unsigned>(k));
      if (attempted_mask & bit) {
        if (ok_mask & bit) {
          channel_consec_fail_[static_cast<std::size_t>(k)].store(0, std::memory_order_relaxed);
        } else {
          channel_consec_fail_[static_cast<std::size_t>(k)].fetch_add(1, std::memory_order_relaxed);
        }
      }
      if (channel_consec_fail_[static_cast<std::size_t>(k)].load(std::memory_order_relaxed) > 0) {
        any_channel_failing = true;
      }
    }

    // #6 Publish the SeqLock snapshot (~0.5 KB memcpy) conditionally: the
    // off-loop forensic reader only consults it when the link is unhealthy, so
    // on the steady healthy path the store is redundant work. Store when this
    // cycle failed, when ANY channel is mid-failure-streak, or every kCapacity-th
    // cycle as a heartbeat so a healthy snapshot never goes fully stale. The
    // working ring still accumulates every cycle regardless.
    const bool publish_outcome_ring =
        !any_recv_ok || any_channel_failing || (cycle_seq % CycleOutcomeRing::kCapacity == 0);
    if (publish_outcome_ring) {
      outcome_ring_seqlock_.Store(outcome_ring_working_);
    }

    if (any_recv_ok) {
      consecutive_recv_failures_.store(0, std::memory_order_relaxed);
    } else {
      const auto failures = consecutive_recv_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
      // Decode the last-unexpected forensics via lightweight atomic loads rather
      // than copying the full comm_stats() snapshot on this hot path.
      const auto last_cmd = transport_.last_unexpected_cmd();
      const auto last_len = transport_.last_unexpected_len();
      if (failures == 1) {
        // 0→1 transition: name the FIRST failed request kind of this cycle
        // and the last rejected CMD byte — a stale echo of the *previous*
        // request implies desync rather than timeout. Throttled; static
        // string literal + primitive args only (RT-safe).
        const auto failed_mask = static_cast<uint8_t>(attempted_mask & ~ok_mask);
        const int first_failed = (failed_mask != 0) ? std::countr_zero(failed_mask) : -1;
        // Decode the attempted mask into per-kind (cmd, mode) wire bytes so the
        // onset line names what was on the wire rather than an opaque hex mask.
        // Stack buffer, alloc-free (RT-safe).
        char attempted_desc[256];
        FormatRequestMask(attempted_mask, joint_io_mode_, is_bulk_, attempted_desc,
                          sizeof(attempted_desc));
        static rclcpp::Clock onset_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
            ::udp_hand_driver::logging::ControllerLogger(), onset_clock,
            ::udp_hand_driver::logging::kThrottleFastMs,
            "RunCommCycle: recv failure onset: first_failed=%s attempted=0x%02X [%s] ok=0x%02X "
            "last_unexpected_cmd=0x%02X last_unexpected_len=%u",
            (first_failed >= 0 && first_failed < kNumRequestKinds)
                ? kRequestKindNames[static_cast<std::size_t>(first_failed)]
                : "none",
            static_cast<unsigned>(attempted_mask), attempted_desc, static_cast<unsigned>(ok_mask),
            static_cast<unsigned>(last_cmd), static_cast<unsigned>(last_len));
      } else if (failures >= 5) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(::udp_hand_driver::logging::ControllerLogger(), steady_clock,
                             ::udp_hand_driver::logging::kThrottleFastMs,
                             "RunCommCycle: consecutive recv failures=%lu (attempted=0x%02X "
                             "ok=0x%02X last_unexpected_cmd=0x%02X)",
                             static_cast<unsigned long>(failures),
                             static_cast<unsigned>(attempted_mask), static_cast<unsigned>(ok_mask),
                             static_cast<unsigned>(last_cmd));
      }
    }

    // #7 Publish the cycle's comm-stats working copy as one consistent SeqLock
    // snapshot. All this cycle's transport mutations (per_kind, aggregates,
    // last_unexpected) and the total_cycles stamp above are now captured, so an
    // off-loop reader never sees a torn per_kind/aggregate pair. RT-safe memcpy.
    transport_.PublishCommStats();
  }

  // Terminal E-Stop zero-write, invoked once by CommLoop::OnLoopAborted() on
  // the loop thread as the loop unwinds (moved out of RunCommCycle's E-Stop
  // branch per review #2-A so the hot path stays free of the write + its recv).
  // Fire-and-forget: no echo wait, allocation-free, noexcept.
  void OnCommLoopAborted() noexcept {
    std::array<float, kNumHandMotors> zeros{};
    transport_.WritePositionFireAndForget(zeros, joint_io_mode_);
  }

  // Intermediate timestamps + FT cost from RunCommCycleTail, so each mode's
  // caller can fill its own PhaseTiming (the phase fields differ per mode).
  struct CommCycleTailResult {
    std::chrono::steady_clock::time_point compute_done;   // end of sensor post-process
    std::chrono::steady_clock::time_point callback_done;  // end of store + callback
    double ft_infer_us{0.0};
  };

  // Common bulk/individual cycle tail (CommLoop thread only — no sync). Runs the
  // gated sensor post-process (PreFilter/ApplyFilters/DetectDrift/RunFTInference),
  // MarkCompute, shared state finalize, the first-state latch + log, then the
  // SeqLock store and callback. The caller has already captured the read-end
  // timestamp and called MarkState(); mode-specific state fields (bulk's
  // sensor_force) must be set on `state` before calling. The post-process gate is
  // unified to `is_sensor_cycle && RunsSensorPostProcess()`: individual mode is
  // 1a-only (the node rejects 1b+individual), where RunsSensorPostProcess() is
  // always true, so the added predicate is behavior-neutral there.
  // noexcept + allocation-free (rt-path.md compliant).
  [[nodiscard]] CommCycleTailResult RunCommCycleTail(
      UdpHandState& state, std::array<int32_t, udp_hand_driver::kMaxHandSensors>& sensor_data_raw,
      bool any_recv_ok, bool is_sensor_cycle) noexcept {
    CommCycleTailResult result{};
    if (is_sensor_cycle && sensor_protocol_->RunsSensorPostProcess()) {
      sensor_data_raw = cached_sensor_data_;
      sensor_processor_.PreFilter();
      sensor_processor_.ApplyFilters(cached_sensor_data_);
      sensor_processor_.DetectDrift(sensor_data_raw);
      result.ft_infer_us = RunFTInference(cached_sensor_data_);
    }

    result.compute_done = std::chrono::steady_clock::now();
    comm_loop_.MarkCompute();  // end of compute phase (base t2)

    state.sensor_data_raw = sensor_data_raw;
    state.sensor_data = cached_sensor_data_;
    state.num_fingertips = num_fingertips_;
    state.valid = any_recv_ok;
    state.sensor_seq = sensor_seq_;  // freshness gate for the failure detector

    if (any_recv_ok && !state_read_once_) {
      state_read_once_ = true;
      // Throttled as a defensive RT-safety net; the state_read_once_ gate
      // already guarantees this fires at most once per Start().
      static rclcpp::Clock first_state_clock(RCL_STEADY_TIME);
      RCLCPP_INFO_THROTTLE(::udp_hand_driver::logging::ControllerLogger(), first_state_clock,
                           ::udp_hand_driver::logging::kThrottleIdleMs,
                           "First hand state received (write commands now enabled)");
    }

    state_seqlock_.Store(state);
    if (callback_) {
      callback_(state, ft_seqlock_.Load());
    }
    result.callback_done = std::chrono::steady_clock::now();
    return result;
  }

  // Fill the mode-independent tail of a PhaseTiming record and push it to the
  // profiler. Bulk and individual modes differ only in the read-end timestamp
  // that anchors sensor_proc_us (bulk t3, individual t4) — passed as
  // sensor_proc_base. CommLoop thread only; noexcept + allocation-free.
  void FinalizePhaseTiming(UdpHandTimingProfiler::PhaseTiming& pt, const CommCycleTailResult& tail,
                           std::chrono::steady_clock::time_point sensor_proc_base,
                           std::chrono::steady_clock::time_point t0,
                           bool is_sensor_cycle) noexcept {
    pt.sensor_proc_us =
        std::chrono::duration<double, std::micro>(tail.compute_done - sensor_proc_base).count() -
        tail.ft_infer_us;
    pt.ft_infer_us = tail.ft_infer_us;
    pt.total_us = std::chrono::duration<double, std::micro>(tail.callback_done - t0).count();
    pt.is_sensor_cycle = is_sensor_cycle;
    timing_profiler_.Update(pt);
  }

  // Consume any pending calibration request. Called on CommLoop thread.
  void DispatchCalibrationRequest() noexcept {
    if (!calib_request_pending_.load(std::memory_order_acquire))
      return;
    const PendingCalibration req = pending_calib_;
    calib_request_pending_.store(false, std::memory_order_release);

    switch (req.sensor_type) {
      case calibration::kSensorBarometer: {
        if (!ft_inferencer_) {
          RCLCPP_WARN(::udp_hand_driver::logging::ControllerLogger(),
                      "Calibration: barometer request ignored (no inferencer)");
          return;
        }
        // Calibration START/ABORT are externally triggered, but the dispatch
        // runs on the CommLoop hot path. Throttle as a defensive RT-safety
        // net even though duplicate requests are already de-duped upstream.
        static rclcpp::Clock calib_clock(RCL_STEADY_TIME);
        if (req.action == calibration::kActionStart) {
          RCLCPP_INFO_THROTTLE(::udp_hand_driver::logging::ControllerLogger(), calib_clock,
                               ::udp_hand_driver::logging::kThrottleSlowMs,
                               "Calibration: barometer START (sample_count=%u)",
                               static_cast<unsigned>(req.sample_count));
          ft_inferencer_->ResetCalibration(req.sample_count);
        } else if (req.action == calibration::kActionAbort) {
          RCLCPP_INFO_THROTTLE(::udp_hand_driver::logging::ControllerLogger(), calib_clock,
                               ::udp_hand_driver::logging::kThrottleSlowMs,
                               "Calibration: barometer ABORT (noop)");
          // Currently barometer cal has no externally-visible abort state;
          // leaving the in-progress buffers alone is acceptable.
        }
        break;
      }
      // Future sensors: add cases here.
      default:
        RCLCPP_WARN(::udp_hand_driver::logging::ControllerLogger(),
                    "Calibration: unknown sensor_type=%u", static_cast<unsigned>(req.sensor_type));
        break;
    }
  }

  // Run FT calibration or inference. Returns inference elapsed time in us.
  double RunFTInference(
      const std::array<int32_t, udp_hand_driver::kMaxHandSensors>& sensor_data) noexcept {
    // Consume any pending calibration request before processing this cycle.
    DispatchCalibrationRequest();

    if (!ft_enabled_ || !ft_inferencer_)
      return 0.0;

    if (!ft_inferencer_->is_calibrated()) {
      static_cast<void>(ft_inferencer_->FeedCalibration(sensor_data, num_fingertips_));
      return 0.0;
    }

    const auto ft_t0 = std::chrono::steady_clock::now();
    auto ft_result = ft_inferencer_->Infer(sensor_data, num_fingertips_);
    const auto ft_t1 = std::chrono::steady_clock::now();
    ft_seqlock_.Store(ft_result);
    return std::chrono::duration<double, std::micro>(ft_t1 - ft_t0).count();
  }

  rtc::ThreadConfig thread_cfg_;
  // CommLoop cadence (Hz). Drives the self-clocked read/publish period; write
  // is command-gated (see RunCommCycle). Validated (> 0) in Start().
  double loop_rate_hz_;
  int sensor_decimation_;
  // Whole-cycle UDP decimation (load-reduction test knob). N → 1 real comm
  // cycle per N CommLoop cycles; the other N-1 do no send/recv/publish. 1 =
  // communicate every cycle (default; bit-identical to pre-feature behavior).
  int comm_decimation_;
  int num_fingertips_;
  bool use_fake_hand_;
  std::vector<std::string> fingertip_names_;
  HandCommunicationMode communication_mode_;
  bool sensor_init_ok_{false};

  // F/T inferencer config (before transport_ for initializer list order)
  FingertipFTInferencer::Config ft_config_;

  // Firmware sensor protocol (version seam). Defaulted to 1a in Start() if the
  // node did not inject one. Owned here; the CommLoop dispatches sensor decode
  // and queries capabilities (motor-space read, post-process) through it.
  std::unique_ptr<SensorProtocol> sensor_protocol_;

  // Sub-systems
  UdpHandTransport transport_;
  UdpHandSensorProcessor sensor_processor_;
  UdpHandTimingProfiler timing_profiler_;
  std::unique_ptr<FingertipFTInferencer> ft_inferencer_;
  rtc::SeqLock<udp_hand_driver::FingertipFTState> ft_seqlock_{};
  bool ft_enabled_{false};

  // E-Stop flag (set by RtControllerNode, null if not used)
  std::atomic<bool>* estop_flag_{nullptr};

  // Command handoff (RT-safe, no wake primitive): the producer
  // (SendCommandAndRequestStates — ROS command sub / CM ControlLoop) stores the
  // latest cmd into the SeqLock and sets event_pending_ (release). The
  // self-clocked CommLoop latches it at the top of its next cycle (acquire).
  static_assert(std::is_trivially_copyable_v<std::array<float, kNumHandMotors>>,
                "SeqLock payload must be trivially copyable");
  rtc::SeqLock<std::array<float, kNumHandMotors>> staged_cmd_seqlock_{};
  std::atomic<bool> event_pending_{false};

  // Count of cycles skipped by comm_decimation_ (load-reduction telemetry).
  std::atomic<uint64_t> comm_skip_count_{0};

  // Shared state
  std::atomic<bool> running_{false};
  bool state_read_once_{false};  // True after first successful state read
  StateCallback callback_;
  rtc::HandUdpTimingBuffer* timing_producer_{nullptr};
  rtc::SeqLock<UdpHandState> state_seqlock_{};
  std::atomic<std::size_t> cycle_count_{0};

  // Non-RT: captured in Start() to report the average cycle rate in Stop().
  std::chrono::steady_clock::time_point start_time_{};

  // Link health. consecutive_recv_failures_ is the aggregate "no channel
  // answered this cycle" streak — kept for onset/steady logging + the forensic
  // JSON field. channel_consec_fail_ is the PER-CHANNEL streak that drives the
  // strict link-down decision (#1): one healthy channel no longer masks a dead
  // one. Writer = CommLoop; readers = detector / publish / stats threads
  // (relaxed atomics, tear-free scalar reads).
  std::atomic<uint64_t> consecutive_recv_failures_{0};
  std::array<std::atomic<uint32_t>, kNumRequestKinds> channel_consec_fail_{};

  // Cycle outcome ring (writer: CommLoop; readers: failure detector, stats
  // save). outcome_ring_working_ accumulates across cycles (CommLoop-thread
  // only); each cycle ends with one Store() into the SeqLock (~0.5 KB memcpy —
  // negligible at loop_rate_hz).
  static_assert(std::is_trivially_copyable_v<CycleOutcomeRing>,
                "SeqLock payload must be trivially copyable");
  rtc::SeqLock<CycleOutcomeRing> outcome_ring_seqlock_{};
  CycleOutcomeRing outcome_ring_working_{};

  // ── RunCommCycle hot-path state (CommLoop thread only — no sync) ─────────
  // Per-loop-lifetime constants hoisted in Start() from sensor_protocol_ /
  // communication_mode_.
  bool is_bulk_{false};
  packets::JointMode joint_io_mode_{packets::JointMode::kJoint};

  // Latched write command + its pending flag (per-command gate). pending_cmd_
  // holds the last staged command; has_pending_write_ is set when latched and
  // cleared after one write attempt (see the write gate in RunCommCycle).
  std::array<float, kNumHandMotors> pending_cmd_{};
  bool has_pending_write_{false};

  // Scratch buffers reused every cycle (avoid per-tick re-zeroing of large
  // arrays; overwritten by the transport before use).
  std::array<uint8_t, packets::kMotorPacketSize> send_buf_{};
  std::array<uint8_t, packets::kMotorPacketSize> echo_buf_{};
  std::array<float, packets::kMotorDataCount> motor_pos_buf_{};
  std::array<float, packets::kMotorDataCount> motor_vel_buf_{};
  std::array<float, packets::kMotorDataCount> motor_cur_buf_{};
  std::array<int32_t, udp_hand_driver::kSensorValuesPerFingertip> sensor_raw_buf_{};
  // Raw bulk-sensor receive scratch (sized for the largest response, 259B).
  std::array<uint8_t, packets::kMaxPacketSize> sensor_recv_buf_{};

  // Sensor caches (persist across decimated sensor cycles). 1a int32 pipeline
  // vs 1b float force; the unused one stays zero.
  std::array<int32_t, udp_hand_driver::kMaxHandSensors> cached_sensor_data_{};
  std::array<float, udp_hand_driver::kMaxSensorForceValues> cached_sensor_force_{};

  // Decimation counters (CommLoop thread only).
  int sensor_cycle_counter_{0};
  int comm_cycle_counter_{0};

  // Monotonic sensor-freshness sequence (CommLoop thread only). Bumped once per
  // cycle that produced a fresh validated sensor read; published into
  // UdpHandState::sensor_seq so the ~50 Hz detector can distinguish a genuine
  // firmware stall from a decimated byte-frozen republish it polled twice.
  uint32_t sensor_seq_{0};

  // Pending calibration request (ROS thread -> CommLoop handoff).
  // Written by RequestCalibration() (arbitrary thread), consumed by
  // DispatchCalibrationRequest() (CommLoop thread). Last-wins semantics.
  struct PendingCalibration {
    uint8_t sensor_type{0};
    uint8_t action{0};
    uint16_t sample_count{0};
  };

  PendingCalibration pending_calib_{};
  std::atomic<bool> calib_request_pending_{false};

  // Self-clocked comm thread. Declared last so ~PeriodicRtThread joins the loop
  // before any member it touches (transport_, sensor_processor_, …) is
  // destroyed. Stop()/~UdpHandController also Join() it explicitly.
  CommLoop comm_loop_{this};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_CONTROLLER_HPP_
