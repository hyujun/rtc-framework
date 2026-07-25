// Shared fixtures for the full-pipeline RtControllerNode tests
// (test_cm_config_pipeline.cpp / test_rt_loop_pipeline.cpp).
//
// Unlike rt_cm_test_access.hpp (private-member injection, Tier 1), these
// fixtures drive the REAL bring-up: a registry-registered controller whose
// YAML (config/test_fixtures/controllers/rtc_cm_cfg_test.yaml, selected via
// config_variant=test_fixtures) declares an "arm" device group, plus a
// registry-registered DeviceBackend stub. Each test .cpp registers them with
// RTC_REGISTER_CONTROLLER / RTC_REGISTER_DEVICE_BACKEND in its own TU (the
// macros open an anonymous namespace, so they cannot live in this header) —
// one registration per test binary; separate gtest executables never collide.
#pragma once

#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_backend_registry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <thread>

namespace rtc {

// ── Registry-loadable controller with an "arm" device group ──────────────────
//
// LoadConfig captures the (override-merged) YAML values so tests can assert
// the ApplyControllerParamOverrides result end to end; the base LoadConfig
// call parses the topics: section into topic_config_ ("arm" group → CM builds
// active_groups_ / group_slot_map_ from it).
//
// Compute returns a fixed, recognisable command vector so the RT-loop test
// can assert the tick pipeline (state read → Compute → WriteCommand) verbatim.
// The optional compute_sleep_us knob (static — the CM owns the instance, tests
// have no handle before on_configure) forces deterministic deadline overruns.
class PipelineTestController : public RTControllerInterface {
 public:
  static constexpr double kCmd0 = 1.5;
  static constexpr double kCmd1 = -2.5;

  // Captured YAML values (post-override). Static: the registry factory creates
  // the instance CM-side, so tests read these after on_configure.
  static inline double captured_kp{-1.0};
  static inline std::string captured_label{};
  static inline bool captured_enabled{false};
  static inline int64_t captured_count{-1};
  static inline double captured_deep_b{-1.0};
  static inline std::vector<double> captured_vals{};
  static inline std::vector<std::string> captured_tags{};
  static inline std::vector<int64_t> captured_ids{};
  static inline std::vector<bool> captured_flags{};

  // GetDefaultDt() as observed INSIDE LoadConfig, i.e. at PreConfigure time.
  // Controllers validate rate-dependent YAML there (filter cutoffs against
  // Nyquist), so a bring-up that hands the control rate over only after
  // PreConfigure makes those checks run against the fallback dt and reject
  // configs that are legal at the configured rate. -1 = LoadConfig never ran.
  static inline double captured_dt_at_load_config{-1.0};

  static inline std::atomic<int> compute_sleep_us{0};

  // Channel count Compute() reports. Tests set an out-of-contract value
  // (negative, or past kMaxDeviceChannels) to exercise the RT loop's bound
  // on a faulty controller — see issue #196 §4.
  static inline std::atomic<int> output_num_channels{2};

  // ── ControllerOutput fault injection (issue #196 Phase 4) ─────────────────
  //
  // The validator's whole purpose is to stop output that no in-tree
  // controller produces, so nothing in the normal fixture path exercises it.
  // These knobs make Compute() emit each rejectable shape on demand.
  enum class OutputFault {
    kNone,
    kInvalidFlag,       ///< valid = false
    kDeviceCount,       ///< num_devices disagrees with the state
    kChannelCount,      ///< num_channels past what the device reported
    kNonFiniteCommand,  ///< NaN in commands[]
  };
  static inline std::atomic<OutputFault> output_fault{OutputFault::kNone};

  // How many ticks the fault lasts. Negative → until reset. A finite budget
  // is what makes the E-STOP threshold testable: the loop free-runs, so
  // "exactly N-1 bad ticks" has to be enforced by the producer, not by
  // trying to stop the consumer at the right moment.
  static inline std::atomic<int> output_fault_ticks{-1};

  // Ticks on which a fault was actually injected — the ground truth the
  // node's reject counter is compared against.
  static inline std::atomic<int> injected_fault_ticks{0};

  // Instances the registry factory has built (issue #196 Phase 5). The
  // duplicate-config_key scan runs before Pass 1, so a refused bring-up must
  // leave this at zero — that is what separates "rejected before creating
  // controllers and LifecycleNodes" from "rejected after".
  static inline std::atomic<int> instances_created{0};

  // Not noexcept: the base RTControllerInterface ctor is not, and marking this
  // one noexcept would turn a throwing base subobject into std::terminate
  // inside the registry factory instead of a catchable bring-up failure.
  PipelineTestController() { instances_created.fetch_add(1, std::memory_order_relaxed); }

  static void ResetCaptured() {
    captured_kp = -1.0;
    captured_label.clear();
    captured_enabled = false;
    captured_count = -1;
    captured_deep_b = -1.0;
    captured_dt_at_load_config = -1.0;
    captured_vals.clear();
    captured_tags.clear();
    captured_ids.clear();
    captured_flags.clear();
    compute_sleep_us.store(0, std::memory_order_relaxed);
    output_num_channels.store(2, std::memory_order_relaxed);
    output_fault.store(OutputFault::kNone, std::memory_order_relaxed);
    output_fault_ticks.store(-1, std::memory_order_relaxed);
    injected_fault_ticks.store(0, std::memory_order_relaxed);
    instances_created.store(0, std::memory_order_relaxed);
  }

  void LoadConfig(const YAML::Node& cfg) override {
    RTControllerInterface::LoadConfig(cfg);  // topics: → topic_config_
    // FIRST LoadConfig only — that is the PreConfigure one, and the only call
    // whose rate visibility is in question. The Pass-3 on_configure LoadConfig
    // always runs after SetControlRate, so recording every call would overwrite
    // the observation with a value that cannot distinguish the two orderings.
    if (captured_dt_at_load_config < 0.0) {
      captured_dt_at_load_config = GetDefaultDt();
    }
    if (!cfg) {
      return;
    }
    if (const auto gains = cfg["gains"]) {
      captured_kp = gains["kp"] ? gains["kp"].as<double>() : -1.0;
      captured_label = gains["label"] ? gains["label"].as<std::string>() : "";
      captured_enabled = gains["enabled"] && gains["enabled"].as<bool>();
      captured_count = gains["count"] ? gains["count"].as<int64_t>() : -1;
      if (gains["vals"]) {
        captured_vals = gains["vals"].as<std::vector<double>>();
      }
      if (gains["tags"]) {
        captured_tags = gains["tags"].as<std::vector<std::string>>();
      }
      if (gains["ids"]) {
        captured_ids = gains["ids"].as<std::vector<int64_t>>();
      }
      if (gains["flags"]) {
        captured_flags = gains["flags"].as<std::vector<bool>>();
      }
    }
    if (cfg["deep"] && cfg["deep"]["a"] && cfg["deep"]["a"]["b"]) {
      captured_deep_b = cfg["deep"]["a"]["b"].as<double>();
    }
  }

  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    const int sleep_us = compute_sleep_us.load(std::memory_order_relaxed);
    if (sleep_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
    ControllerOutput out{};
    out.num_devices = 1;
    out.devices[0].num_channels = output_num_channels.load(std::memory_order_relaxed);
    out.devices[0].commands[0] = kCmd0;
    out.devices[0].commands[1] = kCmd1;
    InjectOutputFault(out);
    return out;
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return "rtc_cm_cfg_test"; }

 private:
  // Applies the configured fault to `out`, honouring the tick budget. Called
  // from Compute() on the RT thread — relaxed atomics only, no allocation.
  static void InjectOutputFault(ControllerOutput& out) noexcept {
    const OutputFault fault = output_fault.load(std::memory_order_relaxed);
    if (fault == OutputFault::kNone) {
      return;
    }
    const int budget = output_fault_ticks.load(std::memory_order_relaxed);
    if (budget == 0) {
      return;  // budget spent — emit a clean output so the counter can reset
    }
    if (budget > 0) {
      output_fault_ticks.fetch_sub(1, std::memory_order_relaxed);
    }
    injected_fault_ticks.fetch_add(1, std::memory_order_relaxed);

    switch (fault) {
      case OutputFault::kInvalidFlag:
        out.valid = false;
        break;
      case OutputFault::kDeviceCount:
        // The fixture group declares exactly one device, so 2 is a mismatch.
        out.num_devices = 2;
        break;
      case OutputFault::kChannelCount:
        // Inside kMaxDeviceChannels but past the 2 channels the stub backend
        // reports — the case the array-capacity clamp alone cannot catch.
        out.devices[0].num_channels = 3;
        break;
      case OutputFault::kNonFiniteCommand:
        out.devices[0].commands[1] = std::numeric_limits<double>::quiet_NaN();
        break;
      case OutputFault::kNone:
        break;
    }
  }
};

// ── Controllers whose Name() differs from their registration config_key ──────
//
// PipelineTestController::Name() returns "rtc_cm_cfg_test", the same string it
// is registered under everywhere in this directory. That collapses the two
// halves of the controller_name_to_idx_ namespace into one key, so no test
// built on it alone can tell a guard that checks {Name(), config_key} from one
// that checks {Name()} only (issue #205 B-1/B-2). These two subclasses keep the
// strings distinct so the config_key half is reachable on its own.
//
// Subclassing rather than duplicating: only Name() differs, and the config
// capture / Compute / fault-injection behaviour must stay identical or a
// failure here would not mean what it says.
class AliasNameTestControllerA : public PipelineTestController {
 public:
  static constexpr const char* kName = "rtc_cm_alias_name_a";

  std::string_view Name() const noexcept override { return kName; }
};

class AliasNameTestControllerB : public PipelineTestController {
 public:
  static constexpr const char* kName = "rtc_cm_alias_name_b";

  std::string_view Name() const noexcept override { return kName; }
};

// ── Registry-loadable DeviceBackend stub with all three state lanes ──────────
//
// ReadState/ReadMotorState/ReadSensorState fill fixed, recognisable values so
// the RT-loop test can assert the per-capability copy blocks in ControlLoop.
// WriteCommand records the last slot (write → count.release; test reads
// count.acquire → values — commands are constant per tick, so the benign
// overwrite race cannot produce torn observations).
class PipelineStubBackend : public DeviceBackend {
 public:
  static constexpr double kPos0 = 0.11;
  static constexpr double kPos1 = 0.22;
  static constexpr double kMotorPos0 = 7.5;
  static constexpr int32_t kSensor0 = 42;
  static constexpr float kInfer0 = 3.5F;

  // Configure() calls the CM has made. CreateDeviceBackends() runs only after
  // DeclareAndLoadParameters() returns true, so a bring-up refused at the D1
  // checkpoint must leave this at zero — that is what proves the refusal
  // happened before any device wiring, even when controllers were already
  // instantiated (issue #196 Phase 5, Tier 2).
  //
  // Reset by ResetCaptured() below. It used to be reset only by the one test
  // that read it, which made `EXPECT_EQ(0, configure_calls)` — the natural way
  // to assert "refused before device wiring" — silently depend on whether an
  // earlier test in the same binary had already incremented it (issue #205
  // B-3). Counters that only read as zero are exactly the ones that must not
  // inherit state.
  static inline std::atomic<int> configure_calls{0};

  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/, const DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr /*state_cb_group*/) override {
    configure_calls.fetch_add(1, std::memory_order_relaxed);
    group_name_ = config.group_name;
  }

  void Activate() override {}

  void Deactivate() override {}

  // Channel count ReadState() reports. Tests set an out-of-contract value to
  // exercise the RT loop's bound on a faulty backend — see issue #196 §4.
  static inline std::atomic<int> state_num_channels{2};

  // Makes ReadState report a non-finite measured position. The device read
  // path bounds counts but copies position doubles verbatim, so this is the
  // one input that can make the CM's hold command itself non-finite — the
  // condition the validator exists to keep off the wire.
  static inline std::atomic<bool> state_position_nan{false};

  static void ResetStateChannels() {
    state_num_channels.store(2, std::memory_order_relaxed);
    state_position_nan.store(false, std::memory_order_relaxed);
  }

  // Full per-test reset for this backend — the counterpart to
  // PipelineTestController::ResetCaptured(). Call BOTH in SetUp(): the
  // controller helper cannot reach this class's statics.
  static void ResetCaptured() {
    ResetStateChannels();
    configure_calls.store(0, std::memory_order_relaxed);
  }

  bool ReadState(DeviceStateCache& cache) noexcept override {
    cache.num_channels = state_num_channels.load(std::memory_order_relaxed);
    cache.positions[0] = state_position_nan.load(std::memory_order_relaxed)
                             ? std::numeric_limits<double>::quiet_NaN()
                             : kPos0;
    cache.positions[1] = kPos1;
    cache.velocities[0] = 1.0;
    cache.velocities[1] = 2.0;
    cache.efforts[0] = 0.5;
    cache.efforts[1] = 0.6;
    cache.valid = true;
    return true;
  }

  bool HasMotorState() const noexcept override { return true; }

  bool HasSensorState() const noexcept override { return true; }

  void ReadMotorState(DeviceStateCache& cache) noexcept override {
    cache.num_motor_channels = 1;
    cache.motor_positions[0] = kMotorPos0;
    cache.motor_velocities[0] = 0.1;
    cache.motor_efforts[0] = 0.2;
  }

  void ReadSensorState(DeviceStateCache& cache) noexcept override {
    cache.num_sensor_channels = 2;
    cache.sensor_data[0] = kSensor0;
    cache.sensor_data[1] = kSensor0 + 1;
    cache.sensor_data_raw[0] = kSensor0 + 2;
    cache.sensor_data_raw[1] = kSensor0 + 3;
    cache.num_inference_groups = 1;
    cache.inference_data[0] = kInfer0;
    cache.inference_enable[0] = true;
  }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType /*command_type*/) noexcept override {
    last_num_channels_ = slot.num_channels;
    last_actual_num_channels_ = slot.actual_num_channels;
    last_commands_[0] = slot.commands[0];
    last_commands_[1] = slot.commands[1];
    last_actual_positions_[0] = slot.actual_positions[0];
    last_actual_positions_[1] = slot.actual_positions[1];
    write_count_.fetch_add(1, std::memory_order_release);
  }

  // Honours the LastStateStamp contract: default-constructed until the first
  // state arrives. It used to return now() unconditionally, which claimed the
  // device had reported before it ever did — harmless while CM kept its own
  // `received` flag, but the startup gate now reads this and a fixture that
  // lies here would pass the very test it exists to fail (issue #198 §1).
  std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(last_state_ns_.load(std::memory_order_acquire)));
  }

  void FireStateReady() {
    SetStateStamp(std::chrono::steady_clock::now());
    NotifyStateReady();
  }

  /// Backdate (or clear) the liveness stamp without notifying — lets a test
  /// stage a device that reported once and then went quiet.
  void SetStateStamp(std::chrono::steady_clock::time_point stamp) {
    last_state_ns_.store(stamp.time_since_epoch().count(), std::memory_order_release);
  }

  // Safe output (issue #198 Phase 4). Records the call so a test can tell
  // "CM handed the decision to the backend" from "CM wrote a zero-length slot
  // and the backend silently dropped it" — at the wire those looked the same,
  // which is what made the old behaviour so easy to mistake for fail-closed.
  void WriteSafeCommand() noexcept override {
    safe_write_count_.fetch_add(1, std::memory_order_release);
  }

  int SafeWriteCount() const { return safe_write_count_.load(std::memory_order_acquire); }

  int WriteCount() const { return write_count_.load(std::memory_order_acquire); }

  int LastNumChannels() const { return last_num_channels_; }

  int LastActualNumChannels() const { return last_actual_num_channels_; }

  const std::array<double, 2>& LastCommands() const { return last_commands_; }

  const std::array<double, 2>& LastActualPositions() const { return last_actual_positions_; }

  const std::string& GroupName() const { return group_name_; }

 private:
  std::string group_name_;
  std::atomic<int> write_count_{0};
  std::atomic<int> safe_write_count_{0};
  std::atomic<std::int64_t> last_state_ns_{0};
  int last_num_channels_{0};
  int last_actual_num_channels_{0};
  std::array<double, 2> last_commands_{};
  std::array<double, 2> last_actual_positions_{};
};

}  // namespace rtc
