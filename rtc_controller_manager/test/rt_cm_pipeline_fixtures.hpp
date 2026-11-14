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

  // `rt_layout_profile` as this controller's OWN node saw it at on_configure —
  // the node a controller reads it from. "<absent>" = not declared there.
  static inline std::string captured_layout_profile{};

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

  // Per-device command_type override for device 0 (issue #342). Negative
  // leaves it unset, so the device inherits the controller-global type;
  // otherwise the value is a CommandType.
  //
  // Deliberately NOT an OutputFault. One of the two cases it has to express —
  // kPdFeedforward onto a position-only backend — is not a fault at all but
  // the documented graceful fallback the shipped WBC→hand lane depends on, and
  // folding it into the fault enum would put "must be rejected" and "must be
  // forwarded untouched" behind one knob whose name asserts the first.
  //
  // GetCommandType() stays kPosition (the base default) throughout, which is
  // the whole point: this controller passes ValidateCommandTypePairing, so the
  // configure gate cannot see the mode it actually emits — and neither can any
  // other TU that builds a node here.
  static inline std::atomic<int> device0_command_type{-1};

  // How many ticks the override lasts, on the same terms as
  // output_fault_ticks: negative → until reset. A finite budget is what makes
  // "exactly N-1 rejected ticks" assertable against a free-running loop.
  static inline std::atomic<int> device0_command_type_ticks{-1};

  // Ticks on which the override was actually emitted — ground truth for the
  // node's own tally.
  static inline std::atomic<int> injected_command_type_ticks{0};

  // Instances the registry factory has built (issue #196 Phase 5). The
  // duplicate-config_key scan runs before Pass 1, so a refused bring-up must
  // leave this at zero — that is what separates "rejected before creating
  // controllers and LifecycleNodes" from "rejected after".
  static inline std::atomic<int> instances_created{0};

  // on_cleanup calls the CM made on instances of this class (#935). A refused
  // configure drops the controllers it built, and one that got as far as
  // on_configure holds resources only on_cleanup releases — so a refusal must
  // leave this equal to instances_created.
  static inline std::atomic<int> cleanup_count{0};

  // Makes this controller's on_configure report FAILURE (#935) — the Pass-3
  // refusal, after its siblings may already have configured.
  static inline std::atomic<bool> fail_on_configure{false};

  // DeviceState::hole_mask as seen by Compute on the last tick (issue #284).
  // The read end of PipelineStubBackend::state_hole_mask — together they span
  // the whole ingress→controller path, which is the only span that can catch a
  // dropped field in the two PODs' field-by-field copy.
  static inline std::atomic<uint64_t> observed_hole_mask{0};
  // Same seam, the other two lanes (#446). Separate observers rather than one,
  // because the defect they guard is a field DROPPED from the copy — a single
  // combined value would let one survivor mask two losses.
  static inline std::atomic<uint64_t> observed_velocity_hole_mask{0};
  static inline std::atomic<uint64_t> observed_effort_hole_mask{0};
  // The fingertip lane's per-group receipt time and sample counter as seen by
  // Compute — the read end of PipelineStubBackend::sensor_recv_steady_ns /
  // sensor_sequence. Group 1
  // rather than group 0 so a copy that fills only the first element, or one
  // that copies a scalar instead of the array, is visible.
  static inline std::atomic<int64_t> observed_inference_recv_ns_g1{0};
  static inline std::atomic<uint64_t> observed_inference_sequence_g1{0};
  // The floating-base and IMU lanes as seen by Compute — the read end of
  // PipelineStubBackend::kBasePose / kBaseTwist / kImuSite1 and the base_* /
  // imu_* knobs. EVERY field, not a sample of them: this seam fails one field
  // at a time, and a dropped field reads as 0 / false here, so only an
  // assertion on that field at this end can see it. IMU site 1's values
  // rather than site 0's for the same reason the fingertip observers read
  // group 1.
  static inline std::array<std::atomic<double>, kBasePoseDim> observed_base_pose{};
  static inline std::array<std::atomic<double>, kBaseTwistDim> observed_base_twist{};
  static inline std::array<std::atomic<double>, kImuChannelsPerSite> observed_imu_site1{};
  static inline std::atomic<int> observed_num_imu_sites{0};
  static inline std::atomic<int64_t> observed_base_recv_ns{0};
  static inline std::atomic<uint64_t> observed_base_sequence{0};
  static inline std::atomic<bool> observed_base_valid{false};
  // The IMU lane as seen by Compute — the read end of the stub's imu_* knobs,
  // every field of BOTH sites: the stub gives each site its own stamp, counter
  // and flag, so a copy that crosses sites or stops at one is visible here.
  static inline std::array<std::atomic<double>, kImuChannelsPerSite> observed_imu_site0{};
  static inline std::array<std::atomic<int64_t>, kMaxImuSites> observed_imu_recv_ns{};
  static inline std::array<std::atomic<uint64_t>, kMaxImuSites> observed_imu_sequence{};
  static inline std::array<std::atomic<bool>, kMaxImuSites> observed_imu_valid{};
  // The joint lane's receipt stamp and counter as seen by Compute.
  static inline std::atomic<int64_t> observed_joint_recv_ns{0};
  static inline std::atomic<uint64_t> observed_joint_sequence{0};

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
    captured_layout_profile.clear();
    captured_vals.clear();
    captured_tags.clear();
    captured_ids.clear();
    captured_flags.clear();
    compute_sleep_us.store(0, std::memory_order_relaxed);
    output_num_channels.store(2, std::memory_order_relaxed);
    output_fault.store(OutputFault::kNone, std::memory_order_relaxed);
    output_fault_ticks.store(-1, std::memory_order_relaxed);
    injected_fault_ticks.store(0, std::memory_order_relaxed);
    device0_command_type.store(-1, std::memory_order_relaxed);
    device0_command_type_ticks.store(-1, std::memory_order_relaxed);
    injected_command_type_ticks.store(0, std::memory_order_relaxed);
    instances_created.store(0, std::memory_order_relaxed);
    cleanup_count.store(0, std::memory_order_relaxed);
    fail_on_configure.store(false, std::memory_order_relaxed);
    observed_hole_mask.store(0, std::memory_order_relaxed);
    observed_velocity_hole_mask.store(0, std::memory_order_relaxed);
    observed_effort_hole_mask.store(0, std::memory_order_relaxed);
    observed_inference_recv_ns_g1.store(0, std::memory_order_relaxed);
    observed_inference_sequence_g1.store(0, std::memory_order_relaxed);
    for (auto& v : observed_base_pose)
      v.store(0.0, std::memory_order_relaxed);
    for (auto& v : observed_base_twist)
      v.store(0.0, std::memory_order_relaxed);
    for (auto& v : observed_imu_site1)
      v.store(0.0, std::memory_order_relaxed);
    observed_num_imu_sites.store(0, std::memory_order_relaxed);
    observed_base_recv_ns.store(0, std::memory_order_relaxed);
    observed_base_sequence.store(0, std::memory_order_relaxed);
    observed_base_valid.store(false, std::memory_order_relaxed);
    for (auto& v : observed_imu_site0)
      v.store(0.0, std::memory_order_relaxed);
    for (auto& v : observed_imu_recv_ns)
      v.store(0, std::memory_order_relaxed);
    for (auto& v : observed_imu_sequence)
      v.store(0, std::memory_order_relaxed);
    for (auto& v : observed_imu_valid)
      v.store(false, std::memory_order_relaxed);
    observed_joint_recv_ns.store(0, std::memory_order_relaxed);
    observed_joint_sequence.store(0, std::memory_order_relaxed);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override {
    const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
    const auto lc = get_lifecycle_node();
    captured_layout_profile = (lc && lc->has_parameter("rt_layout_profile"))
                                  ? lc->get_parameter("rt_layout_profile").as_string()
                                  : std::string("<absent>");
    if (fail_on_configure.load(std::memory_order_relaxed)) {
      return CallbackReturn::FAILURE;
    }
    return ret;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override {
    cleanup_count.fetch_add(1, std::memory_order_relaxed);
    return RTControllerInterface::on_cleanup(prev);
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

  ControllerOutput Compute(const ControllerState& state) noexcept override {
    // What the RT loop actually handed this controller for device 0 (#284).
    // Recorded from INSIDE Compute rather than read off the backend, because
    // the field's whole risk is the DeviceStateCache→DeviceState copy that
    // sits between the two — an assertion on the cache would pass with that
    // copy deleted.
    observed_hole_mask.store(state.devices[0].hole_mask, std::memory_order_relaxed);
    observed_velocity_hole_mask.store(state.devices[0].velocity_hole_mask,
                                      std::memory_order_relaxed);
    observed_effort_hole_mask.store(state.devices[0].effort_hole_mask, std::memory_order_relaxed);
    observed_inference_recv_ns_g1.store(state.devices[0].inference_recv_steady_ns[1],
                                        std::memory_order_relaxed);
    observed_inference_sequence_g1.store(state.devices[0].inference_sequence[1],
                                         std::memory_order_relaxed);
    const auto& dev0 = state.devices[0];
    for (std::size_t i = 0; i < observed_base_pose.size(); ++i) {
      observed_base_pose[i].store(dev0.base_pose[i], std::memory_order_relaxed);
    }
    for (std::size_t i = 0; i < observed_base_twist.size(); ++i) {
      observed_base_twist[i].store(dev0.base_twist[i], std::memory_order_relaxed);
    }
    for (std::size_t i = 0; i < observed_imu_site1.size(); ++i) {
      observed_imu_site1[i].store(dev0.imu[1][i], std::memory_order_relaxed);
    }
    observed_num_imu_sites.store(dev0.num_imu_sites, std::memory_order_relaxed);
    observed_base_recv_ns.store(dev0.base_recv_steady_ns, std::memory_order_relaxed);
    observed_base_sequence.store(dev0.base_sequence, std::memory_order_relaxed);
    observed_base_valid.store(dev0.base_valid, std::memory_order_relaxed);
    for (std::size_t i = 0; i < observed_imu_site0.size(); ++i) {
      observed_imu_site0[i].store(dev0.imu[0][i], std::memory_order_relaxed);
    }
    for (std::size_t s = 0; s < observed_imu_recv_ns.size(); ++s) {
      observed_imu_recv_ns[s].store(dev0.imu_recv_steady_ns[s], std::memory_order_relaxed);
      observed_imu_sequence[s].store(dev0.imu_sequence[s], std::memory_order_relaxed);
      observed_imu_valid[s].store(dev0.imu_valid[s], std::memory_order_relaxed);
    }
    observed_joint_recv_ns.store(dev0.joint_recv_steady_ns, std::memory_order_relaxed);
    observed_joint_sequence.store(dev0.joint_sequence, std::memory_order_relaxed);
    const int sleep_us = compute_sleep_us.load(std::memory_order_relaxed);
    if (sleep_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
    ControllerOutput out{};
    out.num_devices = 1;
    out.devices[0].num_channels = output_num_channels.load(std::memory_order_relaxed);
    out.devices[0].commands[0] = kCmd0;
    out.devices[0].commands[1] = kCmd1;
    InjectDevice0CommandType(out);
    InjectOutputFault(out);
    return out;
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return "rtc_cm_cfg_test"; }

 private:
  // Applies the per-device command_type override to `out`, honouring its own
  // tick budget (issue #342). Separate from InjectOutputFault so the budgets
  // do not interfere: a test may want a mode override on a tick that is
  // otherwise in contract, which is the only shape that reaches the RT-path
  // command-type guard at all.
  static void InjectDevice0CommandType(ControllerOutput& out) noexcept {
    const int ct = device0_command_type.load(std::memory_order_relaxed);
    if (ct < 0) {
      return;
    }
    const int budget = device0_command_type_ticks.load(std::memory_order_relaxed);
    if (budget == 0) {
      return;  // budget spent — inherit the global type again
    }
    if (budget > 0) {
      device0_command_type_ticks.fetch_sub(1, std::memory_order_relaxed);
    }
    injected_command_type_ticks.fetch_add(1, std::memory_order_relaxed);
    out.devices[0].command_type = static_cast<CommandType>(ct);
  }

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

  // Per-slot freshness ReadState() reports (issue #284). Defaults to 0 — "no
  // holes claimed" — so every pre-existing test in every TU that shares this
  // fixture keeps the state it already asserted against.
  static inline std::atomic<uint64_t> state_hole_mask{0};
  static inline std::atomic<uint64_t> state_velocity_hole_mask{0};
  static inline std::atomic<uint64_t> state_effort_hole_mask{0};

  // Fingertip-lane receipt time and sample counter this backend reports for
  // sensor GROUP 1. Defaults are 0 — "never
  // received" — so tests that predate these fields keep the state they
  // already asserted against.
  static inline std::atomic<int64_t> sensor_recv_steady_ns_g1{0};
  static inline std::atomic<uint64_t> sensor_sequence_g1{0};

  // Joint-lane receipt stamp and counter ReadState() reports. Defaults are 0 —
  // "never received" — so tests that predate these fields keep their state.
  static inline std::atomic<int64_t> joint_recv_steady_ns{0};
  static inline std::atomic<uint64_t> joint_sequence{0};

  // Floating-base lane. Opt-in per test: HasFloatingBase() is
  // what CM derives kFloatingBase from at configure, and the default stays
  // false so every TU sharing this fixture keeps the capability set it already
  // asserted against (and so the gate itself is testable — see
  // FloatingBaseLaneIsWithheldWithoutTheCapability).
  //
  // The values are DISTINCT per slot and per lane on purpose: a copy that
  // assigns the pose array to the twist, or site 0 to site 1, must not pass.
  //
  // ReadBaseState ALSO writes the IMU arrays, the way a producer from before
  // the base and IMU lanes were split filled them from its base read: the base
  // copy must not carry them, and only a stub that writes them can show that
  // (see ImuLaneIsIndependentOfTheBaseLane).
  static inline std::atomic<bool> has_floating_base{false};
  // ReadBaseState() calls CM has made. The withheld-capability test asserts
  // this at 0 so the READ gate has an assertion of its own: without it, a copy
  // block made unconditional and a read gate removed are each invisible
  // alone, because an un-read cache copied through is all zeros too.
  static inline std::atomic<int> read_base_calls{0};
  static inline std::atomic<int64_t> base_recv_steady_ns{0};
  static inline std::atomic<uint64_t> base_sequence{0};
  static inline std::atomic<bool> base_valid{false};
  static constexpr std::array<double, kBasePoseDim> kBasePose{0.125,    -0.25,  0.9375,    0.015625,
                                                              -0.03125, 0.0625, 0.99609375};
  static constexpr std::array<double, kBaseTwistDim> kBaseTwist{1.5,   -2.5,  3.5,
                                                                -0.75, 0.875, -0.9375};
  static constexpr std::array<double, kImuChannelsPerSite> kImuSite1{0.1,  -0.2,  0.3,
                                                                     9.81, -0.01, 0.02};

  // IMU lane. Opt-in per test like the base lane, for the same two reasons.
  // Site 0's VALUES are left zero and only site 1's are filled, so a copy
  // bounded to one site is visible at the far end; the per-site stamp, counter
  // and flag are distinct per site (set by the test), so a copy that crosses
  // sites is visible too. `imu_num_sites` is the count ReadImuState reports —
  // a test sets an out-of-contract value to exercise the RT loop's bound.
  static inline std::atomic<bool> has_imu_state{false};
  static inline std::atomic<int> read_imu_calls{0};
  static inline std::atomic<int> imu_num_sites{2};
  static inline std::array<std::atomic<int64_t>, kMaxImuSites> imu_recv_steady_ns{};
  static inline std::array<std::atomic<uint64_t>, kMaxImuSites> imu_sequence{};
  static inline std::array<std::atomic<bool>, kMaxImuSites> imu_valid{};

  static void ResetStateChannels() {
    state_num_channels.store(2, std::memory_order_relaxed);
    state_position_nan.store(false, std::memory_order_relaxed);
    state_hole_mask.store(0, std::memory_order_relaxed);
    state_velocity_hole_mask.store(0, std::memory_order_relaxed);
    state_effort_hole_mask.store(0, std::memory_order_relaxed);
    sensor_recv_steady_ns_g1.store(0, std::memory_order_relaxed);
    sensor_sequence_g1.store(0, std::memory_order_relaxed);
    has_floating_base.store(false, std::memory_order_relaxed);
    read_base_calls.store(0, std::memory_order_relaxed);
    base_recv_steady_ns.store(0, std::memory_order_relaxed);
    base_sequence.store(0, std::memory_order_relaxed);
    base_valid.store(false, std::memory_order_relaxed);
    joint_recv_steady_ns.store(0, std::memory_order_relaxed);
    joint_sequence.store(0, std::memory_order_relaxed);
    has_imu_state.store(false, std::memory_order_relaxed);
    read_imu_calls.store(0, std::memory_order_relaxed);
    imu_num_sites.store(2, std::memory_order_relaxed);
    for (auto& v : imu_recv_steady_ns)
      v.store(0, std::memory_order_relaxed);
    for (auto& v : imu_sequence)
      v.store(0, std::memory_order_relaxed);
    for (auto& v : imu_valid)
      v.store(false, std::memory_order_relaxed);
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
    cache.hole_mask = state_hole_mask.load(std::memory_order_relaxed);
    cache.velocity_hole_mask = state_velocity_hole_mask.load(std::memory_order_relaxed);
    cache.effort_hole_mask = state_effort_hole_mask.load(std::memory_order_relaxed);
    cache.joint_recv_steady_ns = joint_recv_steady_ns.load(std::memory_order_relaxed);
    cache.joint_sequence = joint_sequence.load(std::memory_order_relaxed);
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
    // TWO groups, not one: the receipt/sequence assertions below read group 1,
    // and a copy bounded by `num_inference_groups` would silently drop it if
    // this stub kept claiming a single group.
    cache.num_inference_groups = 2;
    cache.inference_data[0] = kInfer0;
    cache.inference_enable[0] = true;
    cache.inference_recv_steady_ns[1] = sensor_recv_steady_ns_g1.load(std::memory_order_relaxed);
    cache.inference_sequence[1] = sensor_sequence_g1.load(std::memory_order_relaxed);
  }

  bool HasFloatingBase() const noexcept override {
    return has_floating_base.load(std::memory_order_relaxed);
  }

  void ReadBaseState(DeviceStateCache& cache) noexcept override {
    read_base_calls.fetch_add(1, std::memory_order_relaxed);
    cache.base_pose = kBasePose;
    cache.base_twist = kBaseTwist;
    cache.num_imu_sites = 2;
    cache.imu[1] = kImuSite1;
    cache.base_recv_steady_ns = base_recv_steady_ns.load(std::memory_order_relaxed);
    cache.base_sequence = base_sequence.load(std::memory_order_relaxed);
    cache.base_valid = base_valid.load(std::memory_order_relaxed);
  }

  bool HasImuState() const noexcept override {
    return has_imu_state.load(std::memory_order_relaxed);
  }

  void ReadImuState(DeviceStateCache& cache) noexcept override {
    read_imu_calls.fetch_add(1, std::memory_order_relaxed);
    cache.num_imu_sites = imu_num_sites.load(std::memory_order_relaxed);
    cache.imu[0] = {};
    cache.imu[1] = kImuSite1;
    for (std::size_t s = 0; s < cache.imu_recv_steady_ns.size(); ++s) {
      cache.imu_recv_steady_ns[s] = imu_recv_steady_ns[s].load(std::memory_order_relaxed);
      cache.imu_sequence[s] = imu_sequence[s].load(std::memory_order_relaxed);
      cache.imu_valid[s] = imu_valid[s].load(std::memory_order_relaxed);
    }
  }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType command_type) noexcept override {
    // The mode is recorded, not ignored (issue #342). This backend keeps
    // AcceptsCommandType's position-only default, so a kTorque argument
    // arriving here IS the defect — a counter is the only way an assertion can
    // say "it never reached the wire" rather than "it was not the last thing
    // to reach the wire".
    last_command_type_ = command_type;
    if (command_type == CommandType::kTorque) {
      torque_write_count_.fetch_add(1, std::memory_order_relaxed);
    }
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

  CommandType LastCommandType() const { return last_command_type_; }

  /// Ticks on which this position-only backend was handed kTorque — the count
  /// the #342 guard must hold at zero for the whole run.
  int TorqueWriteCount() const { return torque_write_count_.load(std::memory_order_relaxed); }

  const std::string& GroupName() const { return group_name_; }

 private:
  std::string group_name_;
  std::atomic<int> write_count_{0};
  std::atomic<int> safe_write_count_{0};
  std::atomic<int> torque_write_count_{0};
  std::atomic<std::int64_t> last_state_ns_{0};
  int last_num_channels_{0};
  int last_actual_num_channels_{0};
  CommandType last_command_type_{CommandType::kPosition};
  std::array<double, 2> last_commands_{};
  std::array<double, 2> last_actual_positions_{};
};

}  // namespace rtc
