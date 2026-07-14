// Shared test helpers for RtControllerNode unit tests (Tier 1).
//
// Defines the friend accessor `rtc::ControllerLifecycleTestAccess` (name fixed
// by the friend declaration in rt_controller_node.hpp:71) plus a reusable
// MockController. Included only by NEW test files (test_estop.cpp,
// test_device_target_callback.cpp). The pre-existing test_controller_lifecycle
// .cpp / test_switch_service.cpp keep their own inline copies — each gtest
// binary is a separate TU, so the redefinition is ODR-safe across executables.
#pragma once

#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_msgs/msg/robot_target.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace rtc {

// Minimal RTControllerInterface stub. Records lifecycle / E-STOP / target
// dispatch so tests can assert CM forwarding behaviour.
class MockController : public RTControllerInterface {
 public:
  explicit MockController(std::string name) : name_(std::move(name)) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override {
    activate_count_.fetch_add(1, std::memory_order_relaxed);
    return RTControllerInterface::on_activate(prev);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*prev*/) noexcept override {
    deactivate_count_.fetch_add(1, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
  }

  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    return ControllerOutput{};
  }

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override {
    last_target_slot_ = device_idx;
    last_target_.assign(target.begin(), target.end());
    set_target_count_.fetch_add(1, std::memory_order_relaxed);
  }

  std::string_view Name() const noexcept override { return name_; }

  void TriggerEstop() noexcept override {
    trigger_estop_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void ClearEstop() noexcept override {
    clear_estop_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void SetHandEstop(bool enabled) noexcept override {
    hand_estop_.store(enabled, std::memory_order_relaxed);
    set_hand_estop_count_.fetch_add(1, std::memory_order_relaxed);
  }

  int ActivateCount() const { return activate_count_.load(std::memory_order_relaxed); }

  int DeactivateCount() const { return deactivate_count_.load(std::memory_order_relaxed); }

  int TriggerEstopCount() const { return trigger_estop_count_.load(std::memory_order_relaxed); }

  int ClearEstopCount() const { return clear_estop_count_.load(std::memory_order_relaxed); }

  int SetHandEstopCount() const { return set_hand_estop_count_.load(std::memory_order_relaxed); }

  bool HandEstop() const { return hand_estop_.load(std::memory_order_relaxed); }

  int SetTargetCount() const { return set_target_count_.load(std::memory_order_relaxed); }

  int LastTargetSlot() const { return last_target_slot_; }

  const std::vector<double>& LastTarget() const { return last_target_; }

 private:
  std::string name_;
  std::atomic<int> activate_count_{0};
  std::atomic<int> deactivate_count_{0};
  std::atomic<int> trigger_estop_count_{0};
  std::atomic<int> clear_estop_count_{0};
  std::atomic<int> set_hand_estop_count_{0};
  std::atomic<int> set_target_count_{0};
  std::atomic<bool> hand_estop_{false};
  int last_target_slot_{-1};
  std::vector<double> last_target_;
};

// Friend accessor — lives in rtc:: to match the friend declaration. Provides
// the private-member injection / call wrappers used by Tier 1 tests.
class ControllerLifecycleTestAccess {
 public:
  static void InjectControllers(RtControllerNode& node,
                                std::vector<std::unique_ptr<RTControllerInterface>> ctrls,
                                const std::vector<std::string>& types) {
    node.controllers_ = std::move(ctrls);
    node.controller_states_ = std::vector<std::atomic<int>>(node.controllers_.size());
    node.controller_topic_configs_.assign(node.controllers_.size(), {});
    node.controller_slot_mappings_.assign(node.controllers_.size(), {});
    node.controller_name_to_idx_.clear();
    node.controller_types_.clear();
    for (std::size_t i = 0; i < node.controllers_.size(); ++i) {
      node.controller_name_to_idx_[std::string(node.controllers_[i]->Name())] = static_cast<int>(i);
      node.controller_types_.push_back(i < types.size() ? types[i] : "");
    }
  }

  static void SetActiveIdx(RtControllerNode& node, int idx) {
    node.active_controller_idx_.store(idx, std::memory_order_release);
  }

  // ── System model config (URDF / Extended-URDF closure resolution) ───────────
  // Exposes the closure sidecar path that DeclareAndLoadParameters resolved, so
  // tests can assert the Extended-URDF branch behaviour (set when a sidecar
  // exists, empty when missing/disabled) independent of which URDF source
  // branch — top-level or devices-fallback — won.
  static std::string GetResolvedClosureYamlPath(const RtControllerNode& node) {
    return node.system_model_config_.closure_yaml_path;
  }

  // ── E-STOP ────────────────────────────────────────────────────────────────
  // estop_pub_ is a plain (non-lifecycle) publisher, so PublishEstopStatus()
  // works without driving on_activate.
  static void EnsureEstopPublisher(RtControllerNode& node) {
    if (!node.estop_pub_) {
      node.estop_pub_ = node.create_publisher<std_msgs::msg::Bool>("/rtc_cm/estop", 1);
    }
  }

  static void CallTriggerEstop(RtControllerNode& node, std::string_view reason) {
    node.TriggerGlobalEstop(reason);
  }

  static void CallClearEstop(RtControllerNode& node) { node.ClearGlobalEstop(); }

  static bool IsEstopped(const RtControllerNode& node) { return node.IsGlobalEstopped(); }

  static std::string GetEstopReason(const RtControllerNode& node) {
    return std::string(node.estop_reason_.data());
  }

  static bool GetEstopLogPending(const RtControllerNode& node) {
    return node.estop_log_pending_.load(std::memory_order_acquire);
  }

  static void ClearEstopLogPending(RtControllerNode& node) {
    node.estop_log_pending_.store(false, std::memory_order_release);
  }

  // ── Device-timeout watchdog ────────────────────────────────────────────────
  static void AddDeviceTimeout(RtControllerNode& node, const std::string& group,
                               std::chrono::milliseconds timeout, bool received,
                               std::chrono::steady_clock::time_point last_update) {
    node.device_timeouts_.emplace_back();
    auto& e = node.device_timeouts_.back();
    e.group_name = group;
    e.timeout = timeout;
    e.last_update = last_update;
    e.received.store(received, std::memory_order_relaxed);
  }

  static void CallCheckTimeouts(RtControllerNode& node) { node.CheckTimeouts(); }

  static bool CallAllTimeoutDevicesReceived(const RtControllerNode& node) {
    return node.AllTimeoutDevicesReceived();
  }

  static void InjectDeviceNameConfigs(RtControllerNode& node,
                                      std::map<std::string, DeviceNameConfig> configs) {
    node.device_name_configs_ = std::move(configs);
  }

  // ── Device-backend creation (CreateDeviceBackends / PropagateCapabilities) ──
  static void EnsureRtCallbackGroup(RtControllerNode& node) {
    if (!node.cb_group_rt_callback_) {
      node.cb_group_rt_callback_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
  }

  static void SetGroupSlotMap(RtControllerNode& node, std::map<std::string, int> m) {
    node.group_slot_map_ = std::move(m);
  }

  static void CallCreateDeviceBackends(RtControllerNode& node) { node.CreateDeviceBackends(); }

  static void CallPropagateCapabilities(RtControllerNode& node) {
    node.PropagateCapabilitiesIntoMappings();
  }

  // Inject one controller slot mapping (num_groups=1, slots[0]=slot0) so the
  // PropagateCapabilitiesIntoMappings inner loop has work to patch.
  static void AddControllerSlotMapping(RtControllerNode& node, int slot0) {
    RtControllerNode::ControllerSlotMapping m;
    m.num_groups = 1;
    m.slots[0] = slot0;
    node.controller_slot_mappings_.push_back(m);
  }

  static std::uint16_t GetSlotCapability(const RtControllerNode& node, std::size_t slot) {
    return slot < node.slot_to_capability_.size() ? node.slot_to_capability_[slot]
                                                  : std::uint16_t{0};
  }

  static std::uint16_t GetMappingCapability(const RtControllerNode& node, std::size_t ctrl_idx,
                                            std::size_t group_idx) {
    return node.controller_slot_mappings_[ctrl_idx].capabilities[group_idx];
  }

  static DeviceBackend* GetBackend(RtControllerNode& node, std::size_t slot) {
    return slot < static_cast<std::size_t>(RtControllerNode::kMaxDevices)
               ? node.backends_[slot].get()
               : nullptr;
  }

  static int MaxDevices() { return RtControllerNode::kMaxDevices; }

  static bool GetStateReceived(const RtControllerNode& node) {
    return node.state_received_.load(std::memory_order_acquire);
  }

  static void CallCreateDigitalTwinPublishers(RtControllerNode& node) {
    node.CreateDigitalTwinPublishers();
  }

  // ── Log drain (DrainLog) ────────────────────────────────────────────────────
  static void CallDrainLog(RtControllerNode& node) { node.DrainLog(); }

  static void SetPrintTimingSummary(RtControllerNode& node, bool v) {
    node.print_timing_summary_.store(v, std::memory_order_relaxed);
  }
};

}  // namespace rtc
