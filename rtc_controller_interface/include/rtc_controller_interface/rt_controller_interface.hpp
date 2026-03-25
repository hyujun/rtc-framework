#ifndef RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
#define RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_

// Shared types (constants, data structs) live in rtc_base.
// This header re-exports them and adds the abstract Strategy interface.
#include "rtc_base/types/types.hpp"

#include <yaml-cpp/yaml.h>

#include <map>
#include <span>
#include <string_view>
#include <vector>

namespace rtc
{

// ── Abstract interface (Strategy Pattern) ─────────────────────────────────────
//
// All virtual methods are noexcept to guarantee real-time safety: any
// exception thrown inside a 500 Hz timer would terminate the process.
class RTControllerInterface {
public:
  virtual ~RTControllerInterface() = default;

  RTControllerInterface(const RTControllerInterface &)            = delete;
  RTControllerInterface & operator=(const RTControllerInterface &) = delete;
  RTControllerInterface(RTControllerInterface &&)                 = delete;
  RTControllerInterface & operator=(RTControllerInterface &&)      = delete;

  // Compute one control step. Must be noexcept for RT safety.
  [[nodiscard]] virtual ControllerOutput Compute(
    const ControllerState & state) noexcept = 0;

  virtual void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept = 0;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

  // Auto-hold: initialise target from current state so the robot holds
  // its position when no external goal has been received.
  // Each controller implements this according to its own target format
  // (joint-space, task-space, etc.).  Called once from ControlLoop Phase 0
  // when state_received_ is true but target_received_ is false.
  virtual void InitializeHoldPosition(
    const ControllerState & state) noexcept = 0;

  // E-STOP interface — default no-ops for controllers that do not need it.
  virtual void TriggerEstop() noexcept                      {}
  virtual void ClearEstop() noexcept                        {}
  [[nodiscard]] virtual bool IsEstopped() const noexcept    {return false;}
  virtual void SetHandEstop(bool /*enabled*/) noexcept      {}

  // ── Extensibility hooks for the controller registry ──────────────────────
  //
  // LoadConfig()
  //   Called once at node startup.  `cfg` is the YAML node already scoped to
  //   this controller's key (e.g. the content under `pd_controller:` in its
  //   YAML file).  Override to read per-controller gains / flags from disk.
  //   Not noexcept — YAML parsing can throw; the call site wraps it in try/catch.
  //
  // UpdateGainsFromMsg()
  //   Called from the ~/controller_gains subscriber (sensor thread).
  //   `gains` is a flat array whose layout is controller-specific:
  //   document the layout in the controller's header.  Default is a no-op.
  // Returns the type of command this controller outputs (position or torque).
  // Value is set by LoadConfig() from the YAML command_type field.
  [[nodiscard]] virtual CommandType GetCommandType() const noexcept {
    return CommandType::kPosition;
  }

  virtual void LoadConfig(const YAML::Node & cfg);
  virtual void UpdateGainsFromMsg(std::span<const double> gains) noexcept
  {
    (void)gains;
  }

  // GetCurrentGains()
  //   Returns the current gains as a flat array matching the layout expected
  //   by UpdateGainsFromMsg().  Used by the GUI "Load Gain" feature to read
  //   back the active controller's runtime gains.  Default returns empty.
  [[nodiscard]] virtual std::vector<double> GetCurrentGains() const noexcept
  {
    return {};
  }

  // GetTopicConfig()
  //   Returns the per-controller topic configuration (subscribe/publish topics).
  //   Populated by LoadConfig() from the YAML "topics" section.
  //   If no "topics" section exists, returns the default topic set.
  [[nodiscard]] const TopicConfig & GetTopicConfig() const noexcept
  {
    return topic_config_;
  }

  // ── Device name configuration ──────────────────────────────────────────
  //   SetDeviceNameConfigs() is called by RtControllerNode after all
  //   controllers are constructed and device configs are loaded from YAML.
  //   After setting, OnDeviceConfigsSet() is called for controllers to
  //   resolve kinematics (e.g. end_id_ from tip_link).
  void SetDeviceNameConfigs(std::map<std::string, DeviceNameConfig> configs)
  {
    device_name_configs_ = std::move(configs);
    OnDeviceConfigsSet();
  }

  [[nodiscard]] const DeviceNameConfig* GetDeviceNameConfig(
      const std::string& device_name) const noexcept
  {
    auto it = device_name_configs_.find(device_name);
    return (it != device_name_configs_.end()) ? &it->second : nullptr;
  }

  // Returns the name of the primary device (first group in topic config).
  // Use instead of hardcoding "ur5e" to support arbitrary robot names.
  [[nodiscard]] std::string GetPrimaryDeviceName() const noexcept
  {
    if (!topic_config_.groups.empty()) {
      return topic_config_.groups.front().first;
    }
    if (!device_name_configs_.empty()) {
      return device_name_configs_.begin()->first;
    }
    return {};
  }

  // Set the control loop rate (Hz). Called by the manager at init time.
  void SetControlRate(double hz) noexcept { control_rate_ = hz; }
  [[nodiscard]] double GetDefaultDt() const noexcept {
    return (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.002;
  }

protected:
  RTControllerInterface();

  // Called after SetDeviceNameConfigs(). Override to resolve URDF-based
  // kinematics (e.g. tip_link → end_id_).
  virtual void OnDeviceConfigsSet() {}

  // Parses the "topics" section of a controller YAML node.
  // Called by the base LoadConfig(); subclasses that override LoadConfig()
  // should call RTControllerInterface::LoadConfig(cfg) to inherit this.
  static TopicConfig ParseTopicConfig(const YAML::Node & topics_node);

  // Default topic configuration — device_name determines topic namespace.
  static TopicConfig MakeDefaultTopicConfig(const std::string& device_name = "ur5e");

  TopicConfig topic_config_;
  std::map<std::string, DeviceNameConfig> device_name_configs_;
  double control_rate_{500.0};
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
