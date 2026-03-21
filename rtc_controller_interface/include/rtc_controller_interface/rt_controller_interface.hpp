#ifndef RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
#define RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_

// Shared types (constants, data structs) live in rtc_base.
// This header re-exports them and adds the abstract Strategy interface.
#include "rtc_base/types/types.hpp"

#include <yaml-cpp/yaml.h>

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

  virtual void SetRobotTarget(
    std::span<const double, kNumRobotJoints> target) noexcept = 0;

  virtual void SetHandTarget(
    std::span<const float, kNumHandMotors> target) noexcept = 0;

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

  // GetPerControllerDeviceFlags()
  //   Returns per-controller device enable/disable overrides.
  //   nullopt = inherit from global.
  [[nodiscard]] const PerControllerDeviceFlags & GetPerControllerDeviceFlags()
    const noexcept
  {
    return per_controller_device_flags_;
  }

protected:
  RTControllerInterface();

  // Parses the "topics" section of a controller YAML node.
  // Called by the base LoadConfig(); subclasses that override LoadConfig()
  // should call RTControllerInterface::LoadConfig(cfg) to inherit this.
  static TopicConfig ParseTopicConfig(const YAML::Node & topics_node);

  // Default topic configuration — matches the original hard-coded topics.
  static TopicConfig MakeDefaultTopicConfig();

  TopicConfig topic_config_;
  PerControllerDeviceFlags per_controller_device_flags_;
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
