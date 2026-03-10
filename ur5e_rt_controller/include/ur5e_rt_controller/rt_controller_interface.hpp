#ifndef UR5E_RT_CONTROLLER_RT_CONTROLLER_INTERFACE_H_
#define UR5E_RT_CONTROLLER_RT_CONTROLLER_INTERFACE_H_

// Shared types (constants, data structs) live in ur5e_rt_base.
// This header re-exports them and adds the abstract Strategy interface.
#include "ur5e_rt_base/types.hpp"

#include <yaml-cpp/yaml.h>

#include <span>
#include <string_view>

namespace ur5e_rt_controller
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
    std::span<const double, kNumHandJoints> target) noexcept = 0;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

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
  virtual void LoadConfig(const YAML::Node & cfg) {(void)cfg;}
  virtual void UpdateGainsFromMsg(std::span<const double> gains) noexcept
  {
    (void)gains;
  }

protected:
  RTControllerInterface() = default;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_RT_CONTROLLER_INTERFACE_H_
