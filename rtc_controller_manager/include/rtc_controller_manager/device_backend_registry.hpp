#ifndef RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_REGISTRY_H_
#define RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_REGISTRY_H_

#include "rtc_controller_manager/device_backend.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rtc {

// ── DeviceBackendEntry ───────────────────────────────────────────────────────
//
// One entry per registered backend type. Mirrors the ControllerRegistry
// pattern in rtc_controller_interface.
//
//   type_tag — string used in YAML `devices.<group>.backend.type`
//              (e.g. "mujoco_native", "ur_driver_native", "udp_hand_native")
//   factory  — callable: () → unique_ptr<DeviceBackend>
struct DeviceBackendEntry {
  std::string type_tag;
  std::function<std::unique_ptr<DeviceBackend>()> factory;
};

// ── DeviceBackendRegistry ────────────────────────────────────────────────────
//
// Singleton. External packages (integrated_bringup) register backend types at
// static-init time via RTC_REGISTER_DEVICE_BACKEND. CM looks up by type_tag
// during on_configure (not on the RT path).
//
// Thread safety: all Register() calls happen during static init or before
// the RT loop starts. Lookup is read-only after init.
class DeviceBackendRegistry {
 public:
  static DeviceBackendRegistry& Instance() noexcept;

  void Register(DeviceBackendEntry entry);

  [[nodiscard]] const std::vector<DeviceBackendEntry>& GetEntries() const noexcept {
    return entries_;
  }

  /// Look up a registered backend by type tag. Returns nullptr if no entry
  /// matches (caller logs / fails configure).
  [[nodiscard]] const DeviceBackendEntry* Find(const std::string& type_tag) const noexcept;

  /// Convenience: construct a new backend instance for the given type tag.
  /// Returns nullptr if no entry matches.
  [[nodiscard]] std::unique_ptr<DeviceBackend> Create(const std::string& type_tag) const;

 private:
  DeviceBackendRegistry() = default;
  std::vector<DeviceBackendEntry> entries_;
};

}  // namespace rtc

// ── Registration macro ───────────────────────────────────────────────────────
//
// Usage (in a .cpp file inside the bringup package that owns the backend):
//
//   RTC_REGISTER_DEVICE_BACKEND(
//     mujoco_native,                              // type_tag (unquoted)
//     std::make_unique<rtc::MujocoNativeBackend>()
//   )
//
// As with RTC_REGISTER_CONTROLLER, a companion ForceXxxRegistration() function
// must be defined in the same TU and called from main() to prevent the linker
// from stripping the TU when linking static libraries.

#define RTC_REGISTER_DEVICE_BACKEND(type_tag, FactoryExpr)                                      \
  namespace {                                                                                   \
  [[maybe_unused]] const bool rtc_reg_device_backend_##type_tag = [] {                          \
    ::rtc::DeviceBackendRegistry::Instance().Register({#type_tag, [] { return FactoryExpr; }}); \
    return true;                                                                                \
  }();                                                                                          \
  }  // anonymous namespace

#endif  // RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_REGISTRY_H_
