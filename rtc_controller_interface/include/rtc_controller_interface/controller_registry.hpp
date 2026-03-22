#ifndef RTC_CONTROLLER_INTERFACE_CONTROLLER_REGISTRY_H_
#define RTC_CONTROLLER_INTERFACE_CONTROLLER_REGISTRY_H_

#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rtc
{

// ── Controller entry for the plugin registry ─────────────────────────────────
//
// Each entry describes how to instantiate one controller type and where to
// find its YAML configuration.
//
//   config_key     — YAML filename stem, e.g. "demo_joint_controller"
//   config_subdir  — "direct/" for torque controllers, "indirect/" for position
//   config_package — ament package that owns the config YAML
//   factory        — callable: (urdf_path) → unique_ptr<RTControllerInterface>
struct ControllerEntry
{
  std::string config_key;
  std::string config_subdir;
  std::string config_package;
  std::function<std::unique_ptr<RTControllerInterface>(const std::string &)> factory;
};

// ── Singleton controller registry ────────────────────────────────────────────
//
// External packages register their controllers at static-init time (before
// main) via RTC_REGISTER_CONTROLLER.  RtControllerNode queries the registry
// once during DeclareAndLoadParameters().
//
// Thread safety: all Register() calls happen during static init or before
// the RT loop starts.  GetEntries() is called once at startup.
class ControllerRegistry
{
public:
  static ControllerRegistry & Instance() noexcept;

  void Register(ControllerEntry entry);

  [[nodiscard]] const std::vector<ControllerEntry> & GetEntries() const noexcept
  {
    return entries_;
  }

private:
  ControllerRegistry() = default;
  std::vector<ControllerEntry> entries_;
};

}  // namespace rtc

// ── Registration macro ───────────────────────────────────────────────────────
//
// Usage (in a .cpp file):
//
//   RTC_REGISTER_CONTROLLER(
//     demo_joint_controller,          // config_key (unquoted)
//     "indirect/",                    // config_subdir
//     "ur5e_bringup",                // ament package name
//     std::make_unique<MyCtrl>(urdf)  // factory expression (urdf is in scope)
//   )
//
// The macro creates a file-scope bool whose initializer calls Register().
// A companion ForceXxxRegistration() function must be defined in the same
// translation unit and called from main() to prevent the linker from
// stripping the TU when linking static libraries.

#define RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, FactoryExpr) \
  namespace {                                                                           \
  [[maybe_unused]] const bool rtc_reg_##config_key = [] {                               \
    ::rtc::ControllerRegistry::Instance().Register({                                    \
      #config_key, config_subdir, config_package,                                       \
      [](const std::string & urdf) { return FactoryExpr; }                              \
    });                                                                                 \
    return true;                                                                        \
  }();                                                                                  \
  }  // anonymous namespace

#endif  // RTC_CONTROLLER_INTERFACE_CONTROLLER_REGISTRY_H_
