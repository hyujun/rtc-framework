#ifndef RTC_CONTROLLER_INTERFACE_CONTROLLER_REGISTRY_H_
#define RTC_CONTROLLER_INTERFACE_CONTROLLER_REGISTRY_H_

#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rtc {

// ── Controller entry for the plugin registry ─────────────────────────────────
//
// Each entry describes how to instantiate one controller type and where to
// find its YAML configuration.
//
//   config_key     — YAML filename stem, e.g. "demo_joint_controller"
//   config_subdir  — "direct/" for torque controllers, "indirect/" for position
//   config_package — ament package that owns the config YAML
//   factory        — callable: (urdf_path) → unique_ptr<RTControllerInterface>
//   config_required — see below
struct ControllerEntry {
  std::string config_key;
  std::string config_subdir;
  std::string config_package;
  std::function<std::unique_ptr<RTControllerInterface>(const std::string&)> factory;

  // True for a controller that has no meaningful built-in defaults, so an
  // absent config file means "this robot does not run this controller" rather
  // than "run it on defaults" (issue #196 decision D2, which stays the default).
  //
  // One binary serves every config_variant and registration happens at
  // static-init, before any parameter exists, so a controller cannot decline to
  // register for a robot. This flag is how it declines to be *instantiated*:
  // the CM skips the entry when the variant ships no YAML for it. Without it
  // such a controller fails PreConfigure and refuses the whole bring-up,
  // taking down every other controller on robots that never wanted it.
  //
  // It does NOT relax validation of a config that IS present: a file that
  // exists but is malformed, or whose top-level key is misspelled, still
  // refuses the configure. Only the file-absent case is affected.
  bool config_required = false;
};

// ── Singleton controller registry ────────────────────────────────────────────
//
// External packages register their controllers at static-init time (before
// main) via RTC_REGISTER_CONTROLLER.  RtControllerNode queries the registry
// once during DeclareAndLoadParameters().
//
// Thread safety: all Register() calls happen during static init or before
// the RT loop starts.  GetEntries() is called once at startup.
class ControllerRegistry {
 public:
  static ControllerRegistry& Instance() noexcept;

  void Register(ControllerEntry entry);

  [[nodiscard]] const std::vector<ControllerEntry>& GetEntries() const noexcept { return entries_; }

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
//     "integrated_bringup",                // ament package name
//     std::make_unique<MyCtrl>(urdf)  // factory expression (urdf is in scope)
//   )
//
// The macro creates a file-scope bool whose initializer calls Register().
// A companion ForceXxxRegistration() function must be defined in the same
// translation unit and called from main() to prevent the linker from
// stripping the TU when linking static libraries.

#define RTC_REGISTER_CONTROLLER_IMPL(config_key, config_subdir, config_package, FactoryExpr, \
                                     ConfigRequired)                                         \
  namespace {                                                                                \
  [[maybe_unused]] const bool rtc_reg_##config_key = [] {                                    \
    ::rtc::ControllerRegistry::Instance().Register(                                          \
        {#config_key, config_subdir, config_package,                                         \
         [](const std::string& urdf) { return FactoryExpr; }, ConfigRequired});              \
    return true;                                                                             \
  }();                                                                                       \
  }  // anonymous namespace

#define RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, FactoryExpr) \
  RTC_REGISTER_CONTROLLER_IMPL(config_key, config_subdir, config_package, FactoryExpr, false)

// Same as RTC_REGISTER_CONTROLLER, but the controller is skipped on a
// config_variant that ships no YAML for it instead of refusing the bring-up.
// Use this ONLY when built-in defaults cannot produce a runnable controller
// (e.g. a policy controller with no model path) — see ControllerEntry above.
#define RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG(config_key, config_subdir, config_package, \
                                                 FactoryExpr)                               \
  RTC_REGISTER_CONTROLLER_IMPL(config_key, config_subdir, config_package, FactoryExpr, true)

#endif  // RTC_CONTROLLER_INTERFACE_CONTROLLER_REGISTRY_H_
