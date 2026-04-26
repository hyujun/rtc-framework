#include "rtc_controller_interface/controller_registry.hpp"

#include <algorithm>

#include <rclcpp/logging.hpp>

namespace rtc {

ControllerRegistry &ControllerRegistry::Instance() noexcept {
  static ControllerRegistry instance;
  return instance;
}

void ControllerRegistry::Register(ControllerEntry entry) {
  // Warn (don't throw) on duplicate config_key — second registration shadows
  // the first when CM looks up by key, which is almost always a packaging
  // mistake. Static-init ordering across TUs makes throwing fragile.
  const auto it = std::find_if(entries_.begin(), entries_.end(),
                               [&](const ControllerEntry &e) {
                                 return e.config_key == entry.config_key;
                               });
  if (it != entries_.end()) {
    RCLCPP_WARN(
        rclcpp::get_logger("rtc_controller_interface"),
        "ControllerRegistry: duplicate config_key '%s' — second registration "
        "(package='%s') will shadow the first (package='%s'). Check that the "
        "key is unique across all RTC_REGISTER_CONTROLLER macros in linked "
        "TUs.",
        entry.config_key.c_str(), entry.config_package.c_str(),
        it->config_package.c_str());
  }
  entries_.push_back(std::move(entry));
}

} // namespace rtc
