#include "rtc_controller_manager/device_backend_registry.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <utility>

namespace rtc {

DeviceBackendRegistry& DeviceBackendRegistry::Instance() noexcept {
  static DeviceBackendRegistry instance;
  return instance;
}

void DeviceBackendRegistry::Register(DeviceBackendEntry entry) {
  // Warn (don't throw) on duplicate type_tag — second registration shadows
  // the first when CM looks up by tag, which is almost always a packaging
  // mistake. Static-init ordering across TUs makes throwing fragile.
  const auto it = std::find_if(entries_.begin(), entries_.end(), [&](const DeviceBackendEntry& e) {
    return e.type_tag == entry.type_tag;
  });
  if (it != entries_.end()) {
    RCLCPP_WARN(rclcpp::get_logger("rtc_controller_manager"),
                "DeviceBackendRegistry: duplicate type_tag '%s' — second registration "
                "will shadow the first. Check that the tag is unique across all "
                "RTC_REGISTER_DEVICE_BACKEND macros in linked TUs.",
                entry.type_tag.c_str());
  }
  entries_.push_back(std::move(entry));
}

const DeviceBackendEntry* DeviceBackendRegistry::Find(const std::string& type_tag) const noexcept {
  const auto it = std::find_if(entries_.begin(), entries_.end(),
                               [&](const DeviceBackendEntry& e) { return e.type_tag == type_tag; });
  return it != entries_.end() ? &*it : nullptr;
}

std::unique_ptr<DeviceBackend> DeviceBackendRegistry::Create(const std::string& type_tag) const {
  const auto* entry = Find(type_tag);
  if (entry == nullptr || !entry->factory) {
    return nullptr;
  }
  return entry->factory();
}

}  // namespace rtc
